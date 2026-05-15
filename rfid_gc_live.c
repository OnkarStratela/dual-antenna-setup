// rfid_gc_live.c
// Live-state RFID scanner using two antennas (Source_0 and Source_1).
//
// Every sweep (continuous), prints a single line while scanning runs.
// The output uses FIXED SLOTS so tags never shift positions:
//
// Empty (no tags in range — shows activity without extra noise):
//   []
//
// Tags visible (slot 0 is always antenna 0, slot 1 is always antenna 1).
// Per-tag RSSI in dBm is printed in brackets right after the antenna
// number. Empty slots render as pure whitespace so the comma and the
// other slot never shift columns:
//   [TX=30 mW] [(0)(-45) E2801160600002054E1A1234,   (1)(-52) E2801160600002054E1A5678]
//   [TX=30 mW] [(0)(-45) E2801160600002054E1A1234,                                    ]
//   [TX=30 mW] [                                  ,   (1)(-52) E2801160600002054E1A5678]
//
// Cross-read arbitration:
//   The two antennas' fields overlap above the drip trays, so the same
//   EPC often appears on the "wrong" antenna. To filter that out, every
//   EPC is tracked across sweeps with an exponential moving average of
//   its RSSI on each antenna. The antenna with the higher long-term
//   average is the tag's "owner" and any read from a non-owner antenna
//   is suppressed (it never reaches the printed line). A small dB
//   hysteresis stops the owner from flicking back and forth when the
//   averages are close.
//   Reader-reported RSSI is in tenths of dBm (e.g. -650 == -65.0 dBm),
//   so RSSI_HYSTERESIS is also in tenths.
//
// Antenna index in YELLOW; Src0 tag EPC in GREEN, Src1 tag EPC in RED.
//
// Usage:
//   ./rfid_gc_live              -> both antennas at default power
//   ./rfid_gc_live <mW>         -> both antennas at <mW> (global power)
//   ./rfid_gc_live -h | --help  -> show usage

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <signal.h>

#include "CAENRFIDLib_Light.h"
#include "host.h"

// Configuration
#define GC_PORT             "/dev/ttyACM0"
#define GC_BAUDRATE         921600
#define DEFAULT_POWER_MW    30            // sensible default for ~7 cm read zone
#define MIN_POWER_MW        1             // reader rejects below its hardware floor
#define MAX_POWER_MW        316           // R3100C Lepton3 max (25 dBm)
#define GC_SCAN_MS          100           // ms between scan cycles (= line rate when printing every cycle)
#define GC_MAX_TAGS         64            // max tags merged across both antennas per sweep
#define ANTENNA_COUNT       2
#define MAX_ID_LENGTH       64
#define MAX_TRACKED_TAGS    32            // distinct EPCs tracked for the rolling-RSSI arbitration
#define RSSI_HYSTERESIS     30            // tenths of dBm (= 3.0 dB) gap needed to switch a tag's owner

// ANSI colours
#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define YELLOW "\033[0;33m"
#define CYAN   "\033[0;36m"
#define RESET  "\033[0m"

volatile int running = 0;

typedef struct {
    char    tag[2 * MAX_ID_LENGTH + 1];
    int     antenna;
    int16_t rssi;        /* tenths of dBm, as reported by the reader */
} TagEntry;

/* Rolling-RSSI history used to assign each EPC to a single antenna. */
typedef struct {
    char    epc[2 * MAX_ID_LENGTH + 1];
    int32_t rssi_avg[ANTENNA_COUNT];   /* EMA in tenths of dBm */
    bool    seen[ANTENNA_COUNT];
    int     owner;                     /* 0, 1, or -1 if not yet decided */
} TagHistory;

static TagHistory tag_hist[MAX_TRACKED_TAGS];
static int        tag_hist_count = 0;

static TagHistory *find_or_add_tag(const char *epc) {
    for (int i = 0; i < tag_hist_count; i++) {
        if (strcmp(tag_hist[i].epc, epc) == 0)
            return &tag_hist[i];
    }
    if (tag_hist_count >= MAX_TRACKED_TAGS) return NULL;
    TagHistory *h = &tag_hist[tag_hist_count++];
    snprintf(h->epc, sizeof h->epc, "%s", epc);
    for (int a = 0; a < ANTENNA_COUNT; a++) {
        h->rssi_avg[a] = 0;
        h->seen[a]     = false;
    }
    h->owner = -1;
    return h;
}

/* EMA: new = (old * 7 + sample) / 8  (alpha = 1/8). On first sample
   for this antenna we just seed the average with the sample itself. */
static void update_tag_history(int ant, const char *epc, int16_t rssi) {
    TagHistory *h = find_or_add_tag(epc);
    if (h == NULL) return;
    if (!h->seen[ant]) {
        h->rssi_avg[ant] = rssi;
        h->seen[ant]     = true;
    } else {
        h->rssi_avg[ant] = (h->rssi_avg[ant] * 7 + rssi) / 8;
    }
}

/* Returns the antenna (0 or 1) that currently owns this EPC, or -1 if
   we have no history (caller should pass through unfiltered). */
static int decide_owner(const char *epc) {
    TagHistory *h = find_or_add_tag(epc);
    if (h == NULL) return -1;
    if (h->seen[0] && !h->seen[1]) { h->owner = 0; return 0; }
    if (h->seen[1] && !h->seen[0]) { h->owner = 1; return 1; }
    int32_t diff = h->rssi_avg[0] - h->rssi_avg[1];
    if (h->owner == -1) {
        h->owner = (diff >= 0) ? 0 : 1;
    } else if (h->owner == 0 && diff < -RSSI_HYSTERESIS) {
        h->owner = 1;
    } else if (h->owner == 1 && diff >  RSSI_HYSTERESIS) {
        h->owner = 0;
    }
    return h->owner;
}

static void hex_str(uint8_t *bytes, uint16_t len, char *out) {
    for (int i = 0; i < len; i++)
        sprintf(out + (i * 2), "%02X", bytes[i]);
    out[len * 2] = '\0';
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s              both antennas at %d mW (default)\n"
        "  %s <mW>         both antennas at <mW> (global power)\n"
        "  %s -h | --help  show this message\n"
        "\nValid power range: %d..%d mW\n",
        prog, DEFAULT_POWER_MW, prog, prog,
        MIN_POWER_MW, MAX_POWER_MW);
}

static bool parse_power(const char *s, uint32_t *out) {
    if (s == NULL || *s == '\0') return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    if (v < MIN_POWER_MW || v > MAX_POWER_MW) return false;
    *out = (uint32_t)v;
    return true;
}

static void handle_sigint(int sig) {
    (void)sig;
    printf("\n" YELLOW "[GC] Stopping..." RESET "\n");
    running = 0;
}

// Width (visible chars, ignoring ANSI colour codes) reserved for each
// antenna slot inside the brackets. Comfortably fits
// "(N)(-XXX) " + a 24-char EPC-96 hex string; longer tags overflow
// without truncation.
#define SLOT_WIDTH 36

// Sweep output:
//   - bare [] when no antenna saw any tag
//   - otherwise: [TX=…] [<slot0>,   <slot1>]
//     Each slot is padded to SLOT_WIDTH. If an antenna missed, its slot
//     is pure whitespace (no "(N)"), so the comma and the other slot
//     keep their column positions and nothing shifts.
static void print_sweep_line(uint32_t power,
                             TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                             const int cnt[ANTENNA_COUNT])
{
    if (cnt[0] == 0 && cnt[1] == 0) {
        printf("[]\n");
        fflush(stdout);
        return;
    }

    printf(CYAN "[TX=%u mW]" RESET " [", (unsigned)power);

    for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
        if (ant > 0)
            printf(",   "); /* fixed separator between the two slots */

        if (cnt[ant] == 0) {
            printf("%*s", SLOT_WIDTH, "");
            continue;
        }

        /* Visible width of the slot content (excludes ANSI codes) so we
           can right-pad to SLOT_WIDTH and keep the ']' column stable. */
        int visible = 3; /* "(N)" */
        for (int i = 0; i < cnt[ant]; i++) {
            char rbuf[16];
            int rlen = snprintf(rbuf, sizeof rbuf,
                                "(%d) ", (int)bucket[ant][i].rssi);
            if (i > 0) visible += 1; /* space between multiple tags */
            visible += rlen + (int)strlen(bucket[ant][i].tag);
        }

        const char *tagcol = (ant == 0) ? GREEN : RED;
        printf(YELLOW "(%d)" RESET, ant);
        for (int i = 0; i < cnt[ant]; i++) {
            if (i > 0) printf(" ");
            printf("(%d) %s%s" RESET,
                   (int)bucket[ant][i].rssi,
                   tagcol, bucket[ant][i].tag);
        }
        int pad = SLOT_WIDTH - visible;
        if (pad > 0) printf("%*s", pad, "");
    }
    printf("]\n");
    fflush(stdout);
}

int main(int argc, char **argv) {

    uint32_t power = DEFAULT_POWER_MW;

    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        if (!parse_power(argv[1], &power)) {
            usage(argv[0]);
            return 1;
        }
    } else if (argc != 1) {
        usage(argv[0]);
        return 1;
    }

    CAENRFIDErrorCodes ec;
    CAENRFIDReader reader = {
        .connect       = _connect,
        .disconnect    = _disconnect,
        .tx            = _tx,
        .rx            = _rx,
        .clear_rx_data = _clear_rx_data,
        .enable_irqs   = _enable_irqs,
        .disable_irqs  = _disable_irqs
    };

    RS232_params port_params = {
        .com         = GC_PORT,
        .baudrate    = GC_BAUDRATE,
        .dataBits    = 8,
        .stopBits    = 1,
        .parity      = 0,
        .flowControl = 0,
    };

    const char *sources[ANTENNA_COUNT] = { "Source_0", "Source_1" };
    char model[64]  = {0};
    char serial[64] = {0};

    signal(SIGINT, handle_sigint);

    printf(CYAN "===== Dual-Antenna RFID Live Scanner =====" RESET "\n");
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %u mW (both antennas)\n", power);
    printf("Cycle     : %d ms\n", GC_SCAN_MS);
    printf("Antennas  : %s, %s\n", sources[0], sources[1]);
    printf("Arbitrate : on  (rolling-RSSI owner, %.1f dB hysteresis)\n\n",
           RSSI_HYSTERESIS / 10.0);

    printf("[GC] Connecting...\n");
    ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
    if (ec != CAENRFID_StatusOK) {
        printf("[GC] ERROR: Could not connect (code %d)\n", ec);
        printf("  - Check USB cable\n");
        printf("  - Try: sudo chmod 666 %s\n", GC_PORT);
        printf("  - Or:  sudo usermod -a -G dialout $USER  (then re-login)\n");
        return -1;
    }

    ec = CAENRFID_GetReaderInfo(&reader, model, serial);
    if (ec == CAENRFID_StatusOK)
        printf("[GC] Reader: %s  Serial: %s\n", model, serial);

    ec = CAENRFID_SetPower(&reader, power);
    if (ec != CAENRFID_StatusOK) {
        printf("[GC] WARNING: SetPower(%u) returned %d -- "
               "value may be below the reader's hardware floor.\n",
               power, ec);
    }
    printf("[GC] Ready. Empty sweeps print []. Tagged sweeps prepend [TX …]. Ctrl+C to stop.\n\n");

    running = 1;

    TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS];
    int      cnt[ANTENNA_COUNT];

    while (running) {

        cnt[0] = 0;
        cnt[1] = 0;

        for (int ant = 0; ant < ANTENNA_COUNT && running; ant++) {
            CAENRFIDTagList *tag_list = NULL, *node;
            uint16_t num_tags = 0;

            ec = CAENRFID_InventoryTag(&reader, (char *)sources[ant],
                                       0, 0, 0,
                                       NULL, 0,
                                       RSSI,
                                       &tag_list, &num_tags);

            if (ec == CAENRFID_StatusOK && num_tags > 0) {
                node = tag_list;
                while (node != NULL) {
                    if (cnt[ant] < GC_MAX_TAGS && cnt[0] + cnt[1] < GC_MAX_TAGS) {
                        hex_str(node->Tag.ID, node->Tag.Length,
                                bucket[ant][cnt[ant]].tag);
                        bucket[ant][cnt[ant]].antenna = ant;
                        bucket[ant][cnt[ant]].rssi    = node->Tag.RSSI;
                        cnt[ant]++;
                    }
                    CAENRFIDTagList *next = node->Next;
                    free(node);
                    node = next;
                }
            } else {
                // Free list if returned with non-OK code
                node = tag_list;
                while (node != NULL) {
                    CAENRFIDTagList *next = node->Next;
                    free(node);
                    node = next;
                }
            }
        }

        /* --- Cross-read arbitration -----------------------------------
           1) update the rolling RSSI history with everything observed
              this sweep, then
           2) build filtered buckets containing only the entries whose
              owning antenna matches the antenna that reported them.
           This stops a tag from showing up under the antenna that's
           only picking it up via field spill-over from the other side. */
        for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
            for (int i = 0; i < cnt[ant]; i++) {
                update_tag_history(ant,
                                   bucket[ant][i].tag,
                                   bucket[ant][i].rssi);
            }
        }

        TagEntry filt_bucket[ANTENNA_COUNT][GC_MAX_TAGS];
        int      filt_cnt[ANTENNA_COUNT] = { 0, 0 };
        for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
            for (int i = 0; i < cnt[ant]; i++) {
                int owner = decide_owner(bucket[ant][i].tag);
                if (owner == -1 || owner == ant) {
                    filt_bucket[ant][filt_cnt[ant]++] = bucket[ant][i];
                }
            }
        }

        print_sweep_line(power, filt_bucket, filt_cnt);

        usleep(GC_SCAN_MS * 1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("[GC] Disconnected.\n");
    return 0;
}
