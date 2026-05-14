// rfid_gc_live.c
// Live-state RFID scanner using two antennas.
//
// Every sweep (continuous), prints a single line while scanning runs:
//
// Empty (no tags in range — shows activity without extra noise):
//   []
//
// Tags visible (shows TX so power stays obvious):
//   [TX Source_0=30 Source_1=80 mW] [(0) @30mW EPC, (1) @80mW EPC]
//
// Antenna number is printed in YELLOW, tag code in GREEN.
//
// Usage:
//   ./rfid_gc_live                  -> both antennas at default power
//   ./rfid_gc_live <mW>             -> both antennas at <mW>
//   ./rfid_gc_live <mW0> <mW1>      -> Source_0 at mW0, Source_1 at mW1
//   ./rfid_gc_live -h | --help      -> show usage

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
#define GC_PORT          "/dev/ttyACM0"
#define GC_BAUDRATE      921600
#define DEFAULT_POWER_MW 30               // sensible default for ~7 cm read zone
#define MIN_POWER_MW     1                // reader rejects below its hardware floor
#define MAX_POWER_MW     316              // R3100C Lepton3 max (25 dBm)
#define GC_SCAN_MS       100              // ms between scan cycles (= line rate when printing every cycle)
#define GC_MAX_TAGS      64               // max tags merged across both antennas per sweep
#define ANTENNA_COUNT    2
#define MAX_ID_LENGTH    64

// ANSI colours
#define GREEN  "\033[0;32m"
#define YELLOW "\033[0;33m"
#define CYAN   "\033[0;36m"
#define RESET  "\033[0m"

volatile int running = 0;

typedef struct {
    char tag[2 * MAX_ID_LENGTH + 1];
    int  antenna;
} TagEntry;

static void hex_str(uint8_t *bytes, uint16_t len, char *out) {
    for (int i = 0; i < len; i++)
        sprintf(out + (i * 2), "%02X", bytes[i]);
    out[len * 2] = '\0';
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s                  both antennas at %d mW (default)\n"
        "  %s <mW>             both antennas at <mW>\n"
        "  %s <mW0> <mW1>      Source_0 at mW0, Source_1 at mW1\n"
        "  %s -h | --help      show this message\n"
        "\nValid power range: %d..%d mW\n",
        prog, DEFAULT_POWER_MW, prog, prog, prog,
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

static void fmt_tx_power_prefix(char *buf, size_t cap,
                                uint32_t pwr[ANTENNA_COUNT],
                                bool same_power) {
    /* Shown before every snapshot so the CLI power settings are always visible. */
    if (same_power)
        snprintf(buf, cap,
                 CYAN "[TX=%u mW]" RESET,
                 (unsigned)pwr[0]);
    else
        snprintf(buf, cap,
                 CYAN "[TX Source_0=%u Source_1=%u mW]" RESET,
                 (unsigned)pwr[0], (unsigned)pwr[1]);
}

// Sweep output: bare [] when idle; Tx prefix only when listing tags (merged Src0→Src1 order).
static void print_sweep_line(uint32_t pwr[ANTENNA_COUNT], bool same_power,
                             TagEntry bucket[ANTENNA_COUNT][GC_MAX_TAGS],
                             const int cnt[ANTENNA_COUNT])
{
    int total = cnt[0] + cnt[1];
    if (total == 0) {
        printf("[]\n");
        fflush(stdout);
        return;
    }

    char pfx[96];
    fmt_tx_power_prefix(pfx, sizeof pfx, pwr, same_power);
    printf("%s ", pfx);

    printf("[");
    bool first_elem = true;
    for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
        for (int i = 0; i < cnt[ant]; i++) {
            if (!first_elem)
                printf(", ");
            first_elem = false;

            TagEntry *e = &bucket[ant][i];
            printf(YELLOW "(%d)" RESET, e->antenna);
            if (!same_power)
                printf(" @" CYAN "%u" RESET "mW", (unsigned)pwr[ant]);
            printf(" " GREEN "%s" RESET, e->tag);
        }
    }
    printf("]\n");
    fflush(stdout);
}

int main(int argc, char **argv) {

    uint32_t power[ANTENNA_COUNT] = { DEFAULT_POWER_MW, DEFAULT_POWER_MW };

    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        uint32_t p;
        if (!parse_power(argv[1], &p)) {
            usage(argv[0]);
            return 1;
        }
        power[0] = power[1] = p;
    } else if (argc == 3) {
        uint32_t p0, p1;
        if (!parse_power(argv[1], &p0) || !parse_power(argv[2], &p1)) {
            usage(argv[0]);
            return 1;
        }
        power[0] = p0;
        power[1] = p1;
    } else if (argc != 1) {
        usage(argv[0]);
        return 1;
    }

    bool same_power = (power[0] == power[1]);

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
    if (same_power) {
        printf("Power     : %u mW (both antennas)\n", power[0]);
    } else {
        printf("Power     : %s=%u mW, %s=%u mW\n",
               sources[0], power[0], sources[1], power[1]);
    }
    printf("Cycle     : %d ms\n", GC_SCAN_MS);
    printf("Antennas  : %s, %s\n\n", sources[0], sources[1]);

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

    if (same_power) {
        ec = CAENRFID_SetPower(&reader, power[0]);
        if (ec != CAENRFID_StatusOK) {
            printf("[GC] WARNING: SetPower(%u) returned %d -- "
                   "value may be below the reader's hardware floor.\n",
                   power[0], ec);
        }
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

            if (!same_power) {
                CAENRFID_SetPower(&reader, power[ant]);
            }

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

        print_sweep_line(power, same_power, bucket, cnt);

        usleep(GC_SCAN_MS * 1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("[GC] Disconnected.\n");
    return 0;
}
