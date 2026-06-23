// rfid_arbiter.c
// ---------------------------------------------------------------------------
// "Right tag on the right antenna, with zero overlap, inside 3 seconds."
//
// Standalone tool (does NOT touch the existing rfid_gc_live scanner). It drives
// the same CAEN R3104C reader on the same two logical sources Source_0/Source_1
// (-> physical antennas Ant0/Ant1) and binds each mug (one unique EPC) to
// EXACTLY ONE antenna.
//
//                    WHY PRESENCE + SELF-TUNING POWER
//   Measured on this rig: the two antennas differ by only a FIXED ~4 dB (a
//   cable/gain bias, not proximity), so an RSSI margin cannot tell which
//   antenna a mug is over. BUT there is always a power at which the far antenna
//   goes silent while the near one still reads. That power is NOT a fixed
//   number -- it differs between the two ports (because of the 4 dB bias) and
//   with beer in the tray. So this tool does not hard-code a power: it
//   AUTO-TUNES it.
//
//                    THE CONTROL LOOP (no magic numbers)
//   Start at a power that reliably scans. Then every cycle:
//     * if any tag is read by BOTH antennas  -> power is too high (cross-read):
//                                               step it DOWN.
//     * if a present tag is read weakly / by NEITHER antenna -> too low (beer/
//                                               distance): step it UP.
//     * otherwise it is just right -> hold (SETTLED).
//   It converges on the power where each mug is read by only its near antenna,
//   and tracks changes (a second mug, beer level) live.
//
//                    THE DECISION (0 overlap, latched)
//   A tag is bound to an antenna only when that antenna reads it solidly
//   (>= READ_MIN_HITS of INV_PER_SWEEP inventories) while the OTHER antenna
//   reads it ZERO times, held for CONFIRM_STREAK consecutive sweeps. Each EPC
//   holds a single `owner`, so it can never appear on both antennas. The owner
//   is latched until the tag is gone (read by neither) for RELEASE_MS.
//
// Usage:
//   ./rfid_arbiter              auto-tuning power, default start power
//   ./rfid_arbiter <mW>         FIX the power at <mW> (disables auto-tuning)
//   ./rfid_arbiter -h|--help    show usage
// ---------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>

#include "CAENRFIDLib_Light.h"
#include "host.h"

// ----------------------------- Configuration ------------------------------
#define GC_PORT             "/dev/ttyACM0"
#define GC_BAUDRATE         921600
#define POWER_START_MW      30             // reliable scan to begin with; loop tunes from here
#define MIN_POWER_MW        1
#define MAX_POWER_MW        316            // R3104C Lepton3 max (25 dBm)
#define SCAN_MS             25             // sleep between sweeps
#define INV_PER_SWEEP       4              // inventories per antenna per sweep
#define READ_MIN_HITS       3              // a "solid" read = this many of INV_PER_SWEEP
#define CONFIRM_STREAK      3              // consecutive decisive sweeps before committing
#define RELEASE_MS          900            // absent this long (both antennas) => mug removed
#define PRESENCE_MS         450            // seen within this => still "present" (for power-up)
#define ADJUST_COOLDOWN_MS  110            // min time between power changes (let reads settle)
#define DECISION_BUDGET_MS  3000           // spec ceiling; we warn if a commit is slower
#define ANTENNA_COUNT       2
#define MAX_TRACKS          32

// ------------------------------- ANSI colours ------------------------------
#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define YELLOW "\033[0;33m"
#define CYAN   "\033[0;36m"
#define GREY   "\033[0;90m"
#define BOLD   "\033[1m"
#define RESET  "\033[0m"

volatile int running = 0;

typedef struct {
    bool     used;
    char     epc[2 * MAX_ID_LENGTH + 1];

    int      hits_now[ANTENNA_COUNT];   // inventories this sweep that saw the tag (0..INV_PER_SWEEP)
    int16_t  rssi_now[ANTENNA_COUNT];   // best RSSI this sweep, tenths dBm (valid if hits_now>0)
    double   rssi_disp[ANTENNA_COUNT];  // smoothed RSSI for display

    int      candidate;                 // antenna leading the confidence build, or -1
    int      streak;                    // consecutive decisive sweeps for `candidate`

    int      owner;                     // committed antenna: -1 none, else 0/1
    uint32_t decided_power;             // power at the moment of commit

    uint64_t first_ms;                  // first time this EPC was ever seen
    uint64_t decided_ms;                // time the binding was committed
    uint64_t last_seen_ms;              // last time seen on ANY antenna
} Track;

static Track tracks[MAX_TRACKS];

// ------------------------------- helpers -----------------------------------
static void hex_str(uint8_t *bytes, uint16_t len, char *out) {
    for (int i = 0; i < len; i++)
        sprintf(out + (i * 2), "%02X", bytes[i]);
    out[len * 2] = '\0';
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  %s              auto-tuning power (recommended), starts at %d mW\n"
        "  %s <mW>         FIX power at <mW> (disables auto-tuning)\n"
        "  %s -h | --help  show this message\n"
        "\nValid power range: %d..%d mW\n",
        prog, POWER_START_MW, prog, prog, MIN_POWER_MW, MAX_POWER_MW);
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
    printf("\n" YELLOW "[ARB] Stopping..." RESET "\n");
    running = 0;
}

static int find_track(const char *epc) {
    for (int i = 0; i < MAX_TRACKS; i++)
        if (tracks[i].used && strcmp(tracks[i].epc, epc) == 0)
            return i;
    return -1;
}

static int alloc_track(const char *epc, uint64_t now) {
    for (int i = 0; i < MAX_TRACKS; i++) {
        if (!tracks[i].used) {
            memset(&tracks[i], 0, sizeof(Track));
            tracks[i].used         = true;
            tracks[i].candidate    = -1;
            tracks[i].owner        = -1;
            tracks[i].first_ms     = now;
            tracks[i].last_seen_ms = now;
            tracks[i].rssi_disp[0] = NAN;
            tracks[i].rssi_disp[1] = NAN;
            snprintf(tracks[i].epc, sizeof(tracks[i].epc), "%s", epc);
            return i;
        }
    }
    return -1;
}

static const char *short_epc(const char *epc) {
    size_t n = strlen(epc);
    return (n > 6) ? epc + (n - 6) : epc;
}

static void inventory_into_tracks(CAENRFIDReader *reader, const char *source,
                                  int ant, uint64_t now) {
    for (int k = 0; k < INV_PER_SWEEP; k++) {
        CAENRFIDTagList *tag_list = NULL, *node;
        uint16_t num_tags = 0;
        CAENRFIDErrorCodes ec = CAENRFID_InventoryTag(
            reader, (char *)source, 0, 0, 0, NULL, 0, RSSI | PHASE, &tag_list, &num_tags);

        node = tag_list;
        while (node != NULL) {
            if (ec == CAENRFID_StatusOK && num_tags > 0) {
                char epc[2 * MAX_ID_LENGTH + 1];
                hex_str(node->Tag.ID, node->Tag.Length, epc);
                int t = find_track(epc);
                if (t < 0) t = alloc_track(epc, now);
                if (t >= 0) {
                    tracks[t].hits_now[ant]++;
                    if (tracks[t].hits_now[ant] == 1 || node->Tag.RSSI > tracks[t].rssi_now[ant])
                        tracks[t].rssi_now[ant] = node->Tag.RSSI;
                    tracks[t].last_seen_ms = now;
                }
            }
            CAENRFIDTagList *next = node->Next;
            free(node);
            node = next;
        }
    }
}

static void print_commit(const Track *t) {
    const char *col = (t->owner == 0) ? GREEN : RED;
    double secs = (t->decided_ms - t->first_ms) / 1000.0;
    int other = t->owner ^ 1;
    printf("%s%s>> ANTENNA %d  OWNS  %s%s%s   (decided in %.2f s @ %u mW, reads %d vs %d)%s\n",
           BOLD, col, t->owner, t->epc, RESET BOLD, col, secs,
           (unsigned)t->decided_power, t->hits_now[t->owner], t->hits_now[other], RESET);
    if ((t->decided_ms - t->first_ms) > DECISION_BUDGET_MS)
        printf("   " YELLOW "!! took longer than the %d ms budget" RESET "\n", DECISION_BUDGET_MS);
    fflush(stdout);
}

static void print_release(const Track *t) {
    printf(GREY "<< ANTENNA %d  released  %s%s\n", t->owner, t->epc, RESET);
    fflush(stdout);
}

#define SLOT_WIDTH 26
static void print_status(uint32_t power, const char *state) {
    printf(CYAN "[%u mW %-14s]" RESET " [", (unsigned)power, state);

    for (int ant = 0; ant < ANTENNA_COUNT; ant++) {
        if (ant > 0) printf(" | ");
        int owner_idx = -1;
        for (int i = 0; i < MAX_TRACKS; i++)
            if (tracks[i].used && tracks[i].owner == ant) { owner_idx = i; break; }
        if (owner_idx < 0) {
            printf("(%d) %-*s", ant, SLOT_WIDTH, "----");
            continue;
        }
        const Track *t = &tracks[owner_idx];
        const char *col = (ant == 0) ? GREEN : RED;
        double r = isnan(t->rssi_disp[ant]) ? 0.0 : t->rssi_disp[ant];
        char body[64];
        int blen = snprintf(body, sizeof body, "...%s %.0fdBm", short_epc(t->epc), r);
        printf("(%d) %s%s%s", ant, col, body, RESET);
        for (int p = blen; p < SLOT_WIDTH; p++) putchar(' ');
    }
    printf("]");

    bool any_pending = false;
    for (int i = 0; i < MAX_TRACKS; i++) {
        if (!(tracks[i].used && tracks[i].owner == -1)) continue;
        const Track *t = &tracks[i];
        if (!any_pending) { printf("   " YELLOW "pending:" RESET); any_pending = true; }
        printf(" ...%s[a0=%d a1=%d/%d](%d/%d)",
               short_epc(t->epc), t->hits_now[0], t->hits_now[1], INV_PER_SWEEP,
               t->streak, CONFIRM_STREAK);
    }
    printf("\n");
    fflush(stdout);
}

// Per-EPC decision after both antennas were polled this sweep.
static void arbitrate(Track *t, uint64_t now, uint32_t power) {
    int h0 = t->hits_now[0], h1 = t->hits_now[1];

    for (int a = 0; a < ANTENNA_COUNT; a++) {
        if (t->hits_now[a] > 0) {
            double raw = t->rssi_now[a] / 10.0;
            t->rssi_disp[a] = isnan(t->rssi_disp[a]) ? raw : 0.4 * raw + 0.6 * t->rssi_disp[a];
        }
    }

    if (h0 == 0 && h1 == 0) {
        if (now - t->last_seen_ms > RELEASE_MS) {
            if (t->owner != -1) print_release(t);
            t->used = false;
        }
        return;
    }

    if (t->owner != -1) return;   // latched

    // Decisive only if ONE antenna reads solidly and the OTHER reads zero.
    int w = -1;
    if (h0 >= READ_MIN_HITS && h1 == 0)      w = 0;
    else if (h1 >= READ_MIN_HITS && h0 == 0) w = 1;

    if (w < 0) {                  // both reading, or too weak -> not decisive
        t->streak = 0;
        t->candidate = -1;
        return;
    }

    if (w == t->candidate) t->streak++;
    else { t->candidate = w; t->streak = 1; }

    if (t->streak >= CONFIRM_STREAK) {
        t->owner         = w;
        t->decided_ms    = now;
        t->decided_power = power;
        print_commit(t);
    }
}

int main(int argc, char **argv) {
    uint32_t power = POWER_START_MW;
    bool auto_tune = true;

    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            usage(argv[0]); return 0;
        }
        if (!parse_power(argv[1], &power)) { usage(argv[0]); return 1; }
        auto_tune = false;        // explicit power => fixed
    } else if (argc != 1) {
        usage(argv[0]); return 1;
    }

    CAENRFIDReader reader = {
        .connect = _connect, .disconnect = _disconnect, .tx = _tx, .rx = _rx,
        .clear_rx_data = _clear_rx_data, .enable_irqs = _enable_irqs,
        .disable_irqs = _disable_irqs
    };
    RS232_params port_params = {
        .com = GC_PORT, .baudrate = GC_BAUDRATE, .dataBits = 8,
        .stopBits = 1, .parity = 0, .flowControl = 0,
    };
    const char *sources[ANTENNA_COUNT] = { "Source_0", "Source_1" };
    char model[64] = {0}, serial[64] = {0};

    signal(SIGINT, handle_sigint);

    printf(CYAN "===== Dual-Antenna RFID Arbiter (self-tuning, 0-overlap, <%d s) =====" RESET "\n",
           DECISION_BUDGET_MS / 1000);
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %s (start %u mW)\n",
           auto_tune ? "AUTO-TUNING" : "FIXED", power);
    printf("Commit when: one antenna reads >= %d/%d while the other reads 0,\n",
           READ_MIN_HITS, INV_PER_SWEEP);
    printf("            for %d consecutive sweeps; released after %d ms absent.\n\n",
           CONFIRM_STREAK, RELEASE_MS);

    printf("[ARB] Connecting...\n");
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
    if (ec != CAENRFID_StatusOK) {
        printf("[ARB] ERROR: Could not connect (code %d). Try: sudo chmod 666 %s\n",
               ec, GC_PORT);
        return -1;
    }
    if (CAENRFID_GetReaderInfo(&reader, model, serial) == CAENRFID_StatusOK)
        printf("[ARB] Reader: %s  Serial: %s\n", model, serial);
    char fwrel[MAX_FWREL_LENGTH + 1] = {0};
    if (CAENRFID_GetFirmwareRelease(&reader, fwrel) == CAENRFID_StatusOK)
        printf("[ARB] Firmware: %s\n", fwrel);

    CAENRFID_SetPower(&reader, power);

    for (int a = 0; a < ANTENNA_COUNT; a++) {
        CAENRFID_SetSourceConfiguration(&reader, (char *)sources[a],
                                        CONFIG_G2_SESSION, EPC_C1G2_SESSION_S0);
        CAENRFID_SetSourceConfiguration(&reader, (char *)sources[a],
                                        CONFIG_G2_TARGET, EPC_C1G2_TARGET_A);
        CAENRFID_SetSourceConfiguration(&reader, (char *)sources[a],
                                        CONFIG_G2_Q_VALUE, 1);
    }

    printf("[ARB] Ready. Place a mug; the power self-tunes and binds it to one antenna. Ctrl+C to stop.\n\n");
    running = 1;

    uint64_t last_adjust_ms = 0;
    const char *state = "starting";

    while (running) {
        uint64_t now = get_ms_timestamp();

        for (int i = 0; i < MAX_TRACKS; i++) {
            if (!tracks[i].used) continue;
            tracks[i].hits_now[0] = 0;
            tracks[i].hits_now[1] = 0;
        }

        for (int ant = 0; ant < ANTENNA_COUNT && running; ant++)
            inventory_into_tracks(&reader, sources[ant], ant, now);

        // ---- assess the sweep for the power-control loop ----
        bool overlap = false;     // a tag heard by BOTH antennas (cross-read)
        bool weak    = false;     // a present tag not solidly read by either
        for (int i = 0; i < MAX_TRACKS; i++) {
            if (!tracks[i].used) continue;
            int h0 = tracks[i].hits_now[0], h1 = tracks[i].hits_now[1];
            if (h0 > 0 && h1 > 0) overlap = true;
            int best = (h0 > h1) ? h0 : h1;
            bool present_recently = (now - tracks[i].last_seen_ms) <= PRESENCE_MS;
            if (present_recently && best < READ_MIN_HITS) weak = true;
        }

        // ---- self-tuning power controller ----
        if (auto_tune && (now - last_adjust_ms) >= ADJUST_COOLDOWN_MS) {
            uint32_t np = power;
            if (overlap) {                       // too hot: far antenna hears it too
                np = (uint32_t)(power * 0.80);
                if (np >= power) np = power - 1;
            } else if (weak) {                   // too cold: near antenna struggles
                np = (uint32_t)(power * 1.25) + 1;
            }
            if (np < MIN_POWER_MW) np = MIN_POWER_MW;
            if (np > MAX_POWER_MW) np = MAX_POWER_MW;
            if (np != power) {
                power = np;
                CAENRFID_SetPower(&reader, power);
                last_adjust_ms = now;
                for (int i = 0; i < MAX_TRACKS; i++)   // conditions changed; restart pending counts
                    if (tracks[i].used && tracks[i].owner == -1) {
                        tracks[i].streak = 0; tracks[i].candidate = -1;
                    }
            }
        }

        // ---- per-tag decision ----
        for (int i = 0; i < MAX_TRACKS; i++)
            if (tracks[i].used)
                arbitrate(&tracks[i], now, power);

        // ---- human-readable state for the status line ----
        if (overlap && power <= MIN_POWER_MW)       state = "OVERLAP@floor!";
        else if (overlap)                           state = "tuning down";
        else if (weak && power >= MAX_POWER_MW)      state = "TOO-WEAK@max!";
        else if (weak)                              state = "tuning up";
        else                                        state = "settled";

        print_status(power, state);
        usleep(SCAN_MS * 1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("[ARB] Disconnected.\n");
    return 0;
}
