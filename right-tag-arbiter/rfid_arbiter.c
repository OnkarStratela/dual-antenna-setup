// rfid_arbiter.c
// ---------------------------------------------------------------------------
// "Right tag on the right antenna, with zero overlap, inside 3 seconds."
//
// Standalone tool (does NOT touch the existing rfid_gc_live scanner). It drives
// the same CAEN R3104C reader on the same two logical sources Source_0/Source_1
// (-> physical antennas Ant0/Ant1) and binds each mug (one unique EPC) to
// EXACTLY ONE antenna.
//
//                    WHY THIS USES READ-RATE, NOT RSSI
//   Bench measurement on this rig (probe tool):
//       30 mW : Ant0 20/20 (-61 dBm)   Ant1 20/20 (-57 dBm)
//       10 mW : Ant0 20/20 (-62 dBm)   Ant1 20/20 (-58 dBm)
//        5 mW : Ant0  0/20 (silent)    Ant1 20/20 (-59 dBm)   <-- clean split
//   The two antennas differ by only a fixed ~4 dB (a cable/gain bias, NOT
//   proximity), so RSSI margin cannot tell which antenna a mug is over. But at
//   low power the FAR antenna simply stops reading while the NEAR one keeps
//   reading at full rate. So we decide on PRESENCE / READ-RATE: whichever
//   antenna actually reads the tag (and the other does not) owns it. This also
//   automatically cancels the 4 dB bias, because it keys on whether a read
//   happens at all, not on its level.
//
//                          THE GUARANTEE (0 overlap)
//   Each EPC holds a single `owner` field, so it can never be reported on both
//   antennas. Once committed the binding is LATCHED until the tag is absent
//   (read by neither antenna) for RELEASE_MS, i.e. the mug was lifted.
//
//                          THE 3-SECOND DEADLINE
//   A binding commits once one antenna dominates the reads (rate >= READ_HI
//   while the other <= READ_LO) for CONFIRM_STREAK consecutive sweeps -- a
//   fraction of a second in practice. If BOTH antennas keep reading the tag
//   (power too high to separate) the tag stays pending and the tool tells you
//   to lower the power.
//
// Usage:
//   ./rfid_arbiter              both antennas at the default power
//   ./rfid_arbiter <mW>         both antennas at <mW>  (1..316)
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
#define DEFAULT_POWER_MW    5              // power where only the NEAR antenna reads (tune per rig!)
#define MIN_POWER_MW        1
#define MAX_POWER_MW        316            // R3104C Lepton3 max (25 dBm)
#define SCAN_MS             30             // ms between full sweeps
#define INV_PER_SWEEP       5              // inventories per antenna per sweep (read-rate sample)
#define ANTENNA_COUNT       2
#define MAX_TRACKS          32             // distinct EPCs tracked at once

// --- Read-rate arbitration tuning (see README) ---
// rate[a] is an EWMA (0..1) of "fraction of this sweep's inventories on antenna
// a that saw the tag". The owner is the antenna that clearly reads it while the
// other clearly does not, held for CONFIRM_STREAK sweeps.
#define RATE_ALPHA          0.45   // EWMA weight for the newest sweep
#define READ_HI             0.60   // winner must read at least this fraction
#define READ_LO             0.25   // loser must read at most this fraction
#define CONFIRM_STREAK      4      // consecutive decisive sweeps before committing
#define RELEASE_MS          800    // absent this long (both antennas) => mug removed
#define DECISION_BUDGET_MS  3000   // spec ceiling; we warn if a commit is slower

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
    double   rate[ANTENNA_COUNT];       // EWMA read fraction, 0..1
    double   rssi_disp[ANTENNA_COUNT];  // smoothed RSSI for display

    int      candidate;                 // antenna leading the confidence build, or -1
    int      streak;                    // consecutive decisive sweeps for `candidate`
    bool     ambiguous;                 // both antennas currently reading (power too high)

    int      owner;                     // committed antenna: -1 none, else 0/1
    double   decided_rate;              // winner read-rate at commit (for the log)

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
        "  %s              both antennas at %d mW (default)\n"
        "  %s <mW>         both antennas at <mW>\n"
        "  %s -h | --help  show this message\n"
        "\nValid power range: %d..%d mW\n"
        "Tip: pick the LOWEST power at which the near antenna still reads but\n"
        "     the far antenna goes silent (use ./rfid_probe to find it).\n",
        prog, DEFAULT_POWER_MW, prog, prog, MIN_POWER_MW, MAX_POWER_MW);
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
            tracks[i].used      = true;
            tracks[i].candidate = -1;
            tracks[i].owner     = -1;
            tracks[i].first_ms  = now;
            tracks[i].last_seen_ms = now;
            tracks[i].rssi_disp[0] = NAN;
            tracks[i].rssi_disp[1] = NAN;
            snprintf(tracks[i].epc, sizeof(tracks[i].epc), "%s", epc);
            return i;
        }
    }
    return -1; // table full (would need >MAX_TRACKS simultaneous mugs)
}

static const char *short_epc(const char *epc) {
    size_t n = strlen(epc);
    return (n > 6) ? epc + (n - 6) : epc;
}

// Run INV_PER_SWEEP inventories on one antenna, folding results into the tracks.
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
    printf("%s%s>> ANTENNA %d  OWNS  %s%s%s   (decided in %.2f s, reads %.0f%% vs %.0f%%)%s\n",
           BOLD, col, t->owner, t->epc, RESET BOLD, col, secs,
           t->rate[t->owner] * 100.0, t->rate[other] * 100.0, RESET);
    if ((t->decided_ms - t->first_ms) > DECISION_BUDGET_MS)
        printf("   " YELLOW "!! took longer than the %d ms budget "
               "(weak reads? raise power a little)" RESET "\n", DECISION_BUDGET_MS);
    fflush(stdout);
}

static void print_release(const Track *t) {
    printf(GREY "<< ANTENNA %d  released  %s%s\n", t->owner, t->epc, RESET);
    fflush(stdout);
}

// Fixed-slot live line: slot 0 == antenna 0, slot 1 == antenna 1. Only the
// COMMITTED owner of each antenna is shown (so the two slots can never carry
// the same EPC). Tags still being decided are listed afterwards with their
// per-antenna read-rate so it is obvious why they have not committed.
#define SLOT_WIDTH 30
static void print_status(uint32_t power) {
    printf(CYAN "[TX=%u mW]" RESET " [", (unsigned)power);

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
        int blen = snprintf(body, sizeof body, "...%s r=%.0f%% %.0fdBm",
                            short_epc(t->epc), t->rate[ant] * 100.0, r);
        printf("(%d) %s%s%s", ant, col, body, RESET);
        for (int p = blen; p < SLOT_WIDTH; p++) putchar(' ');
    }
    printf("]");

    bool any_pending = false;
    for (int i = 0; i < MAX_TRACKS; i++) {
        if (!(tracks[i].used && tracks[i].owner == -1)) continue;
        const Track *t = &tracks[i];
        if (!any_pending) { printf("   " YELLOW "pending:" RESET); any_pending = true; }
        printf(" ...%s[r0=%.0f%% r1=%.0f%%%s](%d/%d)",
               short_epc(t->epc), t->rate[0] * 100.0, t->rate[1] * 100.0,
               t->ambiguous ? " BOTH-lower-power" : "", t->streak, CONFIRM_STREAK);
    }
    printf("\n");
    fflush(stdout);
}

// Decide / latch / release one EPC after both antennas were polled this sweep.
static void arbitrate(Track *t, uint64_t now) {
    double f0 = (double)t->hits_now[0] / INV_PER_SWEEP;
    double f1 = (double)t->hits_now[1] / INV_PER_SWEEP;

    // ---- update read-rate EWMA (always; a miss decays the rate toward 0) ----
    t->rate[0] = RATE_ALPHA * f0 + (1.0 - RATE_ALPHA) * t->rate[0];
    t->rate[1] = RATE_ALPHA * f1 + (1.0 - RATE_ALPHA) * t->rate[1];

    // ---- smooth displayed RSSI for whichever antenna(s) saw it this sweep ---
    for (int a = 0; a < ANTENNA_COUNT; a++) {
        if (t->hits_now[a] > 0) {
            double raw = t->rssi_now[a] / 10.0;
            t->rssi_disp[a] = isnan(t->rssi_disp[a]) ? raw
                              : 0.4 * raw + 0.6 * t->rssi_disp[a];
        }
    }

    // ---- release if the mug is gone from BOTH antennas ----
    if (t->hits_now[0] == 0 && t->hits_now[1] == 0 &&
        now - t->last_seen_ms > RELEASE_MS) {
        if (t->owner != -1) print_release(t);
        t->used = false;
        return;
    }

    // Already latched: hold it. No live flips => overlap is impossible.
    if (t->owner != -1) return;

    // ---- who dominates the reads this window? ----
    int    winner = (t->rate[0] >= t->rate[1]) ? 0 : 1;
    int    loser  = winner ^ 1;
    bool   decisive = (t->rate[winner] >= READ_HI) && (t->rate[loser] <= READ_LO);
    t->ambiguous = (t->rate[0] > READ_LO && t->rate[1] > READ_LO); // both reading

    if (!decisive) {            // ambiguous or too weak -> hold the count
        if (t->ambiguous) { t->streak = 0; t->candidate = -1; } // both read: nothing to commit
        return;
    }

    if (winner == t->candidate) t->streak++;
    else { t->candidate = winner; t->streak = 1; }

    if (t->streak >= CONFIRM_STREAK) {
        t->owner        = winner;
        t->decided_ms   = now;
        t->decided_rate = t->rate[winner];
        print_commit(t);
    }
}

int main(int argc, char **argv) {
    uint32_t power = DEFAULT_POWER_MW;

    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            usage(argv[0]); return 0;
        }
        if (!parse_power(argv[1], &power)) { usage(argv[0]); return 1; }
    } else if (argc != 1) {
        usage(argv[0]); return 1;
    }

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
        .com = GC_PORT, .baudrate = GC_BAUDRATE, .dataBits = 8,
        .stopBits = 1, .parity = 0, .flowControl = 0,
    };
    const char *sources[ANTENNA_COUNT] = { "Source_0", "Source_1" };
    char model[64] = {0}, serial[64] = {0};

    signal(SIGINT, handle_sigint);

    printf(CYAN "===== Dual-Antenna RFID Arbiter (read-rate, 0-overlap, <%d s) =====" RESET "\n",
           DECISION_BUDGET_MS / 1000);
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %u mW (both antennas)\n", power);
    printf("Sweep     : %d ms, %d inventories/antenna\n", SCAN_MS, INV_PER_SWEEP);
    printf("Commit when: one antenna reads >= %.0f%% while the other reads <= %.0f%%,\n",
           READ_HI * 100.0, READ_LO * 100.0);
    printf("            for %d consecutive sweeps; released after %d ms absent.\n\n",
           CONFIRM_STREAK, RELEASE_MS);

    printf("[ARB] Connecting...\n");
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
    if (ec != CAENRFID_StatusOK) {
        printf("[ARB] ERROR: Could not connect (code %d)\n", ec);
        printf("  - Try: sudo chmod 666 %s\n", GC_PORT);
        return -1;
    }
    if (CAENRFID_GetReaderInfo(&reader, model, serial) == CAENRFID_StatusOK)
        printf("[ARB] Reader: %s  Serial: %s\n", model, serial);
    char fwrel[MAX_FWREL_LENGTH + 1] = {0};
    if (CAENRFID_GetFirmwareRelease(&reader, fwrel) == CAENRFID_StatusOK)
        printf("[ARB] Firmware: %s\n", fwrel);

    ec = CAENRFID_SetPower(&reader, power);
    if (ec != CAENRFID_StatusOK)
        printf("[ARB] WARNING: SetPower(%u) returned %d (below the reader floor?)\n",
               power, ec);

    // Session S0 + Target A keeps a stationary tag answering every round; small
    // Q suits 1-2 tags per antenna. Soft-fail: ignored if the firmware rejects.
    for (int a = 0; a < ANTENNA_COUNT; a++) {
        CAENRFID_SetSourceConfiguration(&reader, (char *)sources[a],
                                        CONFIG_G2_SESSION, EPC_C1G2_SESSION_S0);
        CAENRFID_SetSourceConfiguration(&reader, (char *)sources[a],
                                        CONFIG_G2_TARGET, EPC_C1G2_TARGET_A);
        CAENRFID_SetSourceConfiguration(&reader, (char *)sources[a],
                                        CONFIG_G2_Q_VALUE, 1);
    }

    printf("[ARB] Ready. Place a mug; its tag binds to exactly one antenna. Ctrl+C to stop.\n\n");
    running = 1;

    while (running) {
        uint64_t now = get_ms_timestamp();

        for (int i = 0; i < MAX_TRACKS; i++) {
            if (!tracks[i].used) continue;
            tracks[i].hits_now[0] = 0;
            tracks[i].hits_now[1] = 0;
        }

        for (int ant = 0; ant < ANTENNA_COUNT && running; ant++)
            inventory_into_tracks(&reader, sources[ant], ant, now);

        for (int i = 0; i < MAX_TRACKS; i++)
            if (tracks[i].used)
                arbitrate(&tracks[i], now);

        print_status(power);
        usleep(SCAN_MS * 1000);
    }

    CAENRFID_Disconnect(&reader);
    printf("[ARB] Disconnected.\n");
    return 0;
}
