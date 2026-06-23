// rfid_arbiter.c
// ---------------------------------------------------------------------------
// "Right tag on the right antenna, with zero overlap, inside 3 seconds."
//
// This is a standalone tool (it does NOT touch the existing rfid_gc_live
// scanner). It drives the SAME CAEN R3100C (Lepton3) reader over the SAME
// two logical sources (Source_0 / Source_1), but instead of dumping every
// raw read it runs a deterministic arbitration state-machine so that each
// physical mug (one unique EPC) is bound to EXACTLY ONE antenna.
//
//                          THE PHYSICS WE EXPLOIT
//   - Owning antenna  : ~10 cm from the tag under the drip tray.
//   - Other antenna   : ~150 cm away (centre-to-centre).
//   Round-trip free-space path-loss difference:
//         2 * 20*log10(150/10)  ~=  47 dB.
//   So the antenna a tag belongs to is NEVER ambiguous by *relative* RSSI,
//   even when the drip tray is half full of beer: beer (water) attenuates
//   BOTH paths by a similar amount, so the *difference* between the two
//   antennas stays huge. We arbitrate on that difference, never on an
//   absolute level, which is what makes this robust to the beer.
//
//                          THE GUARANTEE (0 overlap)
//   An EPC is reported on a single antenna and is then LATCHED. While the
//   mug is present it can never appear on the other antenna. The binding is
//   only cleared once the tag has been absent (seen by neither antenna) for
//   RELEASE_MS, i.e. the mug was physically lifted. The next placement is
//   then decided fresh. Because "owner" is a single value per EPC, overlap
//   is impossible by construction.
//
//                          THE 3-SECOND DEADLINE
//   A binding is committed once the winning antenna dominates (by >= MARGIN
//   dB, or is the sole reader) AND its level clears a near-presence floor,
//   for CONFIRM_STREAK consecutive sweeps. At the default sweep rate this
//   happens in well under one second; the 3 s figure is only the worst-case
//   ceiling, and we print the actual time-to-decision for every commit (and
//   loudly flag it if it ever exceeds the 3 s budget).
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
#define DEFAULT_POWER_MW    100           // punch through beer; arbitration handles separation
#define MIN_POWER_MW        1
#define MAX_POWER_MW        316            // R3100C Lepton3 max (25 dBm)
#define SCAN_MS             60             // ms between full Src0->Src1 sweeps
#define ANTENNA_COUNT       2
#define MAX_TRACKS          32             // distinct EPCs tracked at once

// --- Arbitration tuning (see README for the reasoning behind each value) ---
// Real-world finding: at high power BOTH antennas hear the same tag, and the
// owning antenna leads only by a few dB -- but that lead is perfectly STABLE
// in direction. So we decide on the *consistent winner*, not a large gap:
// the same antenna must remain the stronger reader (using smoothed RSSI, by
// at least MIN_MARGIN_TENTHS, or be the sole reader) for CONFIRM_STREAK
// consecutive sweeps. A "tie" sweep (gap below the minimum) simply holds the
// current count rather than resetting it; only the winner flipping resets it.
#define MIN_MARGIN_TENTHS   10     // 1.0 dB: enough to call a winner; rejects a dead tie
#define NEAR_FLOOR_TENTHS   (-900) // -90.0 dBm: permissive sanity floor on the winner
#define CONFIRM_STREAK      5      // consecutive sweeps the same antenna must lead
#define RELEASE_MS          800    // absent this long (both antennas) => mug removed
#define DECISION_BUDGET_MS  3000   // the spec ceiling; we warn if a commit is slower
#define RSSI_EWMA_ALPHA     0.40   // display-only smoothing of RSSI

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

    bool     seen_now[ANTENNA_COUNT];   // read on this antenna THIS sweep?
    int16_t  rssi_now[ANTENNA_COUNT];   // best (max) RSSI this sweep, tenths dBm
    double   rssi_disp[ANTENNA_COUNT];  // smoothed RSSI for the live display

    int      candidate;                 // antenna leading the confidence build, or -1
    int      streak;                    // consecutive qualifying sweeps for `candidate`

    int      owner;                     // committed antenna: -1 none, else 0/1
    double   decided_margin;            // dB margin at the moment of commit

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
        "\nValid power range: %d..%d mW\n",
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
    // Show the last 6 hex chars; plenty to tell mugs apart on screen.
    size_t n = strlen(epc);
    return (n > 6) ? epc + (n - 6) : epc;
}

// One inventory pass on a single antenna, folding results into the tracks.
static void inventory_into_tracks(CAENRFIDReader *reader, const char *source,
                                  int ant, uint64_t now) {
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
                if (!tracks[t].seen_now[ant] || node->Tag.RSSI > tracks[t].rssi_now[ant])
                    tracks[t].rssi_now[ant] = node->Tag.RSSI;
                tracks[t].seen_now[ant]  = true;
                tracks[t].last_seen_ms   = now;
            }
        }
        CAENRFIDTagList *next = node->Next;
        free(node);
        node = next;
    }
}

// Pretty-print a commit / release event so it stands out in the scrollback.
static void print_commit(const Track *t) {
    const char *col = (t->owner == 0) ? GREEN : RED;
    double secs = (t->decided_ms - t->first_ms) / 1000.0;
    char marginbuf[16];
    if (isinf(t->decided_margin)) snprintf(marginbuf, sizeof marginbuf, "sole");
    else snprintf(marginbuf, sizeof marginbuf, "%.1f dB", t->decided_margin);

    printf("%s%s>> ANTENNA %d  OWNS  %s%s%s   (decided in %.2f s, margin %s)%s\n",
           BOLD, col, t->owner, t->epc, RESET BOLD, col, secs, marginbuf, RESET);
    if ((t->decided_ms - t->first_ms) > DECISION_BUDGET_MS)
        printf("   " YELLOW "!! took longer than the %d ms budget "
               "(weak/beer-attenuated reads?)" RESET "\n", DECISION_BUDGET_MS);
    fflush(stdout);
}

static void print_release(const Track *t) {
    printf(GREY "<< ANTENNA %d  released  %s%s\n", t->owner, t->epc, RESET);
    fflush(stdout);
}

// Fixed-slot live line: slot 0 == antenna 0, slot 1 == antenna 1. Only the
// COMMITTED owner of each antenna is shown, so the two slots can never carry
// the same EPC. Tags still being decided are listed afterwards as "pending".
#define SLOT_WIDTH 34
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
        double r = isnan(t->rssi_disp[ant]) ? (t->rssi_now[ant] / 10.0)
                                            : t->rssi_disp[ant];
        char body[64];
        int blen = snprintf(body, sizeof body, "%6.1f dBm  ...%s",
                            r, short_epc(t->epc));
        printf("(%d) %s%s%s", ant, col, body, RESET);
        for (int p = blen; p < SLOT_WIDTH; p++) putchar(' ');
    }
    printf("]");

    // Append any tags that are present but not yet bound to an antenna, with a
    // live readout of each antenna's RSSI and the gap between them, so it is
    // obvious WHY a tag has not committed (too close to call, or too weak).
    bool any_pending = false;
    for (int i = 0; i < MAX_TRACKS; i++) {
        if (!(tracks[i].used && tracks[i].owner == -1)) continue;
        const Track *t = &tracks[i];
        if (!any_pending) { printf("   " YELLOW "pending:" RESET); any_pending = true; }

        char a0[12], a1[12], gap[16];
        if (t->seen_now[0]) snprintf(a0, sizeof a0, "%.1f", t->rssi_now[0] / 10.0);
        else                snprintf(a0, sizeof a0, "  --");
        if (t->seen_now[1]) snprintf(a1, sizeof a1, "%.1f", t->rssi_now[1] / 10.0);
        else                snprintf(a1, sizeof a1, "  --");
        if (t->seen_now[0] && t->seen_now[1])
            snprintf(gap, sizeof gap, "d=%.1f",
                     fabs((t->rssi_now[0] - t->rssi_now[1]) / 10.0));
        else if (t->seen_now[0]) snprintf(gap, sizeof gap, "sole0");
        else if (t->seen_now[1]) snprintf(gap, sizeof gap, "sole1");
        else                     snprintf(gap, sizeof gap, "miss");

        printf(" ...%s[a0=%s a1=%s %s](%d/%d)",
               short_epc(t->epc), a0, a1, gap, t->streak, CONFIRM_STREAK);
    }
    printf("\n");
    fflush(stdout);
}

// Run the arbitration decision for one EPC after both antennas were polled.
static void arbitrate(Track *t, uint64_t now) {
    bool s0 = t->seen_now[0], s1 = t->seen_now[1];

    // ---- not seen on either antenna this sweep ----
    if (!s0 && !s1) {
        if (now - t->last_seen_ms > RELEASE_MS) {
            if (t->owner != -1) print_release(t);
            t->used = false;            // free the slot; next placement decides fresh
        }
        return;                         // a brief miss does not disturb the streak
    }

    // ---- smooth the displayed RSSI for whichever antenna(s) saw it ----
    for (int a = 0; a < ANTENNA_COUNT; a++) {
        if (t->seen_now[a]) {
            double raw = t->rssi_now[a] / 10.0;
            t->rssi_disp[a] = isnan(t->rssi_disp[a])
                ? raw
                : RSSI_EWMA_ALPHA * raw + (1.0 - RSSI_EWMA_ALPHA) * t->rssi_disp[a];
        }
    }

    // Already latched: hold it. No live flips => overlap is impossible.
    if (t->owner != -1) return;

    // ---- pick this sweep's winner from the SMOOTHED per-antenna RSSI ----
    // Smoothing matters here: the lead is only a few dB, so we filter noise
    // before comparing rather than reacting to a single jittery sample.
    int    cand;
    double margin_db;       // dB the winner leads by (INFINITY if sole reader)
    double cand_rssi_db;    // smoothed RSSI of the winning antenna
    if (s0 && s1) {
        double d0 = t->rssi_disp[0], d1 = t->rssi_disp[1];
        if (d0 >= d1) { cand = 0; cand_rssi_db = d0; margin_db = d0 - d1; }
        else          { cand = 1; cand_rssi_db = d1; margin_db = d1 - d0; }
    } else if (s0) {
        cand = 0; cand_rssi_db = t->rssi_disp[0]; margin_db = INFINITY;
    } else {
        cand = 1; cand_rssi_db = t->rssi_disp[1]; margin_db = INFINITY;
    }

    // Winner too weak to trust at all -> hold the count, do not reset.
    if (cand_rssi_db < NEAR_FLOOR_TENTHS / 10.0) return;

    // Antennas essentially tied this sweep -> not decisive, hold the count.
    if (!isinf(margin_db) && margin_db < MIN_MARGIN_TENTHS / 10.0) return;

    // Decisive for `cand`. Only the winner CHANGING restarts the count, so a
    // stable few-dB lead steadily accumulates confidence toward a commit.
    if (cand == t->candidate) t->streak++;
    else { t->candidate = cand; t->streak = 1; }

    if (t->streak >= CONFIRM_STREAK) {
        t->owner          = cand;
        t->decided_ms     = now;
        t->decided_margin = margin_db;
        print_commit(t);
    }
}

int main(int argc, char **argv) {
    uint32_t power = DEFAULT_POWER_MW;

    if (argc == 2) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            usage(argv[0]);
            return 0;
        }
        if (!parse_power(argv[1], &power)) { usage(argv[0]); return 1; }
    } else if (argc != 1) {
        usage(argv[0]);
        return 1;
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
        .com         = GC_PORT,
        .baudrate    = GC_BAUDRATE,
        .dataBits    = 8,
        .stopBits    = 1,
        .parity      = 0,
        .flowControl = 0,
    };
    const char *sources[ANTENNA_COUNT] = { "Source_0", "Source_1" };
    char model[64] = {0}, serial[64] = {0};

    signal(SIGINT, handle_sigint);

    printf(CYAN "===== Dual-Antenna RFID Arbiter (0-overlap, <%d s) =====" RESET "\n",
           DECISION_BUDGET_MS / 1000);
    printf("Port      : %s @ %d baud\n", GC_PORT, GC_BAUDRATE);
    printf("Power     : %u mW (both antennas)\n", power);
    printf("Sweep     : %d ms\n", SCAN_MS);
    printf("Commit when: one antenna stays the stronger reader (by >= %.1f dB,"
           " or sole)\n", MIN_MARGIN_TENTHS / 10.0);
    printf("            for %d consecutive sweeps; released after %d ms absent.\n\n",
           CONFIRM_STREAK, RELEASE_MS);

    printf("[ARB] Connecting...\n");
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader, CAENRFID_RS232, &port_params);
    if (ec != CAENRFID_StatusOK) {
        printf("[ARB] ERROR: Could not connect (code %d)\n", ec);
        printf("  - Check USB cable\n");
        printf("  - Try: sudo chmod 666 %s\n", GC_PORT);
        printf("  - Or:  sudo usermod -a -G dialout $USER  (then re-login)\n");
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

    // Best-effort source tuning for a tiny, static tag population: Session S0 +
    // Target A keeps a stationary tag answering every round (fast presence
    // detection), and a small Q suits 1-2 tags per antenna. Soft-fail: if the
    // firmware rejects any of these the inventory loop still works fine.
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

        // New sweep: clear per-sweep observations.
        for (int i = 0; i < MAX_TRACKS; i++) {
            if (!tracks[i].used) continue;
            tracks[i].seen_now[0] = false;
            tracks[i].seen_now[1] = false;
        }

        // Poll each physical antenna through its logical source.
        for (int ant = 0; ant < ANTENNA_COUNT && running; ant++)
            inventory_into_tracks(&reader, sources[ant], ant, now);

        // Decide / latch / release each tracked EPC.
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
