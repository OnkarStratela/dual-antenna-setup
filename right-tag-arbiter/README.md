# Right-Tag / Right-Antenna Arbiter (zero overlap, < 3 s)

A **self-contained** tool that solves one problem on the existing dual-antenna
rig: when a mug is placed on the drip tray, bind **its tag to exactly one
antenna**, with **no chance of the same tag appearing on both antennas**, and
reach that decision **within 3 seconds** of placement.

It does **not** modify the existing `rfid_gc_live` setup. This folder ships its
own copy of the CAEN `SRC/` library and builds its own binary, so you can run
it side-by-side or drop it on the Pi on its own.

---

## The setup it is written for

| Item | Value |
|------|-------|
| Compute | Raspberry Pi CM4 + carrier board |
| Reader | CAEN R3100C (Lepton3), 25 dBm / 316 mW max, USB `/dev/ttyACM0` |
| Antennas | 2 × UHF (CAEN WANT20), under the drip tray |
| Antenna ↔ tag | ~10 cm (tag on the bottom of the mug) |
| Antenna ↔ antenna | ~150 cm, centre-to-centre |
| Drip tray | may be **half full of beer**; mugs themselves are empty |
| Logical sources | `Source_0` → antenna 0, `Source_1` → antenna 1 |

---

## Why this is reliably solvable — and why we use READ-RATE, not RSSI

Bench measurement on this exact rig (with the `rfid_probe` tool), one mug over
antenna 1:

| Power | Source_0 / Ant0 | Source_1 / Ant1 |
|------:|-----------------|-----------------|
| 30 mW | −61 dBm, 20/20  | −57 dBm, 20/20  |
| 10 mW | −62 dBm, 20/20  | −58 dBm, 20/20  |
| **5 mW** | **0/20 (silent)** | −59 dBm, **20/20** |

Two things this proves:

1. The two antennas differ by only a **fixed ~4 dB** that does **not** change
   with mug position — it is a cable/gain bias, not proximity. So an RSSI
   *margin* cannot tell which antenna a mug is over. (That approach was tried
   and always picked the same antenna.)
2. At **low power the far antenna stops reading entirely** while the near one
   still reads at full rate. That is a clean, binary split.

**So the arbiter decides on PRESENCE / READ-RATE: whichever antenna actually
reads the tag (while the other does not) owns it.** Keying on *whether a read
happens* rather than *how strong it is* also automatically cancels the 4 dB
bias. The whole method depends on running at a power low enough that only the
near antenna reads — see "Finding the right power" below.

---

## How it guarantees "0 overlap"

Each mug has a unique EPC. The arbiter keeps one record per EPC with a single
`owner` field (`antenna 0`, `antenna 1`, or "not yet decided"). Because an EPC
can only ever hold **one** owner value, it is structurally impossible for the
same tag to be reported on both antennas.

State machine per EPC:

1. **Deciding** — every sweep fires `INV_PER_SWEEP` inventories on each antenna
   and counts how many saw the tag.
2. **Commit** — the binding is locked in once **one antenna reads the tag
   solidly** (≥ `READ_MIN_HITS` of `INV_PER_SWEEP`) **while the other reads it
   zero times**, held for `CONFIRM_STREAK` consecutive sweeps. If **both**
   antennas hear the tag the sweep is not decisive (and the power controller is
   busy lowering power — see below).
3. **Latched** — once owned, the binding is held. Reads of that EPC on the
   *other* antenna are ignored, so no live flips, so no overlap.
4. **Release** — when the tag is seen by **neither** antenna for `RELEASE_MS`
   (the mug was lifted), the binding is cleared and the next placement is
   decided fresh.

## Automatic power tuning (no magic numbers)

The clean-separation power is **not a fixed value** — it differs between the two
ports (the ~4 dB bias) and with beer in the tray. So the tool tunes it itself,
every cycle:

- a tag heard by **both** antennas ⇒ power too high (cross-read) ⇒ **step down**;
- a present tag **not read solidly** by either ⇒ too low (beer/distance) ⇒ **step up**;
- otherwise ⇒ **hold** (settled).

The status line shows the live power and state: `tuning down`, `tuning up`,
`settled`, or a warning if it cannot win: `OVERLAP@floor!` (both antennas still
hear the tag even at minimum power — the antennas physically overlap at the mug
location) or `TOO-WEAK@max!` (even full power can't read it — too far / too much
beer). Those warnings tell you it's a hardware-geometry issue, not software.

You can still **fix** the power (disabling auto-tune) by passing it on the
command line, e.g. `./run.sh 8`.

### The 3-second deadline

Once the power has settled (typically ~1 s of tuning at startup, instant
thereafter), a binding commits in a few sweeps — a fraction of a second. Every
commit prints its actual time-to-decision and the power it settled at, and flags
any commit slower than the 3000 ms budget.

---

## Build & run (on the Pi / Linux)

```bash
cd right-tag-arbiter
chmod +x run.sh
./run.sh            # AUTO-TUNING power (recommended)
./run.sh 8          # FIX power at 8 mW (disables auto-tuning)
```

`run.sh` fixes `/dev/ttyACM0` permissions if needed, compiles, then runs.
To build only:

```bash
./compile.sh
./rfid_arbiter           # auto-tuning;  or  ./rfid_arbiter 8  to fix power
./rfid_arbiter --help
```

---

## What you see

A live line showing the current power + tuning state, the **committed owner** of
each antenna slot, and any tags still being decided (with their per-antenna hit
counts):

```
[8 mW settled       ] [(0) ...A1234 -58dBm        | (1) ----                      ]
[12 mW tuning down   ] [(0) ----                    | (1) ----                      ]   pending: ...471224[a0=4 a1=4/4](0/3)
```

A prominent event line each time a binding is committed or released:

```
>> ANTENNA 1  OWNS  E2801160600002054E1A471224   (decided in 0.42 s @ 8 mW, reads 4 vs 0)
<< ANTENNA 1  released  E2801160600002054E1A471224
```

- Antenna 0 owners render green, antenna 1 owners red, the `(N)` marker yellow.

---

## Tuning (all at the top of `rfid_arbiter.c`)

You normally don't need to touch anything — power is automatic. If you want to:

| Macro | Default | Effect |
|-------|---------|--------|
| `POWER_START_MW` | `30` | Power the auto-tuner starts from before converging. |
| `READ_MIN_HITS` | `3` | Of `INV_PER_SWEEP`, how many reads count as a "solid" read for the near antenna. |
| `INV_PER_SWEEP` | `4` | Inventories per antenna per sweep. |
| `CONFIRM_STREAK` | `3` | Consecutive decisive sweeps before committing. |
| `RELEASE_MS` | `900` | How long a tag must be gone (both antennas) before the binding clears. |
| `ADJUST_COOLDOWN_MS` | `110` | Minimum time between power steps (lets reads settle). |
| `SCAN_MS` | `25` | Sleep between full sweeps. |

The `rfid_probe` tool is still useful to inspect the raw per-antenna behaviour
(source→antenna mapping and read counts/RSSI at a chosen power).

---

## Files

| File | Purpose |
|------|---------|
| `rfid_arbiter.c` | The arbiter (read-rate presence, commit-and-latch) |
| `rfid_probe.c` | Hardware probe: source→antenna mapping + per-source read count/RSSI; use it to find the right power |
| `compile.sh` | Builds the arbiter against the bundled `SRC/` |
| `compile_probe.sh` | Builds the probe |
| `run.sh` | Fixes USB perms, compiles the arbiter, runs (accepts optional power arg) |
| `SRC/` | Bundled copy of the CAEN "Light" library (independent of the parent folder) |
