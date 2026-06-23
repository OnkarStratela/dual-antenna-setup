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

1. **Deciding** — every sweep it polls both antennas and picks the candidate
   owner = the stronger antenna (or the only antenna that read it).
2. **Commit** — the candidate is locked in once **one antenna dominates the
   reads** — its read-rate ≥ `READ_HI` while the other's ≤ `READ_LO` — for
   `CONFIRM_STREAK` consecutive sweeps. Each sweep fires `INV_PER_SWEEP`
   inventories per antenna; the fraction that see the tag feeds a smoothed
   read-rate per antenna. If **both** antennas keep reading the tag (power too
   high to separate) the tag stays **pending** and the live line shows
   `BOTH-lower-power` so you know to drop the power.
3. **Latched** — once owned, the binding is held. Reads of that EPC on the
   *other* antenna are ignored, so no live flips, so no overlap.
4. **Release** — when the tag is seen by **neither** antenna for `RELEASE_MS`
   (the mug was lifted), the binding is cleared and the next placement is
   decided fresh.

### The 3-second deadline

At the default 60 ms sweep, three confirming sweeps commit a binding in well
under one second. The 3 s in the spec is only the worst-case ceiling. Every
commit prints its **actual** time-to-decision, and if a commit ever exceeds the
3000 ms budget (e.g. very weak, beer-attenuated reads) it is flagged loudly so
you can raise the power or lower `NEAR_FLOOR`.

---

## Build & run (on the Pi / Linux)

```bash
cd right-tag-arbiter
chmod +x run.sh
./run.sh            # default 100 mW on both antennas
./run.sh 120        # override power (mW, 1..316)
```

`run.sh` fixes `/dev/ttyACM0` permissions if needed, compiles, then runs.
To build only:

```bash
./compile.sh
./rfid_arbiter           # or  ./rfid_arbiter 120
./rfid_arbiter --help
```

---

## What you see

A fixed-slot live line (slot 0 = antenna 0, slot 1 = antenna 1) showing only the
**committed owner** of each antenna, plus any tags still being decided:

```
[TX=100 mW] [(0)  -58.3 dBm  ...A1234   | (1) ----                          ]   pending: ...B5678(1/3)
```

A prominent event line each time a binding is committed or released:

```
>> ANTENNA 0  OWNS  E2801160600002054E1AA1234   (decided in 0.30 s, margin 31.4 dB)
<< ANTENNA 0  released  E2801160600002054E1AA1234
```

- `margin` is how much the owning antenna beat the other (or `sole` if only one
  antenna ever saw the tag).
- Antenna 0 owners render green, antenna 1 owners red, the `(N)` marker yellow.

---

## Tuning (all at the top of `rfid_arbiter.c`)

| Macro | Default | Effect |
|-------|---------|--------|
| `DEFAULT_POWER_MW` | `5` | **The critical knob.** Must be low enough that only the near antenna reads. Find it with `rfid_probe` (below). Overridable on the command line. |
| `READ_HI` | `0.60` | The winning antenna must read the tag at least this fraction of the time. |
| `READ_LO` | `0.25` | The other antenna must read at most this fraction. Between `READ_LO` and `READ_HI` is treated as undecided. |
| `INV_PER_SWEEP` | `5` | Inventories per antenna per sweep — the read-rate sample size. |
| `CONFIRM_STREAK` | `4` | Consecutive decisive sweeps before committing. |
| `RELEASE_MS` | `800` | How long a tag must be gone (both antennas) before the binding clears (mug removed). |
| `SCAN_MS` | `30` | Sleep between full sweeps. |
| `RATE_ALPHA` | `0.45` | EWMA weight for the newest sweep's read fraction. |

## Finding the right power

The method only works at a power where the **near** antenna reads but the
**far** one is silent. Use the probe to find it:

```bash
./compile_probe.sh
./rfid_probe 10        # watch the two per-source read counts
./rfid_probe 5
./rfid_probe 3
```

Put a mug over one antenna and step the power **down** until that antenna shows
`reads=20/20` while the other shows `reads=0/20`. Use that value (or 1 mW above
it for margin) as your power. Check the **other** antenna too — if the two
ports differ, pick a power that gives a clean split for **both** positions.
Then run:

```bash
./run.sh 5          # use the power you found
```

You should see `>> ANTENNA 0 OWNS ...` (or `ANTENNA 1`) within a fraction of a
second and nothing on the other slot. If a tag stays `pending` with
`BOTH-lower-power`, the power is too high; lower it. If commits are slow or a
mug is missed, raise it by 1 mW.

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
