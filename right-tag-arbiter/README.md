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

## Why this is reliably solvable

The owning antenna sits ~10 cm from the tag; the other antenna is ~150 cm away.
RFID is a round-trip link, so the path-loss *difference* between the two
antennas is approximately:

```
2 × 20·log10(150 / 10) ≈ 47 dB
```

That is an enormous, unambiguous gap. The crucial consequence for the beer:
water heavily attenuates 915 MHz, **but it attenuates both antenna paths by a
similar amount**, so the *difference* between the two antennas barely changes.

**Therefore the arbiter never decides on an absolute signal level — it decides
on which antenna is _relatively_ stronger.** That is what stays correct whether
the tray is dry or half full of beer.

---

## How it guarantees "0 overlap"

Each mug has a unique EPC. The arbiter keeps one record per EPC with a single
`owner` field (`antenna 0`, `antenna 1`, or "not yet decided"). Because an EPC
can only ever hold **one** owner value, it is structurally impossible for the
same tag to be reported on both antennas.

State machine per EPC:

1. **Deciding** — every sweep it polls both antennas and picks the candidate
   owner = the stronger antenna (or the only antenna that read it).
2. **Commit** — the candidate is locked in once it has, for `CONFIRM_STREAK`
   consecutive sweeps, satisfied **both**:
   - **(a) dominance:** stronger than the other antenna by ≥ `MARGIN` dB, *or*
     it is the only antenna that read the tag, **and**
   - **(b) near-presence:** its own RSSI is ≥ `NEAR_FLOOR` dBm — this rejects a
     faint long-range stray sneaking in on the far antenna.
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
| `DEFAULT_POWER_MW` | `100` | Higher = more reliable reads through beer. Separation does **not** depend on this (it is relative), so prefer enough power for solid reads. Also overridable on the command line. |
| `MARGIN_TENTHS` | `80` (8.0 dB) | Dominance required to pick a winner when *both* antennas see a tag. The geometry gives ~47 dB, so 8 dB is a safe, fade-tolerant threshold. |
| `NEAR_FLOOR_TENTHS` | `-720` (−72.0 dBm) | A genuine ~10 cm read sits well above this. Lower it only if heavy beer attenuation makes even the owning antenna read weaker than −72 dBm (watch for the "budget exceeded" warning). |
| `CONFIRM_STREAK` | `3` | Consecutive qualifying sweeps before committing. Raise for more caution, lower for faster commits. |
| `RELEASE_MS` | `800` | How long a tag must be gone before the binding clears (mug removed). |
| `SCAN_MS` | `60` | Time between full Source_0→Source_1 sweeps. |
| `DECISION_BUDGET_MS` | `3000` | The spec ceiling used only to flag slow commits. |

### Recommended first run

1. Start with the defaults (`./run.sh`).
2. Place a mug on antenna 0's spot. You should see
   `>> ANTENNA 0 OWNS ...` within a fraction of a second, and **nothing** on
   antenna 1. Repeat on antenna 1.
3. If a commit is ever slow or flagged over budget, raise power
   (`./run.sh 150`) or lower `NEAR_FLOOR_TENTHS`.
4. If you ever saw the far antenna try to claim a tag (it should not at 150 cm),
   raise `MARGIN_TENTHS` — but with this geometry you will have tens of dB to
   spare.

---

## Files

| File | Purpose |
|------|---------|
| `rfid_arbiter.c` | The arbiter (commit-and-latch state machine) |
| `compile.sh` | Builds against the bundled `SRC/` |
| `run.sh` | Fixes USB perms, compiles, runs (accepts optional power arg) |
| `SRC/` | Bundled copy of the CAEN "Light" library (independent of the parent folder) |
