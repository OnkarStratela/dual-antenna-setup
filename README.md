# Dual-Antenna RFID Live Scanner

Minimal CAEN RFID setup that continuously scans **two antennas**
(`Source_0` and `Source_1`) at a **single global TX power**. After every
sweep it prints **`[]`** while nothing is visible, then a fixed-slot
**`[(0) tag, (1) tag]`** line once tags appear.

## Output format

**Continuous stream:** one line after each full Src0→Src1 sweep (~every **`GC_SCAN_MS`** ms).

The output uses **fixed slots** so tags never shift between positions —
slot 0 is always antenna 0, slot 1 is always antenna 1.

When **no tags** — only brackets (no Tx prefix):

```
[]
[]
[]
```

Each slot is padded to a fixed visible width, so the comma and the
closing `]` never shift columns. An antenna that misses renders as pure
whitespace — its `(N)` marker is **not** printed, so an empty slot can't
be mistaken for a half-shown entry.

Each tag shows its own **RSSI** in brackets right after the antenna
number, in the form `(N)(rssi) <epc>`. The reader reports RSSI in
**tenths of dBm** (e.g. `-650` means `-65.0 dBm`).

## Cross-read arbitration

Because the two antennas' radiation fields overlap above the trays, the
same tag is sometimes reported by both antennas in the same sweep. The
scanner resolves this **per sweep** with one simple rule:

- If the **same EPC** is reported by **both** antennas in a single
  sweep, the entry on the antenna with the **lower** RSSI is dropped,
  so the tag is printed only under the antenna that sees it strongest.
- Tags reported by only one antenna in a sweep are passed through
  unchanged (no suppression, no history).

This is intentionally a per-sweep decision with **no learning period
and no carry-over between sweeps** — every sweep is judged purely on
the RSSI values returned in that sweep. New EPCs work immediately and
the printed antenna can change instantly when the physical situation
changes.

When **both antennas** see a tag:

```
[TX=30 mW] [(0)(-45) E2801160600002054E1A1234   ,   (1)(-52) E2801160600002054E1A5678   ]
```

When **only antenna 0** sees a tag (slot 1 is blank, columns unchanged):

```
[TX=30 mW] [(0)(-45) E2801160600002054E1A1234   ,                                       ]
```

When **only antenna 1** sees a tag (slot 0 is blank, columns unchanged):

```
[TX=30 mW] [                                    ,   (1)(-52) E2801160600002054E1A5678   ]
```

Line rate defaults to **100 ms** between sweeps (≈ 10 `[ ]`/s idle). Tune
with `GC_SCAN_MS` in `rfid_gc_live.c`.

Colours on **non-empty** lines:

- **`[TX …]`** → cyan *(omitted entirely on empty `[ ]` lines)*
- **`(antenna)`** → yellow · Src0 tag → green · Src1 tag → red

## Files

| File | Purpose |
|------|---------|
| `rfid_gc_live.c` | The scanner program |
| `compile_gc.sh`  | Compiles `rfid_gc_live.c` against the CAEN library |
| `run_gc.sh`      | Fixes USB perms, compiles, then runs the scanner |
| `SRC/`           | CAEN light library sources/headers (do not modify) |

## How to run (Linux)

1. Connect the CAEN reader via USB (it will appear as `/dev/ttyACM0`).
2. From this folder:

   ```bash
   chmod +x run_gc.sh
   ./run_gc.sh
   ```

   This will set `/dev/ttyACM0` permissions if needed, compile, and run
   the scanner with the **default 30 mW** on both antennas.

3. Press `Ctrl+C` to stop.

## Tuning TX power (no rebuild required)

Power is a **single global value** applied to both antennas. Pass it as a
command-line argument (in **mW**, range 1–316). After compiling once,
run the binary directly:

```bash
./rfid_gc_live              # default: 30 mW (both antennas)
./rfid_gc_live 50           # 50 mW (both antennas)
./rfid_gc_live --help       # show usage
```

### Recommended starting point for a 150 mm two-antenna setup

- Boundary at the 75 mm midpoint → target read range ≈ 70 mm per antenna.
- Start at **30 mW** (the default). If the reader rejects that value,
  try `50`. If tags are missed within 6–7 cm, raise to `60–80`.
- If the same tag flickers between `(0)` and `(1)` near the midpoint,
  drop the global power until each antenna only sees its own half.

## Other configuration

Edit the macros at the top of `rfid_gc_live.c` to change:

- `GC_PORT` — serial port (e.g. `/dev/ttyUSB0`)
- `DEFAULT_POWER_MW` — default power when no CLI args are given
- `GC_SCAN_MS` — milliseconds between scan cycles

## Troubleshooting

If the reader fails to connect:
- `sudo chmod 666 /dev/ttyACM0`
- Or add yourself to the dialout group:
  `sudo usermod -a -G dialout $USER` (then log out/in)
