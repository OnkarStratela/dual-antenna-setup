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

Each tag shows its own **RSSI in dBm** (one decimal place) followed by
the backscatter **Phase in degrees** (one decimal place), both in
brackets right after the antenna number, in the form
`(N)(rssi)(p<phase>) <epc>`. The reader stores RSSI internally as
tenths of dBm and the scanner divides by 10.0 for display. Phase is
returned as a raw 16-bit AVP from the Impinj E310 radio (12-bit
unsigned, `0..4095 ↔ 0..360°`) and the scanner converts it with
`raw · 360.0 / 4096.0`. The scanner prints every tag exactly as the
reader returns it for each antenna — there is no filtering or
cross-read arbitration in this version.

> Phase is directly proportional to the round-trip path length:
> `Δd = λ·Δφ / (4π)`. At 915 MHz, `λ ≈ 32.8 cm`, so one full 360° wrap
> corresponds to ~16.4 cm of round-trip motion (≈ 8.2 cm one-way). Use
> the unwrapped phase between successive sweeps to track sub-cm
> displacement, not the absolute value (which is offset by cabling and
> environment).

When **both antennas** see a tag:

```
[TX=30 mW] [(0)(-63.1)(p123.4) E2801160600002054E1A1234,   (1)(-65.0)(p210.8) E2801160600002054E1A5678]
```

When **only antenna 0** sees a tag (slot 1 is blank, columns unchanged):

```
[TX=30 mW] [(0)(-63.1)(p123.4) E2801160600002054E1A1234,                                              ]
```

When **only antenna 1** sees a tag (slot 0 is blank, columns unchanged):

```
[TX=30 mW] [                                            ,   (1)(-65.0)(p210.8) E2801160600002054E1A5678]
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
| `SRC/`           | CAEN light library sources/headers — small additive patch enables `AVP_PHASE` (flag bit `0x80`); see below |

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

## Phase support — what was patched in `SRC/`

The shipped CAEN "Light" library deliberately strips the phase bit
from the inventory flag mask, so out-of-the-box it cannot return
`AVP_PHASE`. We applied a small additive patch (no behaviour changes
for existing callers):

- `CAENRFIDTypes_Light.h` — added `PHASE = 0x0080` to
  `CAENRFID_InventoryFlag`, added `int16_t Phase` to `CAENRFIDTag`,
  added `has_PHASE` to `CAENRFIDInventoryParams`.
- `IO_Light.c` — added `AVP_PHASE` to the `uint16_t` branch of
  `getAVP()` so the AVP is decoded like RSSI.
- `CAENRFIDLib_Light.c` — widened the inventory flag mask from
  `0x017F` to `0x01FF` (so bit 7 is now sent to the reader) and added
  a **soft-fail** phase parser after the RSSI parser. If the firmware
  honours the bit, you get phase; if it silently omits the AVP, the
  inventory loop keeps working and `Tag.Phase` stays at `0`.

Calling code requests phase with `RSSI | PHASE` instead of just
`RSSI` when invoking `CAENRFID_InventoryTag`.

## Troubleshooting

If the reader fails to connect:
- `sudo chmod 666 /dev/ttyACM0`
- Or add yourself to the dialout group:
  `sudo usermod -a -G dialout $USER` (then log out/in)

If phase always reads `0.0`:
- Check the reader's firmware revision (R3100C Lepton3 ≥ 1.2.0 is what
  we developed against). Older firmwares may ignore the phase bit.
- If the value looks scaled wrong (e.g. always 0..90 instead of
  0..360), adjust `PHASE_RAW_FULLSCALE` in `rfid_gc_live.c` — that's
  the single constant controlling raw→degree conversion.
