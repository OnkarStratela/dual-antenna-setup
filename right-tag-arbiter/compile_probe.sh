#!/bin/bash
# Builds the hardware probe (rfid_probe) against the bundled CAEN library.

echo "[probe Build] Compiling rfid_probe..."

gcc \
  rfid_probe.c \
  SRC/host.c SRC/CAENRFIDLib_Light.c SRC/IO_Light.c \
  -ISRC \
  -o rfid_probe \
  -lpthread -lm \
  -Wall

if [ $? -eq 0 ]; then
    chmod +x rfid_probe
    echo "[probe Build] Success! Run with: ./rfid_probe   (or ./rfid_probe 30)"
else
    echo "[probe Build] Compilation failed!"
    exit 1
fi
