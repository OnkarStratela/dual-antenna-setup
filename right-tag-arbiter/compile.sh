#!/bin/bash
# Builds the arbiter against the bundled CAEN "Light" library (self-contained:
# this folder ships its own SRC/ copy and never touches the parent project).

echo "[ARB Build] Compiling rfid_arbiter..."

if [ ! -d "SRC" ]; then
    echo "[ARB Build] ERROR: SRC directory not found!"
    exit 1
fi
if [ ! -f "rfid_arbiter.c" ]; then
    echo "[ARB Build] ERROR: rfid_arbiter.c not found!"
    exit 1
fi

gcc \
  rfid_arbiter.c \
  SRC/host.c SRC/CAENRFIDLib_Light.c SRC/IO_Light.c \
  -ISRC \
  -o rfid_arbiter \
  -lpthread -lm \
  -Wall

if [ $? -eq 0 ]; then
    chmod +x rfid_arbiter
    echo "[ARB Build] Success! Run with: ./rfid_arbiter"
else
    echo "[ARB Build] Compilation failed!"
    exit 1
fi
