#!/bin/bash
# Fix USB permissions if needed, compile, then run the arbiter.
# Pass an optional power in mW, e.g.  ./run.sh 120

if [ -e /dev/ttyACM0 ] && [ ! -r /dev/ttyACM0 ]; then
    echo "[ARB] Setting USB permissions..."
    sudo chmod 666 /dev/ttyACM0
fi

chmod +x compile.sh
./compile.sh || exit 1

./rfid_arbiter "$@"
