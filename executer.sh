#!/bin/bash

BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="$BASE_DIR/src"

echo "=== Compiling in $SRC_DIR ==="
cd "$SRC_DIR"

gcc -o blb.out blackboard.c -lrt -lm
gcc -o dyn.out dynamics.c -lrt -lm
gcc -o key.out keyboard.c -lncurses -lrt
gcc -o win.out window.c -lncurses -lrt
gcc -o obs.out obstacle.c -lrt -lm
gcc -o tar.out target.c -lrt -lm
gcc -o master master.c

echo "=== Compilation done ==="
echo "=== Sleeping 2 seconds... ==="
sleep 2

echo "=== Running master FROM src (cwd = src) ==="
./master   

