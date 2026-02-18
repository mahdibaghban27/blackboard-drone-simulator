#!/bin/bash

set -e

mkdir -p bins

gcc -o master               src/master.c -lcjson -lpthread -lm
gcc -o bins/Dynamics.out    src/dynamics.c -lm -lpthread
gcc -o bins/Keyboard.out    src/keyboard.c -lncurses -lpthread
gcc -o bins/Window.out      src/window.c -lncurses -lpthread -lm
gcc -o bins/Watchdog.out    src/watchdog.c -lpthread
gcc -o bins/Obstacle.out    src/obstacle.c -lpthread
gcc -o bins/Target.out      src/target.c -lpthread

echo "Build done. Now run: ./master"
