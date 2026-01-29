# Simulation Setup & Run Guide

## Requirements

- **GCC / build tools**
- **POSIX-compatible Linux environment**
- **ncurses**
- **cJSON**
- **pthread**
- (For **Assignment 3 / networked mode**) a terminal emulator for launching ncurses windows (recommended: **konsole**)

---

## Install dependencies (Ubuntu/Debian)

### Minimum (Assignment 2 / local simulation)

Install ncurses:

```bash
sudo apt install libncurses5-dev libncursesw5-dev
```

If you also need cJSON headers on some systems:

```bash
sudo apt install libcjson-dev
```

### Full (networked mode + konsole)

To run Assignment 3 (networked mode) and ensure a terminal emulator like **konsole** is available for launching ncurses windows, install:

```bash
sudo apt update && sudo apt install -y \
  build-essential make pkg-config cmake \
  libncurses-dev libncurses5-dev libncursesw5-dev \
  libcjson-dev \
  konsole konsole-kpart
```

---

## Run the simulation (Assignment 2)

This script compiles all modules and launches the full multi-process simulation.

```bash
chmod +x executer.sh
./executer.sh
./master
```

After compilation:
1. Select option **1** for **Assignment 2** and press **Enter**.
2. After the simulation starts, press the **`I`** key to begin the simulation.

---

## Run Assignment 3 (Networked mode)

### On **Server** machine

1. Run:

```bash
chmod +x executer.sh
./executer.sh
./master
```
2. Select mode **2**
3. Select role **1 (server)**
4. Enter a port (e.g. `6000`)

### On **Client** machine

1. Run:

```bash
chmod +x executer.sh
./executer.sh
./master
```
2. Select mode **2**
3. Select role **2 (client)**
4. Enter the server IP (e.g. `192.168.1.10`)
5. Enter the same port (e.g. `6000`)

