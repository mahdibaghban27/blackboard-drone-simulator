# 2D Drone Simulation – Multi-Process Blackboard Architecture

## 1. Overview

This project implements a 2D drone simulation using a multi-process architecture and a shared blackboard data structure.  
Processes communicate through POSIX shared memory and synchronize using a POSIX named semaphore.

The system consists of:
- A drone physics engine (Dynamics)
- A visualization interface (Window)
- A control interface (Keyboard)
- Random obstacle and target generators
- A central blackboard process
- A master process that spawns and supervises all children



## 2. Repository Structure

```
.
├── executer.sh
└── src
    ├── blackboard.c
    ├── blackboard.h
    ├── blb.out
    ├── dynamics.c
    ├── dyn.out
    ├── keyboard.c
    ├── key.out
    ├── master
    ├── master.c
    ├── obs.out
    ├── obstacle.c
    ├── parameters.txt
    ├── target.c
    ├── tar.out
    ├── window.c
    └── win.out
```

## 3. System Architecture

The architecture follows the classical Blackboard Model:

- All processes share the same memory segment  
- Processes cooperate indirectly  
- Synchronization is enforced using a named semaphore  
- Master process handles spawning and shutdown  

### Architecture Diagram

```
                       +----------------------+
                       |       Master         |
                       |   (Process Spawner)  |
                       +----------+-----------+
                                  |
                forks & execs     |
                                  v
    +-------------------+-------------------+-------------------+
    |                   |                   |                   |
    v                   v                   v                   v
+--------+        +-----------+       +-----------+       +-----------+
|Blackbd |        | Dynamics  |       |  Window   |       | Keyboard  |
| blb.out|        | dyn.out  |       | win.out   |       | key.out   |
+---+----+        +-----+-----+       +-----+-----+       +-----+-----+
    |                   |                   |                   |
    |                   |                   |                   |
    |                   |                   |                   |
    |          +--------+-------------------+-----------------------+
    |          |           Shared Memory (newBlackboard)            |
    |          | POSIX shm_open + mmap + named semaphore (SEM_NAME) |
    |          +--------+-------------------+-----------------------+
    |                   |                   |                   |
    v                   v                   v                   v
+-----------+     +-----------+       +-----------+       +-----------+
| Obstacle  |     |  Target   |       |  (Other)  |       |   ...     |
| obs.out   |     | tar.out   |       |           |       |           |
+-----------+     +-----------+       +-----------+       +-----------+
```

## 4. Components

### Blackboard (`blackboard.c`)
- Creates shared memory and semaphore  
- Initializes drone state and statistics  
- Periodically computes score  
- Reads `parameters.txt` to update counts of obstacles and targets  

### Dynamics (`dynamics.c`)
Implements physics based on a discrete-time mass-damper model:

```
x_i = (Fx * DT² / M)
      - (K/M) * (x_{i-1} - x_{i-2}) * DT
      + (2 * x_{i-1} - x_{i-2})
```

Additional features:
- Repulsive force from obstacles  
- Attractive force from targets  
- Collision detection  
- Travel distance accumulation  

### Window (`window.c`)
Ncurses visualization:
- Drone position  
- Obstacles (red)  
- Targets (green)  
- Drone (cyan)  
- Elapsed time  
- 2D grid map  

### Keyboard (`keyboard.c`)
Reads user input (WASD, QEZC, Arrows):

- Updates `command_force_x` and `command_force_y`  
- Provides ncurses-based control UI  

### Obstacle Generator (`obstacle.c`)
- Periodically clears and regenerates obstacles  
- Ensures obstacles do not overlap with the drone  

### Target Generator (`target.c`)
- Same logic as obstacle generator  
- Regenerates targets periodically  

### Master (`master.c`)
- Forks and execs: blackboard, window, dynamics, keyboard, obstacle, target  
- If any child exits → terminates all children  
- Provides controlled global shutdown  

## 5. Build & Run

### Requirements
- GCC  
- ncurses library  
- POSIX environment (Linux recommended)  

Install ncurses (Ubuntu/Debian):

```
sudo apt install libncurses5-dev libncursesw5-dev
```

### Run the simulation

```
chmod +x executer.sh
./executer.sh
```

This script:
1. Switches to `src/`  
2. Compiles all modules  
3. Runs the master process  

## 6. Controls

- `W`, `A`, `S`, `D` – Up, Left, Down, Right  
- `Q`, `E`, `Z`, `C` – Diagonals  
- `X` – Brake (forces → 0)  

## 7. Configuration

Modify `parameters.txt`:

```
num_obstacles=5
num_targets=5
```

Blackboard automatically reloads these values.

## 8. Notes

This implementation includes:
- Multi-process architecture  
- Blackboard with shared memory  
- POSIX semaphore synchronization  
- Ncurses visual/interactive UI  
- Physics engine with repulsive/attractive fields  

**Watchdog, logging system, and fault-tolerance mechanisms are not part of this version.**

