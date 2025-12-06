# 2D Drone Simulation – Multi-Process Blackboard Architecture

## 1. Overview

This project implements a 2D drone simulation using a multi-process architecture and a shared blackboard data structure.  
Processes communicate through POSIX shared memory and synchronize using a POSIX named semaphore.

![Demo](Image/demo.gif)

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
                            |    Master Process    |
                            |   (System Supervisor)|
                            +-----------+----------+
                                        |
                               Spawns Children
                                        |
      +--------------+--------------+---+--------------+--------------+
      |              |              |                  |              |
      v              v              v                  v              v
+-----------+  +-----------+  +-----------+      +-----------+  +-----------+
| Blackboard|  | Dynamics  |  |  Window   |      | Keyboard  |  | Generators|
| (blb.out) |  | (dyn.out) |  | (win.out) |      | (key.out) |  | (obs/tar) |
+-----+-----+  +-----+-----+  +-----+-----+      +-----+-----+  +-----+-----+
      |              |              |                  |              |
      |              |              |                  |              |
      v              v              v                  v              v
+-------------------------------------------------------------------------+
|                       POSIX SHARED MEMORY                               |
|     (Data Structure: newBlackboard  |  Synchronization: SEM_NAME)       |
+-------------------------------------------------------------------------+
```


## 4. Components

### Blackboard (`blackboard.c`)
- Creates shared memory and semaphore  
- Initializes drone state and statistics  
- Periodically computes score  
- Reads `parameters.txt` to update counts of obstacles and targets  

### Dynamics (dynamics.c)

This module computes the drone motion using the discrete-time second-order dynamic model defined in the assignment.

**Mathematical update equation (Assignment Eq. 3):**

$$
x_i = \frac{F_x \, DT^2}{M}
      - \frac{K}{M}(x_{i-1} - x_{i-2})\,DT
      + (2x_{i-1} - x_{i-2})
$$

and similarly:

$$
y_i = \frac{F_y \, DT^2}{M}
      - \frac{K}{M}(y_{i-1} - y_{i-2})\,DT
      + (2y_{i-1} - y_{i-2})
$$

**Code implementation:**

```c
double x_i_new =
    (Fx * DT * DT / M)
  - (K * (x_i - x_i_minus_1) * DT / M)
  + (2 * x_i - x_i_minus_1);

double y_i_new =
    (Fy * DT * DT / M)
  - (K * (y_i - y_i_minus_1) * DT / M)
  + (2 * y_i - y_i_minus_1);
```

**Force components:**
- Command forces from keyboard  
- Repulsive forces from obstacles (Latombe/Khatib model)  
- Attractive forces from targets  
- Collision detection & distance accumulation  
- Boundary constraints (geo-fence)


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



