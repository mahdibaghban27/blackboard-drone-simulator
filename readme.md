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
- A central blackboard (shared state)
- A master process that spawns and supervises all children

**Assignment 2 additions:**
- A **Watchdog** process that monitors component liveness using heartbeat messages.
- A centralized **logger** that provides systematic debug output.
- Proper **IPC cleanup** (shared memory, semaphore, named pipes) to avoid leftover resources.

---

## 2. Repository Structure



```
.
├── bins
│   ├── Dynamics.out
│   ├── Keyboard.out
│   ├── Obstacle.out
│   ├── Target.out
│   ├── Watchdog.out
│   └── Window.out
├── config.json
├── executer.sh
├── logs
│   └── simulation.log
├── master
└── src
    ├── blackboard.c
    ├── blackboard.h
    ├── dynamics.c
    ├── keyboard.c
    ├── logger.c
    ├── logger.h
    ├── master.c
    ├── obstacle.c
    ├── target.c
    ├── watchdog.c
    └── window.c
```

## 3. System Architecture

The architecture follows the classical **Blackboard Model**:

- All processes share the same memory segment  
- Processes cooperate indirectly through the blackboard  
- Synchronization is enforced using a named semaphore  
- The master process handles spawning, supervision, and shutdown  
- A watchdog supervises process liveness using named pipes (FIFO)

### Architecture Diagram

```
                           +----------------------+
                           |    Master Process    |
                           |  (spawns/terminates) |
                           +----------+-----------+
                                      |
      +-------------------------------+---------------------------------------------+
      |               |               |               |              |              |
      v               v               v               v              v              v
+-----------+   +-----------+   +-----------+   +-----------+   +-----------+  +-----------+
| Blackboard |   |  Window   |   | Keyboard  |   | Dynamics  |   | Obstacle  |  |  Target   |
| (server)   |   | (ncurses) |   | (ncurses) |   |  Engine   |   | Generator |  | Generator |
+-----+-----+   +-----+-----+   +-----+-----+   +-----+-----+   +-----+-----+  +-----+-----+
      |               \             |               /                 |              |
      |                \            |              /                  |              |
      v                 v           v             v                   v              v
+--------------------------------------------------------------------------------------------+
|                         POSIX SHARED MEMORY (IPC CORE)                                     |
|            Data: newBlackboard   |   Sync: SEM_NAME   |   Name: SHM_NAME                   |
+--------------------------------------------------------------------------------------------+

                           (Assignment 2 - Fault Detection)
+-----------+        Heartbeat via Named Pipes (FIFO in /tmp)         +----------------------+
| Watchdog  | <-----------------------------------------------------> |  All child processes |
|           |   PIPE_WINDOW / PIPE_KEYBOARD / PIPE_DYNAMICS / ...     |  (send heartbeat)     |
+-----------+                                                          +----------------------+

                           (Assignment 2 - Systematic Debug Output)
+--------------------------------------------------------------------------------------------+
| LOGGER MODULE (logger.c / logger.h)                                                        |
| All components write systematic debug output to: logs/simulation.log                        |
+--------------------------------------------------------------------------------------------+


```


## 4. Components

### Blackboard (`blackboard.c`)
- Creates shared memory and semaphore  
- Initializes drone state and statistics  
- Periodically computes score  
- Reads `parameters.txt` to update counts of obstacles and targets  

### Dynamics (dynamics.c)

This module computes the drone motion using a discrete-time second-order dynamic model with viscous damping.

**Mathematical update equation:**

$$
x_{i+1} = \frac{F_x \cdot DT^2 - M (x_{i-1} - 2x_i) + K \cdot DT \cdot x_i}{M + K \cdot DT}
$$

$$
y_{i+1} = \frac{F_y \cdot DT^2 - M (y_{i-1} - 2y_i) + K \cdot DT \cdot y_i}{M + K \cdot DT}
$$


**Code implementation:**

```c
x_i_new = (1/(M + K*DT)) * (Fx*DT*DT - M*(x_i_minus_1 - 2*x_i) + K*DT*x_i);
y_i_new = (1/(M + K*DT)) * (Fy*DT*DT - M*(y_i_minus_1 - 2*y_i) + K*DT*y_i);

```

Force components:
- Command forces from keyboard  
- Repulsive forces from obstacles (Latombe/Khatib model)  
- Attractive forces from targets  
- Collision detection and distance tracking  
- Boundary constraints (geo-fencing)

### Window (`window.c`)
Ncurses-based visualization:
- Drone position  
- Obstacles and targets  
- Score and elapsed time  
- 2D grid map  

### Keyboard (`keyboard.c`)
Ncurses-based control interface:
- Updates `command_force_x` and `command_force_y`  
- Provides movement, braking, start, and exit controls  

### Obstacle Generator (`obstacle.c`)
- Periodically regenerates obstacles  
- Prevents overlap with drone position  

### Target Generator (`target.c`)
- Similar logic to obstacle generator  
- Periodic target regeneration  

### Watchdog (`watchdog.c`)
- Monitors liveness of critical processes  
- Uses heartbeat messages via named pipes (FIFO)  
- Terminates the system if a process becomes unresponsive  

### Logger (`logger.c`)
- Centralized, systematic debug logging  
- Logs process lifecycle events and errors  
- Outputs to `logs/simulation.log`  

### Master (`master.c`)
- Creates IPC resources (shared memory, semaphore, pipes)  
- Forks and execs all simulation components  
- Terminates all processes if one exits unexpectedly  
- Performs clean shutdown and IPC cleanup  

## 5. Build & Run

### Requirements
- GCC  
- POSIX-compatible Linux environment  
- ncurses  
- cJSON  
- pthread  

Install ncurses (Ubuntu/Debian):

```
sudo apt install libncurses5-dev libncursesw5-dev
```

### Run the simulation

```
chmod +x executer.sh
./executer.sh
./master
```
```
After compilation, select option 1 for Assignment 2 and press Enter.
After the simulation starts, press the **`I`** key to begin the simulation.
```
This script:
1. Switches to `src/`  
2. Compiles all modules  
3. Runs the master process  

## 6. Controls

- `W`, `A`, `S`, `D` – Up, Left, Down, Right  
- `Q`, `E`, `Z`, `C` – Diagonal movement  
- `X` – Brake (reset forces)  
- `ESC` – Exit simulation  

---

## 7. Configuration

Simulation parameters are defined in `config.json`, including:
- Number of obstacles and targets  
- Physical parameters (mass, damping, repulsion coefficient, radius)

Changes take effect without recompilation.

---

## 8. Assignment 3 (Partial)

A partial implementation of Assignment 3 is included:
- Basic socket-based client/server communication  
- Exchange of drone position between two simulations  

**Assignment 3 is not fully completed and is included for experimental purposes only.**

---

## 9. Notes

This project includes:
- Multi-process blackboard architecture  
- POSIX shared memory and semaphores  
- Ncurses-based UI  
- Physics-based drone simulation  
- Watchdog supervision  
- Systematic debug logging  
- Clean IPC resource cleanup  

After normal termination, no leftover FIFOs or shared memory objects should remain.

---
## 10. Changelog (Fixes from Assignment 1 feedback)

Based on the feedback from Assignment 1, the following significant issues were corrected and integrated into this Assignment 2 codebase.

### 1) Fix: "no pipe closing"
**Problem :** Named pipes (FIFOs) and pipe file descriptors were not properly released at shutdown, leaving `/tmp/*_pipe` files behind and causing resource leaks across runs.

**Fix:**
- Explicitly close the watchdog pipe FD when it is used (`close(fd)`).
- Added a cleanup routine that removes the named pipes created by the master process (`unlink()` for each FIFO).
- Added IPC cleanup to avoid leftovers between runs (unlink named semaphore and shared memory when appropriate).

**Result:** No leftover `/tmp/*_pipe` files after a clean shutdown and no leaked pipe descriptors.

### 2) Fix: "No systematic debug output"
**Problem:** Debug output was not systematic (scattered prints / missing structured logs), making it hard to trace process lifecycle and runtime events.

**Fix:**
- Introduced a centralized logging module (`logger.c/.h`) that writes to `logs/simulation.log`.
- Added structured log messages for process start/stop, errors, and watchdog heartbeats.

**Result:** A single consistent log file (`logs/simulation.log`) allows reproducible debugging and clearer evaluation.

## GitHub Repository

https://github.com/mahdibaghban27/blackboard-drone-simulator












