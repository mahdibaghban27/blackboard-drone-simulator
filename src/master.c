#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdbool.h>
#include "blackboard.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdbool.h>
#include "blackboard.h"
#include <sys/stat.h>
#include <cjson/cJSON.h>

// Networking (Assignment 3 - socket protocol)
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <pthread.h>


void summon(char *args[]);
static int command_exists(const char *cmd);
double calculate_score(newBlackboard *bb);
void initialize_logger();
void cleanup_logger();
void read_json(newBlackboard *bb, bool first_time);
void create_named_pipe(const char *pipe_name);
void handle_sigchld(int sig);
void cleanup_pipes(void);
void cleanup_ipc(void);

/* Handle Ctrl+C / terminal close so we can shutdown both peers cleanly. */
void handle_sigint(int sig);


// Assignment 3 helpers
typedef struct {
    newBlackboard *bb;
    sem_t *sem;
    int is_server;
    char server_ip[64];
    int port;
} net_args_t;

static void *network_thread(void *arg);
static int send_line(int sock, const char *line);
static int recv_line(int sock, char *buf, size_t buflen);
static void to_virtual_coords(int width, int height, int x, int y, int *vx, int *vy);
static void from_virtual_coords(int width, int height, int vx, int vy, int *x, int *y);

static volatile sig_atomic_t terminated = 0;

/* Server-only: keep the simulation "world size" stable and predictable. */
#define SERVER_STD_WIDTH  80
#define SERVER_STD_HEIGHT 30

/* Small flag: once handshake finishes, client has the right size in bb->max_* */
static volatile sig_atomic_t net_size_ready = 0;

/* If the socket disconnects (server or client), exit the whole simulation locally. */
static volatile sig_atomic_t net_lost = 0;

/* Local quit request (Ctrl+C / terminal close). */
static volatile sig_atomic_t quit_requested = 0;

int main() {
    signal(SIGCHLD, handle_sigchld);
    signal(SIGINT,  handle_sigint);
    signal(SIGTERM, handle_sigint);

    create_named_pipe(PIPE_BLACKBOARD);
    create_named_pipe(PIPE_DYNAMICS);
    create_named_pipe(PIPE_KEYBOARD);
    create_named_pipe(PIPE_WINDOW);
    create_named_pipe(PIPE_OBSTACLE);
    create_named_pipe(PIPE_TARGET);
    int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return 1;
    }
    if (ftruncate(shm_fd, sizeof(newBlackboard)) == -1) {
        perror("ftruncate failed");
        return 1;
    }
    newBlackboard *bb = mmap(NULL, sizeof(newBlackboard), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (bb == MAP_FAILED) {
        perror("mmap failed");
        return 1;
    }
    memset(bb, 0, sizeof(newBlackboard));
    // Use the named semaphore as a mutex. Initial value MUST be 1 (unlocked),
    // otherwise every sem_wait() will deadlock at startup.
    sem_t *sem = sem_open(SEM_NAME, O_CREAT, 0666, 1);
    if (sem == SEM_FAILED) {
        perror("sem_open failed");
        return 1;
    }
    int fd = open_watchdog_pipe(PIPE_BLACKBOARD);
    read_json(bb, true);

    initialize_logger();
    logger("Blackboard server started. PID: %d", getpid());

    // INITIALIZE THE BLACKBOARD
    bb->state = 0;  // 0 for paused or waiting, 1 for running, 2 for quit
    bb->score = 0.0;
    bb->drone_x = 2; bb->drone_y = 2;
    bb->remote_drone_x = -1;
    bb->remote_drone_y = -1;
    for (int i = 0; i < MAX_OBJECTS; i++) {
        bb->obstacle_xs[i] = -1; bb->obstacle_ys[i] = -1;
        bb->target_xs[i] = -1; bb->target_ys[i] = -1;
    }
    bb->command_force_x = 0; bb->command_force_y = 0;
    bb->max_width   = 20; bb->max_height  = 20; // changes during runtime
    bb->stats.hit_obstacles = 0; bb->stats.hit_targets = 0;
    bb->stats.time_elapsed = 0.0; bb->stats.distance_traveled = 0.0;

    bb->score = calculate_score(bb);
    read_json(bb, true);

    int mode;
    while (1) {
        printf("\n=== === === ===\n\nWELCOME TO DRONE SIMULATION.\n\nChoose mode of operation ...\n"
            "(1): Local object generation and simulation\n"
            "(2): Networked simulation (Assignment 3 - socket client/server)\n"
            "Enter mode: ");
        if (scanf("%d", &mode) != 1) {
            fprintf(stderr, "Invalid input. Please enter a number (1 or 2).\n");
            while (getchar() != '\n'); // Clear input buffer
            continue;
        }
        if (mode == 1 || mode == 2) {
            break;
        }
        fprintf(stderr, "Invalid choice. Please enter 1 or 2.\n");
    }
    printf("\n=== === === ===\n\n");

    // In networked mode (Assignment 3), obstacle/target generators and watchdog are disabled per spec.
    const char *processNames[NUMBER_OF_PROCESSES] = {0};
    int processCount = 0;
    if (mode == 1) {
        const char *temp[] = {"Window", "Dynamics", "Keyboard", "Watchdog", "Obstacle", "Target"};
        memcpy((void*)processNames, temp, sizeof(temp));
        processCount = NUMBER_OF_PROCESSES;
    } else {
        const char *temp[] = {"Window", "Dynamics", "Keyboard"};
        memcpy((void*)processNames, temp, sizeof(temp));
        processCount = 3;
    }

    // Start networking (Assignment 3) in master so that it can bridge state to the blackboard.
    pthread_t net_th;
    net_args_t net_args;
    memset(&net_args, 0, sizeof(net_args));
    if (mode == 2) {
        int role = 0;
        printf("Network role: (1) server  (2) client : ");
        scanf("%d", &role);
        net_args.is_server = (role == 1);

        if (!net_args.is_server) {
            printf("Enter server IP (e.g. 192.168.1.10): ");
            scanf("%63s", net_args.server_ip);
        } else {
            strcpy(net_args.server_ip, "0.0.0.0");
        }
        printf("Enter port (>5000): ");
        scanf("%d", &net_args.port);
        net_args.bb = bb;
        net_args.sem = sem;

        /* In network mode we want both sides to start from a sane standard size.
           Client will still accept the "size w h" from the server during handshake. */
        sem_wait(sem);
        bb->max_width  = SERVER_STD_WIDTH;
        bb->max_height = SERVER_STD_HEIGHT;
        sem_post(sem);

        if (pthread_create(&net_th, NULL, network_thread, &net_args) != 0) {
            perror("pthread_create(network_thread)");
            // Continue without networking rather than aborting.
        }

        /* On client, wait until the handshake writes the final size into bb->max_*,
           so Window does not render with a stale/placeholder size at startup. */
        if (!net_args.is_server) {
            for (int i = 0; i < 200 && !net_size_ready; i++) {
                usleep(10000);
            }
        }

        fd = -1; // watchdog disabled in network mode
    }

    pid_t allPIDs[NUMBER_OF_PROCESSES] = {0};

    for (int i = 0; i < processCount; i++) {
        if (i == 0){
            sleep(1);
        }
        pid_t pid = fork();
        if (pid == 0) {
            // Child process
            // Window and Keyboard must run in their own terminals (each is an ncurses full-screen app).
            if (strcmp(processNames[i], "Window") == 0 || strcmp(processNames[i], "Keyboard") == 0) {

                /* In network mode, Window.c should not overwrite bb->max_* from terminal size. */
                if (mode == 2 && strcmp(processNames[i], "Window") == 0) {
                    setenv("BB_LOCK_SIZE", "1", 1);
                }

                const char *bin = (strcmp(processNames[i], "Window") == 0) ? "./bins/Window.out" : "./bins/Keyboard.out";

                if (mode == 2 && strcmp(processNames[i], "Window") == 0 && command_exists("xterm")) {
                    /* Simple: give Window a terminal big enough so the whole border is visible. */
                    char geom[32];
                    snprintf(geom, sizeof(geom), "%dx%d", SERVER_STD_WIDTH + 5, SERVER_STD_HEIGHT + 5);
                    char *execArgs[] = {"xterm", "-geometry", geom, "-e", (char*)bin, NULL};
                    summon(execArgs);
                } else if (command_exists("konsole")) {
                    char *execArgs[] = {"konsole", "-e", (char*)bin, NULL};
                    summon(execArgs);
                } else if (command_exists("gnome-terminal")) {
                    // Keep terminal open so errors are visible.
                    char cmd[256];
                    snprintf(cmd, sizeof(cmd), "%s; exec bash", bin);
                    char *execArgs[] = {"gnome-terminal", "--", "bash", "-lc", cmd, NULL};
                    summon(execArgs);
                } else if (command_exists("xterm")) {
                    char *execArgs[] = {"xterm", "-e", (char*)bin, NULL};
                    summon(execArgs);
                } else {
                    // Fallback: run in the current terminal (may conflict with other ncurses apps).
                    char *execArgs[] = {(char*)bin, NULL};
                    summon(execArgs);
                }
            } else {
                char binPath[128];
                snprintf(binPath, sizeof(binPath), "./bins/%s.out", processNames[i]);
                char *execArgs[] = {binPath, NULL};
                summon(execArgs);
            }

            exit(EXIT_FAILURE);
        } else if (pid < 0) {
            perror("fork failed");
            return EXIT_FAILURE;
        } else {
            allPIDs[i] = pid;
            printf("Launched %s, PID: %d\n", processNames[i], pid);  // TODO DELETE LATER
        }
    }

    while (1) {
        if (terminated) {
            break;
        }

        /* If the remote peer disconnects (or we requested quit), shutdown locally too.
           In server mode, the network thread will also send 'q' so the client exits cleanly. */
        if (mode == 2 && (net_lost || quit_requested)) {
            sem_wait(sem);
            bb->state = 2;
            sem_post(sem);
            terminated = 1;
            break;
        }

        sem_wait(sem);

        /* Server-only: keep dimensions stable in shared memory.
           (Just in case another node tries to rewrite them.) */
        if (mode == 2 && net_args.is_server) {
            bb->max_width  = SERVER_STD_WIDTH;
            bb->max_height = SERVER_STD_HEIGHT;
        }

        bb->score = calculate_score(bb);
        read_json(bb, true);
        sem_post(sem);
        if (fd >= 0) send_heartbeat(fd);
        sleep(BLACKBOARD_CHECK_DELAY);  // freq of 0.2 Hz
    }

    // Wait for any process to terminate
    int status;
    pid_t terminatedPid = wait(&status);
    if (terminatedPid == -1) {
        perror("wait failed");
        return EXIT_FAILURE;
    }

    printf("Process %d terminated. Terminating all other processes...\n", terminatedPid);

    // Terminate remaining processes
    for (int i = 0; i < processCount; i++) {
        if (allPIDs[i] != 0 && allPIDs[i] != terminatedPid) {
            kill(allPIDs[i], SIGTERM);
        }
    }
    if (fd >= 0) { close(fd); }  // close pipe
    cleanup_logger();
    sem_close(sem);

    munmap(bb, sizeof(newBlackboard));

      // Remove master-created named pipes from /tmp to avoid leftovers between runs

      cleanup_pipes();
      cleanup_ipc();

    while (wait(NULL) > 0);

    printf("All processes terminated. Exiting master process.\n");

    return EXIT_SUCCESS;

}

// ---------------- Assignment 3 (socket protocol) ----------------

static int send_line(int sock, const char *line) {
    size_t len = strlen(line);
    const char *p = line;
    while (len > 0) {
        ssize_t n = send(sock, p, len, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += (size_t)n;
        len -= (size_t)n;
    }
    return 0;
}

static int recv_line(int sock, char *buf, size_t buflen) {
    // Read until '\n' or buffer full-1.
    size_t i = 0;
    while (i + 1 < buflen) {
        char c;
        ssize_t n = recv(sock, &c, 1, 0);
        if (n == 0) {
            return 0; // closed
        }
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (c == '\r') continue;
        if (c == '\n') break;
        buf[i++] = c;
    }
    buf[i] = '\0';
    return 1;
}

static void to_virtual_coords(int width, int height, int x, int y, int *vx, int *vy) {
    // Virtual system origin is bottom-left.
    *vx = x;
    *vy = (height - 1) - y;
}

static void from_virtual_coords(int width, int height, int vx, int vy, int *x, int *y) {
    (void)width;
    *x = vx;
    *y = (height - 1) - vy;
}

static void *network_thread(void *arg) {
    net_args_t *na = (net_args_t*)arg;
    int sock = -1;

    if (na->is_server) {
        int lsock = socket(AF_INET, SOCK_STREAM, 0);
        if (lsock < 0) return NULL;
        int opt = 1;
        setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t)na->port);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(lsock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(lsock);
            return NULL;
        }
        if (listen(lsock, 1) < 0) {
            close(lsock);
            return NULL;
        }
        sock = accept(lsock, NULL, NULL);
        close(lsock);
        if (sock < 0) return NULL;
    } else {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return NULL;
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t)na->port);
        if (inet_pton(AF_INET, na->server_ip, &addr.sin_addr) != 1) {
            close(sock);
            return NULL;
        }
        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock);
            return NULL;
        }
    }

    char buf[256];

    // Handshake
    if (na->is_server) {
        if (send_line(sock, "ok\n") < 0) goto lost;
        if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost;
        // Send size (in characters)
        int w, h;
        sem_wait(na->sem);
        w = na->bb->max_width;
        h = na->bb->max_height;
        sem_post(na->sem);
        snprintf(buf, sizeof(buf), "size %d %d\n", w, h);
        if (send_line(sock, buf) < 0) goto lost;
        if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // sok
        net_size_ready = 1;
    } else {
        if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // ok
        if (send_line(sock, "ook\n") < 0) goto lost;
        if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // size w h
        int w = 0, h = 0;
        if (sscanf(buf, "size %d %d", &w, &h) == 2) {
            sem_wait(na->sem);
            na->bb->max_width = w;
            na->bb->max_height = h;
            sem_post(na->sem);
        }
        if (send_line(sock, "sok\n") < 0) goto lost;
        net_size_ready = 1;
    }

    // Main exchange loop
    while (1) {
        if (na->is_server) {
            // Quit?
            sem_wait(na->sem);
            int st = na->bb->state;
            int w = na->bb->max_width;
            int h = na->bb->max_height;
            int x = na->bb->drone_x;
            int y = na->bb->drone_y;
            sem_post(na->sem);

            if (st == 2) {
                if (send_line(sock, "q\n") < 0) goto lost;
                if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // qok
                break;
            }

            // Send drone position
            if (send_line(sock, "drone\n") < 0) goto lost;
            int vx, vy;
            to_virtual_coords(w, h, x, y, &vx, &vy);
            snprintf(buf, sizeof(buf), "%d %d\n", vx, vy);
            if (send_line(sock, buf) < 0) goto lost;
            if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // dok

            // Receive obstacle (client's drone)
            if (send_line(sock, "obst\n") < 0) goto lost;
            if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // x y
            int ovx, ovy;
            if (sscanf(buf, "%d %d", &ovx, &ovy) == 2) {
                int ox, oy;
                from_virtual_coords(w, h, ovx, ovy, &ox, &oy);
                sem_wait(na->sem);
                na->bb->obstacle_xs[0] = ox;
                na->bb->obstacle_ys[0] = oy;
                na->bb->n_obstacles = 1;
                sem_post(na->sem);
            }
            if (send_line(sock, "pok\n") < 0) goto lost;
        } else {
            int r = recv_line(sock, buf, sizeof(buf));
            if (r <= 0) goto lost;

            if (strcmp(buf, "q") == 0) {
                if (send_line(sock, "qok\n") < 0) goto lost;
                break;
            }

            if (strcmp(buf, "drone") == 0) {
                if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // x y
                int vx, vy;
                if (sscanf(buf, "%d %d", &vx, &vy) == 2) {
                    int w, h;
                    sem_wait(na->sem);
                    w = na->bb->max_width;
                    h = na->bb->max_height;
                    int x, y;
                    from_virtual_coords(w, h, vx, vy, &x, &y);
                    na->bb->remote_drone_x = x;
                    na->bb->remote_drone_y = y;
                    sem_post(na->sem);
                }
                if (send_line(sock, "dok\n") < 0) goto lost;
            } else if (strcmp(buf, "obst") == 0) {
                // Send our drone position as obstacle
                int w, h, x, y;
                sem_wait(na->sem);
                w = na->bb->max_width;
                h = na->bb->max_height;
                x = na->bb->drone_x;
                y = na->bb->drone_y;
                sem_post(na->sem);
                int vx, vy;
                to_virtual_coords(w, h, x, y, &vx, &vy);
                snprintf(buf, sizeof(buf), "%d %d\n", vx, vy);
                if (send_line(sock, buf) < 0) goto lost;
                if (recv_line(sock, buf, sizeof(buf)) <= 0) goto lost; // pok
            }
        }

        usleep(30000); // ~33 Hz
    }

    if (sock >= 0) close(sock);
    return NULL;

lost:
    net_lost = 1;
    if (sock >= 0) close(sock);
    return NULL;
}

static int command_exists(const char *cmd) {
    char buf[256];
    snprintf(buf, sizeof(buf), "command -v %s >/dev/null 2>&1", cmd);
    return system(buf) == 0;
}

void summon(char *args[]) {
    if (execvp(args[0], args) == -1) {
        perror("execvp failed");
        exit(EXIT_FAILURE);
    }
}

void read_json(newBlackboard *bb, bool first_time) {
    const char *filename = JSON_PATH;
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Could not open file");
    }
    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);
    char *data = (char *)malloc(length + 1);
    if (!data) {
        perror("Memory allocation failed for JSON config");
        fclose(file);
    }
    fread(data, 1, length, file);
    data[length] = '\0';
    fclose(file);
    cJSON *json = cJSON_Parse(data);
    if (!json) {
        printf("Error parsing JSON: %s\n", cJSON_GetErrorPtr());
    }
    cJSON *item;  // TODO CHECK FOR NULL and CORRUPTED JSON
    if ((item = cJSON_GetObjectItem(json, "num_obstacles")))    bb->n_obstacles = item->valueint;
    if ((item = cJSON_GetObjectItem(json, "num_targets")))      bb->n_targets = item->valueint;
    if ((item = cJSON_GetObjectItem(json, "mass")))             bb->physix.mass = item->valueint;
    if ((item = cJSON_GetObjectItem(json, "visc_damp_coef")))   bb->physix.visc_damp_coef = item->valueint;
    if ((item = cJSON_GetObjectItem(json, "obst_repl_coef")))   bb->physix.obst_repl_coef = item->valueint;
    if ((item = cJSON_GetObjectItem(json, "radius")))           bb->physix.radius = item->valueint;
    cJSON_Delete(json);
    free(data);
}

double calculate_score(newBlackboard *bb) {
    double score = (double)bb->stats.hit_targets        * 30.0 -
                   (double)bb->stats.hit_obstacles      * 5.0 -
                   bb->stats.time_elapsed               * 0.05 -
                   bb->stats.distance_traveled          * 0.1;
    return score;
}

void initialize_logger() {
    if (access("./logs/simulation.log", F_OK) == 0) {
        if (unlink("./logs/simulation.log") == 0) {
            printf("Existing simulation.log file deleted successfully.\n");
        } else {
            perror("Failed to delete simulation.log");
        }
    } else {
        printf("No existing simulation.log file found.\n");
    }
    if (!log_file) {
        log_file = fopen("./logs/simulation.log", "a");
        if (!log_file) {
            perror("Unable to open log file");
            return;
        }
    }
}

void cleanup_logger() {
    pthread_mutex_lock(&logger_mutex);
    if (log_file) {
        fclose(log_file);
        log_file = NULL;
    }
    pthread_mutex_unlock(&logger_mutex);
}

void create_named_pipe(const char *pipe_name) {
    if (access(pipe_name, F_OK) == -1) {
        if (mkfifo(pipe_name, 0666) == -1) {
            perror("Failed to create named pipe");
            exit(EXIT_FAILURE);
        }
    }
}

void cleanup_pipes(void) {
    unlink(PIPE_BLACKBOARD);
    unlink(PIPE_DYNAMICS);
    unlink(PIPE_KEYBOARD);
    unlink(PIPE_WINDOW);
    unlink(PIPE_OBSTACLE);
    unlink(PIPE_TARGET);
}

void cleanup_ipc(void) {
    sem_unlink(SEM_NAME);
    shm_unlink(SHM_NAME);
}

void handle_sigchld(int sig) {
    terminated = 1;
}

void handle_sigint(int sig) {
    quit_requested = 1;
}

