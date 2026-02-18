#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <stdbool.h>
#include "blackboard.h"
#include "logger.h"


void compute_repulsive_force(double *Fx, double *Fy, newBlackboard *bb);
void compute_attractive_force(double *Fx, double *Fy, newBlackboard *bb);


int main() {
    const char *shm_name = getenv("BB_SHM_NAME");
    if (!shm_name) shm_name = SHM_NAME;
    int shm_fd = shm_open(shm_name, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return 1;
    }
    newBlackboard *bb = mmap(NULL, sizeof(newBlackboard), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (bb == MAP_FAILED) {
        perror("mmap failed");
        return 1;
    }
    const char *sem_name = getenv("BB_SEM_NAME");
    if (!sem_name) sem_name = SEM_NAME;
    sem_t *sem = sem_open(sem_name, 0);
    if (sem == SEM_FAILED) {
        perror("sem_open failed");
        return 1;
    }
    int fd = open_watchdog_pipe(PIPE_DYNAMICS);
    logger("Dynamics started. PID: %d", getpid());

    double vx = 0, vy = 0;
    double Fx, Fy, repulsive_Fx, repulsive_Fy, attractive_Fx=0, attractive_Fy=0;
    double x_i = bb->drone_x, x_i_minus_1 = bb->drone_x, x_i_new;
    double y_i = bb->drone_y, y_i_minus_1 = bb->drone_y, y_i_new;
    time_t now = time(NULL);

    while (1){
        sem_wait(sem);

        Fx = bb->command_force_x;
        Fy = bb->command_force_y;
        repulsive_Fx = 0.0, repulsive_Fy = 0.0;
        attractive_Fx = 0.0, attractive_Fy = 0.0;
        if (bb->state != 0){  // only calculate these when running. the rest of the loop doesn't matter because they WILL be 0
            compute_repulsive_force(&repulsive_Fx, &repulsive_Fy, bb);
            compute_attractive_force(&attractive_Fx, &attractive_Fy, bb);
        }
        Fx += repulsive_Fx + attractive_Fx;
        Fy += repulsive_Fy + attractive_Fy;

        x_i_new = (1/(bb->physix.mass+bb->physix.visc_damp_coef*DT)) * (Fx * DT * DT - bb->physix.mass * (x_i_minus_1-2*x_i) + bb->physix.visc_damp_coef * DT * x_i);
        y_i_new = (1/(bb->physix.mass+bb->physix.visc_damp_coef*DT)) * (Fy * DT * DT - bb->physix.mass * (y_i_minus_1-2*y_i) + bb->physix.visc_damp_coef * DT * y_i);
        bb->drone_x = x_i_new;
        bb->drone_y = y_i_new;
        x_i_minus_1 = x_i;
        x_i = x_i_new;
        y_i_minus_1 = y_i;
        y_i = y_i_new;

        int play_w = bb_play_width(bb);
        int play_h = bb_play_height(bb);

        if (bb->drone_x < 1) {
            bb->drone_x = 1;
            x_i = bb->drone_x;
            x_i_minus_1 = bb->drone_x;
            vx = 0; Fx = 0;
        }
        if (bb->drone_x > play_w-1) {
            bb->drone_x = play_w-2;
            x_i = bb->drone_x;
            x_i_minus_1 = bb->drone_x;
            vx = 0; Fx = 0;
        }
        if (bb->drone_y < 1) {
            bb->drone_y = 1;
            y_i = bb->drone_y;
            y_i_minus_1 = bb->drone_y;
            vy = 0; Fy = 0;
        }
        if (bb->drone_y > play_h-1) {
            bb->drone_y = play_h-2;
            y_i = bb->drone_y;
            y_i_minus_1 = bb->drone_y;
            vy = 0; Fy = 0;
        }

        for (int i=0; i<bb->n_obstacles; i++){ // TODO MAYBE MERGE WITH OTHER LOOPS
            if (bb->drone_x == bb->obstacle_xs[i] && bb->drone_y == bb->obstacle_ys[i]){
                bb->stats.hit_obstacles += 1;
                bb->obstacle_xs[i] = -1;
                bb->obstacle_ys[i] = -1;
                logger("Drone hit an obstacle at position (%d, %d)", bb->drone_x, bb->drone_y);
            }
        }
        for (int i=0; i<bb->n_targets; i++){
            if (bb->drone_x == bb->target_xs[i] && bb->drone_y == bb->target_ys[i]){
                bb->stats.hit_targets += 1;
                bb->target_xs[i] = -1;
                bb->target_ys[i] = -1;
                logger("Drone got a target at position (%d, %d)", bb->drone_x, bb->drone_y);
            }
        }
        if (x_i != x_i_minus_1 || y_i != y_i_minus_1){  
            bb->stats.distance_traveled += sqrt((x_i - x_i_minus_1) * (x_i - x_i_minus_1) + (y_i - y_i_minus_1) * (y_i - y_i_minus_1));
        }
        sem_post(sem);
        if (difftime(time(NULL), now) >= 3){
            send_heartbeat(fd);
            now = time(NULL);
        }
        usleep(DT * 1000000);
    }
    if (fd >= 0) { close(fd); }
    munmap(bb, sizeof(newBlackboard));
    return 0;
}

void compute_repulsive_force(double *Fx, double *Fy, newBlackboard *bb) {
    *Fx = 0;
    *Fy = 0;
    double dx, dy, dist, repulsive;

    int play_w = bb_play_width(bb);
    int play_h = bb_play_height(bb);

    for (int i = 0; i < bb->n_obstacles; i++) {
        if (bb->obstacle_xs[i] < 1 || bb->obstacle_ys[i] < 1 || bb->obstacle_xs[i] >= play_w || bb->obstacle_ys[i] >= play_h) {
            continue;
        }
        dx = bb->obstacle_xs[i] - bb->drone_x;  
        dy = bb->obstacle_ys[i] - bb->drone_y;
        dist = sqrt(dx * dx + dy * dy);
        if (dist < bb->physix.radius && dist > 0) {
            repulsive = bb->physix.obst_repl_coef * 3 * (1.0 / dist - 1.0 / bb->physix.radius) / (dist * dist + EPSILON);
            *Fx -= repulsive * (dx / (dist + EPSILON));
            *Fy -= repulsive * (dy / (dist + EPSILON));
        }
    } // Obstacles

    if (bb->drone_x < bb->physix.radius) {
        repulsive = bb->physix.obst_repl_coef * (1.0 / (bb->drone_x + EPSILON) - 1.0 / bb->physix.radius) / (bb->drone_x * bb->drone_x + EPSILON);
        *Fx += repulsive;
    } // Left wall
    if (play_w - bb->drone_x < bb->physix.radius) {
        repulsive = bb->physix.obst_repl_coef * (1.0 / (play_w - bb->drone_x + EPSILON) - 1.0 / bb->physix.radius) / ((play_w - bb->drone_x) * (play_w - bb->drone_x) + EPSILON);
        *Fx -= repulsive;
    } // Right wall
    if (bb->drone_y < bb->physix.radius) {
        repulsive = bb->physix.obst_repl_coef * (1.0 / (bb->drone_y + EPSILON) - 1.0 / bb->physix.radius) / (bb->drone_y * bb->drone_y + EPSILON);
        *Fy += repulsive;
    } // Top wall
    if (play_h - bb->drone_y < bb->physix.radius) {
        repulsive = bb->physix.obst_repl_coef * (1.0 / (play_h - bb->drone_y + EPSILON) - 1.0 / bb->physix.radius) / ((play_h - bb->drone_y) * (play_h - bb->drone_y) + EPSILON);
        *Fy -= repulsive;
    } // Bottom wall

    // Limit the repulsion force to a maximum of 100  // TODO PARAMETER
    if (*Fx > 100){ *Fx = 100;}
    if (*Fy > 100){ *Fy = 100;}
    if (*Fx < -100){ *Fx = -100;}
    if (*Fy < -100){ *Fy = -100;}
}

void compute_attractive_force(double *Fx, double *Fy, newBlackboard *bb) {
    *Fx = 0;
    *Fy = 0;
    double dx, dy, dist, attractive;

    int play_w = bb_play_width(bb);
    int play_h = bb_play_height(bb);

    for (int i = 0; i < bb->n_targets; i++) {
        if (bb->target_xs[i] < 1 || bb->target_ys[i] < 1 || bb->target_xs[i] >= play_w || bb->target_ys[i] >= play_h) {
            continue;
        }
        dx = bb->target_xs[i] - bb->drone_x;
        dy = bb->target_ys[i] - bb->drone_y;
        dist = sqrt(dx * dx + dy * dy);

        if (dist < bb->physix.radius && dist > 0) {
            attractive = bb->physix.obst_repl_coef * 0.05 * (dist);
            *Fx += attractive * (dx / (dist + EPSILON));
            *Fy += attractive * (dy / (dist + EPSILON));
        }
    }
}

