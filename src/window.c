#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <ncurses.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "blackboard.h"



int main() {

    int shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open failed");
        return 1;
    }
    newBlackboard *bb = mmap(NULL, sizeof(newBlackboard), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (bb == MAP_FAILED) {
        perror("mmap failed");
        return 1;
    }

    sem_t *sem = sem_open(SEM_NAME, 0);
    if (sem == SEM_FAILED) {
        perror("sem_open failed");
        return 1;
    }

    initscr();
    cbreak();
    noecho();
    curs_set(FALSE);

    // ---------- COLOR SETUP ----------
    start_color();
    init_pair(1, COLOR_RED,   COLOR_BLACK);   
    init_pair(2, COLOR_GREEN, COLOR_BLACK); 
    init_pair(3, COLOR_CYAN,  COLOR_BLACK);  
    // ----------------------------------

    WINDOW *win = newwin(WIN_SIZE_Y+10, WIN_SIZE_X+3, 1, 1);

    while (1) {
        sem_wait(sem);

        werase(win);

        box(win, 0, 0);
        mvwprintw(win, 1, 1, "Drone Simulator"); 
        mvwprintw(win, 3, 1, "Drone Position: (%d, %d)", bb->drone_x, bb->drone_y);
        mvwprintw(win, 4, 1, "# targets and obstacles: %d - %d", bb->n_targets, bb->n_obstacles);
        mvwprintw(win, 5, 1, "Time: %.1f secs", bb->stats.time_elapsed);
      
     

        mvwprintw(win, 7, 1, "Map:");
        mvwprintw(win, 8, 0, "-");
        whline(win, '=', WIN_SIZE_X+2);

        for (int y = 0; y < WIN_SIZE_Y; y++) {
            for (int x = 1; x <= WIN_SIZE_X; x++) {

                // Obstacles - RED
                for (int i = 0; i < bb->n_obstacles; i++) {
                    if (bb->obstacle_xs[i] == x && bb->obstacle_ys[i] == y) {
                        wattron(win, COLOR_PAIR(1));
                        mvwprintw(win, 9 + y, x, "O");
                        wattroff(win, COLOR_PAIR(1));
                    }
                }

                // Targets - GREEN
                for (int i = 0; i < bb->n_targets; i++) {
                    if (bb->target_xs[i] == x && bb->target_ys[i] == y) {
                        wattron(win, COLOR_PAIR(2));
                        mvwprintw(win, 9 + y, x, "T");
                        wattroff(win, COLOR_PAIR(2));
                    }
                }

                // Drone - BLUE
                if (bb->drone_x == x && bb->drone_y == y) {
                    wattron(win, COLOR_PAIR(3));
                    mvwprintw(win, 9 + y, x, "D");
                    wattroff(win, COLOR_PAIR(3));
                }
            }
        }

        bb->stats.time_elapsed += RENDER_DELAY / 1000000.0;
        wrefresh(win);
        sem_post(sem); 
        usleep(RENDER_DELAY); // 100ms
    }

    delwin(win);
    endwin();
    sem_close(sem);
    munmap(bb, sizeof(newBlackboard));

    return 0;
}

