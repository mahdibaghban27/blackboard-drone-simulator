#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include "blackboard.h"


void update_forces(int key, int *Fx, int *Fy, newBlackboard *bb);
void reset_game(newBlackboard *bb);
WINDOW* draw_button(WINDOW *parent, int y, int x, const char *label, int width, int height);

int main(int argc, char *argv[]) {
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
    int fd = open_watchdog_pipe(PIPE_KEYBOARD);
    logger("Keyboard process started. PID: %d", getpid()); 

    int Fx = 0, Fy = 0;

    initscr();
    cbreak();  
    noecho();   
    keypad(stdscr, TRUE); 
    curs_set(0);

    WINDOW *win = newwin(25, 60, 1, 1);
    WINDOW *subwindows[13] = {0};
    nodelay(win, 1); nodelay(stdscr, 1);
    box(win, 0, 0);
    refresh();

    if (has_colors()){
        start_color(); init_pair(2, COLOR_GREEN, COLOR_BLACK); init_pair(3, COLOR_RED, COLOR_BLACK);
    }

    werase(win);
    wattrset(win, A_NORMAL);
    box(win, 0, 0);
    wattrset(win, A_BOLD);
    mvwprintw(win, 1, 3, "DRONE SIMULATOR");
    wattrset(win, A_NORMAL);
    subwindows[0]  = draw_button(win, 12, 4,  "Start (I)", 12, 3);
    subwindows[1]  = draw_button(win, 12, 18, "Reset (Y)", 12, 3);
    subwindows[2]  = draw_button(win, 12, 32, "MAP (M)", 12, 3);
    subwindows[3]  = draw_button(win, 12, 46, "Exit (ESC)", 12, 3);
    wattrset(win, A_NORMAL);
    subwindows[4]  = draw_button(win, 15, 4,  "Up-Left (Q)", 16, 3);
    subwindows[5]  = draw_button(win, 15, 23, "Up (W)", 16, 3);
    subwindows[6]  = draw_button(win, 15, 42, "Up-Right (E)", 16, 3);
    subwindows[7]  = draw_button(win, 18, 4,  "Left (A)", 16, 3);
    subwindows[8]  = draw_button(win, 18, 23, "Brake (X)", 16, 3);
    subwindows[9]  = draw_button(win, 18, 42, "Right (D)", 16, 3);
    subwindows[10] = draw_button(win, 21, 4,  "Down-Left (Z)", 16, 3);
    subwindows[11] = draw_button(win, 21, 23, "Down (S)", 16, 3);
    subwindows[12] = draw_button(win, 21, 42, "Down-Right (C)", 16, 3);
    wattrset(win, A_NORMAL);
    // some efforts has been made to reduce the memory consumption of subwindows

    time_t now = time(NULL);
    while (true) {  
        for (int i = 0; i < (int)(sizeof(subwindows)/sizeof(subwindows[0])); i++){
            if (subwindows[i]) { touchwin(subwindows[i]); wrefresh(subwindows[i]); }
        }
        mvwprintw(win, 4, 3, "Time Elapsed: %.2f", bb->stats.time_elapsed);
        mvwprintw(win, 6, 3, "Score: %.2f", bb->score);
        mvwprintw(win, 9, 3, "Command Forces: Fx = %d  , Fy = %d  ", Fx, Fy);
        touchwin(win);
        wrefresh(win);

        int ch = getch();
        sem_wait(sem); 
        if (ch == 'y') {
            reset_game(bb);
        }
        if (ch == 'i') {
            bb->state = 1;
        }
        if (ch == 27) {
            bb->state = 2;
            break;
        }
        if (ch == 'm'){
            if (bb->state == 1) {
                bb->state = 3;
            } else if (bb->state == 3) {
                bb->state = 1;
            }
        }
        update_forces(ch, &Fx, &Fy, bb);
        sem_post(sem);
        if (difftime(time(NULL), now) >= 3){
            send_heartbeat(fd);
            now = time(NULL);
        }
        // refresh();
    }

    if (fd >= 0) { close(fd); }
    delwin(win);
    endwin();
    sem_close(sem);
    munmap(bb, sizeof(newBlackboard));
    kill(getppid(), SIGTERM);

    return 0;
}

WINDOW* draw_button(WINDOW *parent, int y, int x, const char *label, int width, int height) {
    WINDOW *button_win = subwin(parent, height, width, y, x);
    box(button_win, 0, 0);
    int label_x = (width - strlen(label)) / 2;
    int label_y = height / 2;
    mvwprintw(button_win, label_y, label_x, "%s", label);
    wrefresh(button_win);
    return button_win;
}

void update_forces(int key, int *Fx, int *Fy, newBlackboard *bb) {
    switch (key) {
        case 'w': case KEY_UP:      *Fy -= 1;   break; // Up
        case 's': case KEY_DOWN:    *Fy += 1;   break; // Down
        case 'a': case KEY_LEFT:    *Fx -= 1;   break; // Left
        case 'd': case KEY_RIGHT:   *Fx += 1;   break; // Right
        case 'q': *Fx -= 1;         *Fy -= 1;   break; // Up-Left
        case 'e': *Fx += 1;         *Fy -= 1;   break; // Up-Right
        case 'z': *Fx -= 1;         *Fy += 1;   break; // Down-Left
        case 'c': *Fx += 1;         *Fy += 1;   break; // Down-Right
        case 'x': *Fx = 0;          *Fy = 0;    break; // Brake
    }
    bb->command_force_x = * Fx;
    bb->command_force_y = * Fy;
}

void reset_game(newBlackboard *bb) {
    // char text [30];
    // char *format = "Final score %.2f\n";
    // logger(sprintf(text, "Score recorded %.2f\n",  bb->score));
    bb->state = 0;
    bb->score = 0;
    bb->drone_x = 2;
    bb->drone_y = 2;
    bb->command_force_x = 0;
    bb->command_force_y = 0;
    bb->stats.time_elapsed = 0;
    bb->stats.hit_obstacles = 0;
    bb->stats.hit_targets = 0;
    bb->stats.distance_traveled = 0;
    for (int i = 0; i < MAX_OBJECTS; i++) {
        bb->obstacle_xs[i] = -1; bb->obstacle_ys[i] = -1;
    }
    for (int i = 0; i < MAX_OBJECTS; i++) {
        bb->target_xs[i] = -1; bb->target_ys[i] = -1;
    }
}

