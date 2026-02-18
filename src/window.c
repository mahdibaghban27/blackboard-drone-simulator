#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <semaphore.h>
#include <ncurses.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "blackboard.h"



static inline int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Map a coordinate from [src_lo..src_hi] to [dst_lo..dst_hi] (linear scaling). */
static inline int scale_axis(int v, int src_lo, int src_hi, int dst_lo, int dst_hi) {
    if (dst_hi <= dst_lo) return dst_lo;
    if (src_hi <= src_lo) return clampi(v, dst_lo, dst_hi);

    double t = (double)(v - src_lo) / (double)(src_hi - src_lo);
    int out = (int)lround((double)dst_lo + t * (double)(dst_hi - dst_lo));
    return clampi(out, dst_lo, dst_hi);
}

void render_loading(WINDOW *win);
void render_game(WINDOW *win, newBlackboard *bb);
void render_visualization(WINDOW * win, newBlackboard * bb);

/* Draw a border around the actual simulation world (bb->max_width x bb->max_height),
   so it's visually clear where the drone is allowed to move. */
void draw_world_border(WINDOW *win, int top, int left, int h, int w);

int main(int argc, char *argv[]) {
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
    int fd = open_watchdog_pipe(PIPE_WINDOW);
    logger("Window process started. PID: %d", getpid());

    // close(STDIN_FILENO); // close stdin to avoid keyboard input

    initscr();

    // We read the terminal size a lot; enabling keypad + non-blocking input
    // makes KEY_RESIZE events visible on some setups (helps in client mode).
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    // In network client mode we have to "fake" the server window size.
    // The idea is: create a fixed frame window sized exactly like the server.
    // If the current terminal is smaller, we *don't* crash/exit; we just ask the
    // user to resize the terminal and keep waiting.
    int env_lock = (getenv("BB_LOCK_SIZE") != NULL);
    WINDOW *frame = NULL;
    if (env_lock) {
        sem_wait(sem);
        int h = bb->max_height;
        int w = bb->max_width;
        sem_post(sem);

        // A bit of sanity so we don't wait forever on garbage values.
        if (h < 5 || w < 10) {
            h = 20;
            w = 20;
        }

        int cur_h, cur_w;
getmaxyx(stdscr, cur_h, cur_w);

// If the local terminal is smaller than the server's window, we don't exit.
// We render in the available space and scale the view a bit.
int draw_h = (cur_h < h) ? cur_h : h;
int draw_w = (cur_w < w) ? cur_w : w;

// But if the terminal is *really* tiny, ncurses becomes unusable - wait for resize.
while (draw_h < 10 || draw_w < 25) {
    (void)getch(); // let ncurses process resize events
    getmaxyx(stdscr, cur_h, cur_w);
    draw_h = (cur_h < h) ? cur_h : h;
    draw_w = (cur_w < w) ? cur_w : w;

    werase(stdscr);
    box(stdscr, 0, 0);
    mvwprintw(stdscr, 1, 2, "CLIENT MODE - window too small");
    mvwprintw(stdscr, 3, 2, "Server wants: %dx%d", w, h);
    mvwprintw(stdscr, 4, 2, "You have:     %dx%d", cur_w, cur_h);
    mvwprintw(stdscr, 6, 2, "Resize the terminal a bit and it will continue...");
    wrefresh(stdscr);
    usleep(200000);
}

if (cur_h < h || cur_w < w) {
    // One friendly heads-up: view will be scaled to fit your terminal.
    werase(stdscr);
    box(stdscr, 0, 0);
    mvwprintw(stdscr, 1, 2, "CLIENT MODE - scaled view");
    mvwprintw(stdscr, 3, 2, "Server window: %dx%d", w, h);
    mvwprintw(stdscr, 4, 2, "This terminal: %dx%d", cur_w, cur_h);
    mvwprintw(stdscr, 6, 2, "Tip: make the window wider for a 1:1 view.");
    wrefresh(stdscr);
    usleep(250000);
}

frame = newwin(draw_h, draw_w, 0, 0);
        if (!frame) {
            endwin();
            fprintf(stderr, "[Window] newwin() failed (h=%d w=%d)\n", h, w);
            return 1;
        }
    }

    if (has_colors()){
        start_color(); init_pair(1, COLOR_BLUE, COLOR_BLACK); init_pair(2, COLOR_GREEN, COLOR_BLACK); init_pair(3, COLOR_RED, COLOR_BLACK); init_pair(4, COLOR_BLUE, COLOR_BLACK);
    }
    cbreak();
    noecho();
    curs_set(0);
    wrefresh(stdscr);
    
    time_t now = time(NULL);
    while (1){
        sem_wait(sem);
        int lock_size = env_lock || bb->net_lock_size;
        if (!lock_size) {
            getmaxyx(stdscr, bb->max_height, bb->max_width);
        }
        bb->win_ready = 1;

        WINDOW *win = frame ? frame : stdscr;
        if (bb->state == 0){
            render_loading(win);
        } else {
            bb->stats.time_elapsed += RENDER_DELAY/1000000.0;
        }
        if (bb->state == 1){
            render_game(win, bb);
        }
        if (bb->state == 2){
            // char text [30];
            // logger(sprintf(text, "Final score %.2f\n",  bb->score));
            break;
        }
        if (bb->state == 3){
            render_visualization(win, bb);
        }
        sem_post(sem);
        if (difftime(time(NULL), now) >= 3){
            send_heartbeat(fd);
            now = time(NULL);
        }
        usleep(RENDER_DELAY);
    }

    if (fd >= 0) { close(fd); }
    if (frame) delwin(frame);
    endwin();
    sem_close(sem);
    munmap(bb, sizeof(newBlackboard));

    return 0;
}

void render_loading(WINDOW *win){
    werase(win);
    int h, w;
    getmaxyx(win, h, w);
    wattron(win, A_BOLD | A_BLINK);
    const char *text = "DRONE SIMULATOR 101";
    mvwprintw(win, h/2, (w - (int)strlen(text))/2, "%s", text);
    wattroff(win, A_BOLD | A_BLINK);
    box(win, 0, 0);
    wrefresh(win);
}

void render_game(WINDOW * win, newBlackboard *bb){
    werase(win);
    box(win, 0, 0);

    int wh, ww;
    getmaxyx(win, wh, ww);

    // Server uses bb->max_* for physics + networking conversion.
    // For *rendering* we may be smaller (client terminal), so we scale to fit ww/wh.
    int server_h = bb->max_height;
    int server_w = bb->max_width;
    int server_insp = bb_inspection_width(bb);
    int server_play = bb_play_width(bb);

    // Decide inspection panel for *this* window size.
    int insp_w = (ww < INSPECTION_WIDTH + 20) ? 0 : INSPECTION_WIDTH;
    int play_w = ww - insp_w;
    if (play_w < 10) { insp_w = 0; play_w = ww; }

    int split_x = play_w; // first column of the inspection panel (screen coords)

if (insp_w > 0 && split_x > 0 && split_x < ww - 1) {
        for (int y = 1; y < wh - 1; y++) {
            mvwaddch(win, y, split_x, ACS_VLINE);
        }

        int px = split_x + 2;
        int py = 1;
        mvwprintw(win, py++, px, "Time:  %.1f", bb->stats.time_elapsed);
        mvwprintw(win, py++, px, "Score: %.2f", bb->score);
        py++;
        mvwprintw(win, py++, px, "Hits O: %d", bb->stats.hit_obstacles);
        mvwprintw(win, py++, px, "Hits T: %d", bb->stats.hit_targets);
        py++;
        mvwprintw(win, py++, px, "Force: (%d,%d)", bb->command_force_x, bb->command_force_y);
        py++;
        mvwprintw(win, py++, px, "Keys: I start, Y reset");
    }

    for (int i = 0; i < bb->n_obstacles; i++){
        if (bb->obstacle_xs[i] < 1 || bb->obstacle_ys[i] < 1){
            continue;                                                                       // wont break bc if drone hits one, that index becomes -1
        }
        if (bb->obstacle_xs[i] >= split_x || bb->obstacle_ys[i] >= bb->max_height){     // published obstacles are out of bounds
            continue;
        }
        mvwaddch(win, bb->obstacle_ys[i], bb->obstacle_xs[i], 'O'|COLOR_PAIR(3));
    }
    for (int i = 0; i < bb->n_targets; i++){
        if (bb->target_xs[i] < 1 || bb->target_ys[i] < 1){
            continue;                                                                       // wont break bc if drone hits one, that index becomes -1
        }
        if (bb->target_xs[i] >= split_x || bb->target_ys[i] >= bb->max_height){         // published targets are out of bounds
            continue;
        }
        mvwaddch(win, bb->target_ys[i], bb->target_xs[i], 'T'|COLOR_PAIR(2));
    }

    // Assignment 3: show the remote peer drone (client side) if available.
    if (bb->remote_drone_x > 0 && bb->remote_drone_y > 0 &&
        bb->remote_drone_x < split_x && bb->remote_drone_y < bb->max_height) {
        mvwaddch(win, bb->remote_drone_y, bb->remote_drone_x, 'X'|A_BOLD|COLOR_PAIR(4));
    }
    mvwaddch(win, bb->drone_y, bb->drone_x, 'D'|A_BOLD|COLOR_PAIR(1));
    wrefresh(win);
}

void render_visualization(WINDOW * win, newBlackboard * bb){
    // A tiny "map" view. Nothing fancy, just so the (M) toggle still does something.
    render_game(win, bb);
    if (bb->drone_x > 0 && bb->drone_y > 0) {
        mvwaddch(win, bb->drone_y, bb->drone_x, ','|COLOR_PAIR(4));
    }
    wrefresh(win);
}

void draw_world_border(WINDOW *win, int top, int left, int h, int w) {
    /* Border covers the rectangle:
       (top,left) ... (top+h-1, left+w-1)
       This matches the world coordinates if you treat them as 0..w-1 / 0..h-1.
       Your objects already avoid <1 and >=max, so the border stays clean. */
    int y0 = top;
    int x0 = left;
    int y1 = top + h - 1;
    int x1 = left + w - 1;

    mvwaddch(win, y0, x0, ACS_ULCORNER);
    mvwaddch(win, y0, x1, ACS_URCORNER);
    mvwaddch(win, y1, x0, ACS_LLCORNER);
    mvwaddch(win, y1, x1, ACS_LRCORNER);

    for (int x = x0 + 1; x < x1; x++) {
        mvwaddch(win, y0, x, ACS_HLINE);
        mvwaddch(win, y1, x, ACS_HLINE);
    }

    for (int y = y0 + 1; y < y1; y++) {
        mvwaddch(win, y, x0, ACS_VLINE);
        mvwaddch(win, y, x1, ACS_VLINE);
    }
}

