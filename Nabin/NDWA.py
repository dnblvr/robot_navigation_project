#include <SDL2/SDL.h>
#include <math.h>
#include <float.h>

#define WIDTH 800
#define HEIGHT 600
#define PPM 20
#define MAX_TRAJ 200
#define MAX_OBS 50
#define PI 3.141592653589793

/* ===================== DATA STRUCTS ===================== */

typedef struct {
    double x, y, yaw, v, omega;
} RobotState;

typedef struct {
    double x, y, r;
} Obstacle;

typedef struct {
    double max_speed, min_speed;
    double max_omega;
    double max_accel, max_omega_accel;
    double v_res, omega_res;
    double dt, predict_time;
    double robot_radius;
    double to_goal_gain;
    double speed_gain;
    double obstacle_gain;
} Config;

/* ===================== COORDINATES ===================== */

void world_to_screen(double x, double y, int *sx, int *sy) {
    *sx = (int)(x * PPM + WIDTH / 2);
    *sy = (int)(-y * PPM + HEIGHT / 2);
}

void screen_to_world(int sx, int sy, double *x, double *y) {
    *x = (sx - WIDTH / 2) / (double)PPM;
    *y = -(sy - HEIGHT / 2) / (double)PPM;
}

/* ===================== DRAWING ===================== */

void draw_circle(SDL_Renderer *r, int cx, int cy, int radius) {
    for (int i = 0; i < 360; i++) {
        double rad = i * PI / 180.0;
        int x = cx + (int)(radius * cos(rad));
        int y = cy + (int)(radius * sin(rad));
        SDL_RenderDrawPoint(r, x, y);
    }
}

void draw_filled_circle(SDL_Renderer *r, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        int dx = (int)sqrt(radius * radius - dy * dy);
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

void draw_line(SDL_Renderer *r, double x1, double y1, double x2, double y2) {
    int sx1, sy1, sx2, sy2;
    world_to_screen(x1, y1, &sx1, &sy1);
    world_to_screen(x2, y2, &sx2, &sy2);
    SDL_RenderDrawLine(r, sx1, sy1, sx2, sy2);
}

/* ===================== UTILS ===================== */

double normalize_angle(double a) {
    return atan2(sin(a), cos(a));
}

double dist(double x1, double y1, double x2, double y2) {
    return hypot(x1 - x2, y1 - y2);
}

/* ===================== TRAJECTORY ===================== */

int predict_trajectory(
    RobotState s, double v, double omega,
    Config *cfg, RobotState traj[]
) {
    int n = 0;
    double t = 0;
    traj[n++] = s;

    while (t <= cfg->predict_time && n < MAX_TRAJ) {
        s.yaw += omega * cfg->dt;
        s.x += v * cos(s.yaw) * cfg->dt;
        s.y += v * sin(s.yaw) * cfg->dt;
        s.v = v;
        s.omega = omega;
        traj[n++] = s;
        t += cfg->dt;
    }
    return n;
}

/* ===================== COSTS ===================== */

double goal_cost(RobotState *last, double gx, double gy) {
    double dx = gx - last->x;
    double dy = gy - last->y;
    return fabs(normalize_angle(atan2(dy, dx) - last->yaw));
}

double obstacle_cost(
    RobotState traj[], int n,
    Obstacle obs[], int obs_n,
    Config *cfg
) {
    double min_d = DBL_MAX;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < obs_n; j++) {
            double d = dist(traj[i].x, traj[i].y, obs[j].x, obs[j].y);
            if (d <= cfg->robot_radius + obs[j].r)
                return DBL_MAX;
            if (d < min_d) min_d = d;
        }
    }
    return 1.0 / min_d;
}

/* ===================== DWA ===================== */

void dwa_control(
    RobotState *state, Config *cfg,
    double gx, double gy,
    Obstacle obs[], int obs_n,
    SDL_Renderer *r,
    double *best_v, double *best_o,
    RobotState best_traj[], int *best_n
) {
    double vmin = fmax(cfg->min_speed, state->v - cfg->max_accel * cfg->dt);
    double vmax = fmin(cfg->max_speed, state->v + cfg->max_accel * cfg->dt);
    double omin = fmax(-cfg->max_omega, state->omega - cfg->max_omega_accel * cfg->dt);
    double omax = fmin(cfg->max_omega, state->omega + cfg->max_omega_accel * cfg->dt);

    double best_cost = DBL_MAX; // FLT_MAX for floats

    for (double v = vmin; v <= vmax; v += cfg->v_res) {

        for (double o = omin; o <= omax; o += cfg->omega_res) {

            RobotState traj[MAX_TRAJ];
            int n = predict_trajectory(*state, v, o, cfg, traj);

            double cost =
                cfg->to_goal_gain * goal_cost(&traj[n-1], gx, gy) +
                cfg->speed_gain * (cfg->max_speed - traj[n-1].v) +
                cfg->obstacle_gain * obstacle_cost(traj, n, obs, obs_n, cfg);

            if (cost < DBL_MAX) {
                SDL_SetRenderDrawColor(r, 220,220,220,255);
                for (int i = 0; i < n - 1; i++)
                    draw_line(r, traj[i].x, traj[i].y,
                                 traj[i+1].x, traj[i+1].y);
            }

            if (cost < best_cost) {
                best_cost = cost;
                *best_v = v;
                *best_o = o;
                *best_n = n;
                for (int i = 0; i < n; i++) best_traj[i] = traj[i];
            }
        }
    }
}

/* ===================== MOTION ===================== */

void update_motion(RobotState *s, double v, double o, double dt) {
    s->yaw += o * dt;
    s->x += v * cos(s->yaw) * dt;
    s->y += v * sin(s->yaw) * dt;
    s->v = v;
    s->omega = o;
}

/* ===================== MAIN ===================== */

int main(int argc, char *argv[]) {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window *win = SDL_CreateWindow(
        "DWA - Pure SDL2",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, 0
    );

    SDL_Renderer *r = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    Config cfg = {
        0.5, -0.5,
        40 * PI / 180,
        0.2, 40 * PI / 180,
        0.05, 0.1 * PI / 180,
        0.5, 9.0,
        0.5,
        0.1, 1.0, 0.5
    };

    RobotState state = {-10, -5, PI/4, 0, 0};
    double goal_x = 10, goal_y = 10;

    Obstacle obs[MAX_OBS] = {
        {0,2,1},{2,4,1},{5,5,1},{8,7,1},
        {6,10,1},{10,4,1},{-2,8,1.5}
    };
    int obs_n = 7;

    int running = 1;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            if (e.type == SDL_MOUSEBUTTONDOWN) {
                double wx, wy;
                screen_to_world(e.button.x, e.button.y, &wx, &wy);
                if (e.button.button == SDL_BUTTON_LEFT)
                    goal_x = wx, goal_y = wy;
                if (e.button.button == SDL_BUTTON_RIGHT && obs_n < MAX_OBS)
                    obs[obs_n++] = (Obstacle){wx, wy, 1};
            }
        }

        SDL_SetRenderDrawColor(r,255,255,255,255);
        SDL_RenderClear(r);

        double v, o;
        RobotState best_traj[MAX_TRAJ];
        int best_n = 0;

        dwa_control(&state,&cfg,goal_x,goal_y,obs,obs_n,r,&v,&o,best_traj,&best_n);
        update_motion(&state,v,o,cfg.dt);

        SDL_SetRenderDrawColor(r,255,0,0,255);
        for (int i = 0; i < best_n - 1; i++)
            draw_line(r, best_traj[i].x,best_traj[i].y,
                         best_traj[i+1].x,best_traj[i+1].y);

        SDL_SetRenderDrawColor(r,0,0,0,255);
        for (int i = 0; i < obs_n; i++) {
            int ox, oy;
            world_to_screen(obs[i].x, obs[i].y, &ox, &oy);
            draw_circle(r, ox, oy, obs[i].r * PPM);
        }

        int gx, gy;
        world_to_screen(goal_x, goal_y, &gx, &gy);
        SDL_SetRenderDrawColor(r,0,255,0,255);
        draw_filled_circle(r, gx, gy, 6);

        int rx, ry;
        world_to_screen(state.x, state.y, &rx, &ry);
        SDL_SetRenderDrawColor(r,0,0,255,255);
        draw_circle(r, rx, ry, cfg.robot_radius * PPM);

        SDL_RenderPresent(r);
        SDL_Delay(30);
    }

    SDL_Quit();
    return 0;
}
