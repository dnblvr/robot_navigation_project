#include <SDL2/SDL.h>
#include <math.h>
#include <float.h>

/* ===================== DEFINES ===================== */

#define WIDTH 800
#define HEIGHT 600
#define PPM 20
#define MAX_TRAJ 200
#define MAX_OBS 50
#define PI 3.141592653589793f

/* ===================== DATA STRUCTS ===================== */

typedef struct {
    float x, y, yaw, v, omega;
} RobotState;

typedef struct {
    float x, y, r;
} Obstacle;

typedef struct {
    float max_speed, min_speed;
    float max_omega;
    float max_accel, max_omega_accel;
    float v_res, omega_res;
    float dt, predict_time;
    float robot_radius;
    float to_goal_gain;
    float speed_gain;
    float obstacle_gain;
} Config;

/* ===================== COORDINATES ===================== */

void world_to_screen(float x, float y, int *sx, int *sy) {
    *sx = (int)(x * PPM + WIDTH / 2);
    *sy = (int)(-y * PPM + HEIGHT / 2);
}

void screen_to_world(int sx, int sy, float *x, float *y) {
    *x = (sx - WIDTH / 2) / (float)PPM;
    *y = -(sy - HEIGHT / 2) / (float)PPM;
}

/* ===================== DRAWING ===================== */

void draw_circle(SDL_Renderer *r, int cx, int cy, int radius) {
    for (int i = 0; i < 360; i++) {
        float rad = i * PI / 180.0f;
        int x = cx + (int)(radius * cosf(rad));
        int y = cy + (int)(radius * sinf(rad));
        SDL_RenderDrawPoint(r, x, y);
    }
}

void draw_filled_circle(SDL_Renderer *r, int cx, int cy, int radius) {
    for (int dy = -radius; dy <= radius; dy++) {
        int dx = (int)sqrtf(radius * radius - dy * dy);
        SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

void draw_line(SDL_Renderer *r, float x1, float y1, float x2, float y2) {
    int sx1, sy1, sx2, sy2;
    world_to_screen(x1, y1, &sx1, &sy1);
    world_to_screen(x2, y2, &sx2, &sy2);
    SDL_RenderDrawLine(r, sx1, sy1, sx2, sy2);
}

/* ===================== UTILS ===================== */

float normalize_angle(float a) {
    return atan2f(sinf(a), cosf(a));
}

float dist(float x1, float y1, float x2, float y2) {
    return hypotf(x1 - x2, y1 - y2);
}

/* ===================== TRAJECTORY ===================== */

int predict_trajectory(
    RobotState s, float v, float omega,
    Config *cfg, RobotState traj[]
) {
    int n = 0;
    float t = 0.0f;
    traj[n++] = s;

    while (t <= cfg->predict_time && n < MAX_TRAJ) {
        s.yaw += omega * cfg->dt;
        s.x += v * cosf(s.yaw) * cfg->dt;
        s.y += v * sinf(s.yaw) * cfg->dt;
        s.v = v;
        s.omega = omega;
        traj[n++] = s;
        t += cfg->dt;
    }
    return n;
}

/* ===================== COST FUNCTIONS ===================== */

float goal_cost(RobotState *last, float gx, float gy) {
    float dx = gx - last->x;
    float dy = gy - last->y;
    return fabsf(normalize_angle(atan2f(dy, dx) - last->yaw));
}

float obstacle_cost(
    RobotState traj[], int n,
    Obstacle obs[], int obs_n,
    Config *cfg
) {
    float min_d = FLT_MAX;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < obs_n; j++) {
            float d = dist(traj[i].x, traj[i].y, obs[j].x, obs[j].y);
            if (d <= cfg->robot_radius + obs[j].r)
                return FLT_MAX;
            if (d < min_d) min_d = d;
        }
    }
    return 1.0f / min_d;
}

/* ===================== DWA ===================== */

void dwa_control(
    RobotState *state, Config *cfg,
    float gx, float gy,
    Obstacle obs[], int obs_n,
    SDL_Renderer *r,
    float *best_v, float *best_o,
    RobotState best_traj[], int *best_n
) {
    float vmin = fmaxf(cfg->min_speed, state->v - cfg->max_accel * cfg->dt);
    float vmax = fminf(cfg->max_speed, state->v + cfg->max_accel * cfg->dt);
    float omin = fmaxf(-cfg->max_omega, state->omega - cfg->max_omega_accel * cfg->dt);
    float omax = fminf(cfg->max_omega, state->omega + cfg->max_omega_accel * cfg->dt);

    float best_cost = FLT_MAX;

    for (float v = vmin; v <= vmax; v += cfg->v_res) {
        for (float o = omin; o <= omax; o += cfg->omega_res) {

            RobotState traj[MAX_TRAJ];
            int n = predict_trajectory(*state, v, o, cfg, traj);

            float cost =
                cfg->to_goal_gain * goal_cost(&traj[n-1], gx, gy) +
                cfg->speed_gain * (cfg->max_speed - traj[n-1].v) +
                cfg->obstacle_gain * obstacle_cost(traj, n, obs, obs_n, cfg);

            if (cost < FLT_MAX) {
                SDL_SetRenderDrawColor(r, 220, 220, 220, 255);
                for (int i = 0; i < n - 1; i++)
                    draw_line(r, traj[i].x, traj[i].y,
                                 traj[i+1].x, traj[i+1].y);
            }

            if (cost < best_cost) {
                best_cost = cost;
                *best_v = v;
                *best_o = o;
                *best_n = n;
                for (int i = 0; i < n; i++)
                    best_traj[i] = traj[i];
            }
        }
    }
}

/* ===================== MOTION ===================== */

void update_motion(RobotState *s, float v, float o, float dt) {
    s->yaw += o * dt;
    s->x += v * cosf(s->yaw) * dt;
    s->y += v * sinf(s->yaw) * dt;
    s->v = v;
    s->omega = o;
}

/* ===================== MAIN ===================== */

int main(int argc, char *argv[]) {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window *win = SDL_CreateWindow(
        "Dynamic Window Approach (float)",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, 0
    );

    SDL_Renderer *r = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    Config cfg = {
        0.5f, -0.5f,
        40.0f * PI / 180.0f,
        0.2f, 40.0f * PI / 180.0f,
        0.05f, 0.1f * PI / 180.0f,
        0.5f, 9.0f,
        0.5f,
        0.1f, 1.0f, 0.5f
    };

    RobotState state = {-10.0f, -5.0f, PI / 4.0f, 0.0f, 0.0f};
    float goal_x = 10.0f, goal_y = 10.0f;

    Obstacle obs[MAX_OBS] = {
        {0,2,1},{2,4,1},{5,5,1},{8,7,1},
        {6,10,1},{10,4,1},{-2,8,1.5f}
    };
    int obs_n = 7;

    int running = 1;
    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;

            if (e.type == SDL_MOUSEBUTTONDOWN) {
                float wx, wy;
                screen_to_world(e.button.x, e.button.y, &wx, &wy);
                if (e.button.button == SDL_BUTTON_LEFT)
                    goal_x = wx, goal_y = wy;
                if (e.button.button == SDL_BUTTON_RIGHT && obs_n < MAX_OBS)
                    obs[obs_n++] = (Obstacle){wx, wy, 1.0f};
            }
        }

        SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
        SDL_RenderClear(r);

        float v = 0.0f, o = 0.0f;
        RobotState best_traj[MAX_TRAJ];
        int best_n = 0;

        dwa_control(&state, &cfg, goal_x, goal_y, obs, obs_n,
                    r, &v, &o, best_traj, &best_n);

        update_motion(&state, v, o, cfg.dt);

        SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
        for (int i = 0; i < best_n - 1; i++)
            draw_line(r, best_traj[i].x, best_traj[i].y,
                         best_traj[i+1].x, best_traj[i+1].y);

        SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
        for (int i = 0; i < obs_n; i++) {
            int ox, oy;
            world_to_screen(obs[i].x, obs[i].y, &ox, &oy);
            draw_circle(r, ox, oy, obs[i].r * PPM);
        }

        int gx, gy;
        world_to_screen(goal_x, goal_y, &gx, &gy);
        SDL_SetRenderDrawColor(r, 0, 255, 0, 255);
        draw_filled_circle(r, gx, gy, 6);

        int rx, ry;
        world_to_screen(state.x, state.y, &rx, &ry);
        SDL_SetRenderDrawColor(r, 0, 0, 255, 255);
        draw_circle(r, rx, ry, cfg.robot_radius * PPM);

        SDL_RenderPresent(r);
        SDL_Delay(30);
    }

    SDL_Quit();
    return 0;
}
