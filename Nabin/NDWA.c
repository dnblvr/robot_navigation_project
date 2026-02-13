#include <SDL2/SDL.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <time.h>

/* ===================== DEFINES ===================== */
#define WIDTH 800
#define HEIGHT 600
#define PPM 20
#define MAX_TRAJ 200
#define MAX_OBS 200
#define PI 3.141592653589793f

/* ===== ROOM PARAMETERS ===== */
#define ROOM_MIN_X -10.0f
#define ROOM_MAX_X  10.0f
#define ROOM_MIN_Y -7.0f
#define ROOM_MAX_Y  7.0f
#define WALL_SPACING 1.0f
#define WALL_RADIUS 0.6f

/* ===== RANDOM OBSTACLES ===== */
#define RANDOM_OBS  8
#define RANDOM_RADIUS_MIN 0.4f
#define RANDOM_RADIUS_MAX 0.6f

#define SIM_TIME 180.0f   /* 3 minutes */

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

/* ===================== DRAWING ===================== */
void draw_circle(SDL_Renderer *r, int cx, int cy, int radius) {
    for (int i = 0; i < 360; i++) {
        float a = i * PI / 180.0f;
        SDL_RenderDrawPoint(r, cx + (int)(radius * cosf(a)), cy + (int)(radius * sinf(a)));
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

float frand(float min, float max) {
    return min + (float)rand() / RAND_MAX * (max - min);
}

/* ===================== ROOM + OBSTACLES ===================== */
void build_rectangle_room(Obstacle obs[], int *obs_n) {
    *obs_n = 0;

    /* walls */
    for (float x = ROOM_MIN_X; x <= ROOM_MAX_X; x += WALL_SPACING) {
        obs[(*obs_n)++] = (Obstacle){x, ROOM_MIN_Y, WALL_RADIUS};
        obs[(*obs_n)++] = (Obstacle){x, ROOM_MAX_Y, WALL_RADIUS};
    }
    for (float y = ROOM_MIN_Y; y <= ROOM_MAX_Y; y += WALL_SPACING) {
        obs[(*obs_n)++] = (Obstacle){ROOM_MIN_X, y, WALL_RADIUS};
        obs[(*obs_n)++] = (Obstacle){ROOM_MAX_X, y, WALL_RADIUS};
    }

    /* random internal obstacles */
    for (int i = 0; i < RANDOM_OBS && *obs_n < MAX_OBS; i++) {
        obs[(*obs_n)++] = (Obstacle){
            frand(ROOM_MIN_X + 1.5f, ROOM_MAX_X - 1.5f),
            frand(ROOM_MIN_Y + 1.5f, ROOM_MAX_Y - 1.5f),
            frand(RANDOM_RADIUS_MIN, RANDOM_RADIUS_MAX)
        };
    }
}

/* ===================== TRAJECTORY ===================== */
int predict_trajectory(RobotState s, float v, float o, Config *cfg, RobotState traj[]) {
    int n = 0;
    float t = 0.0f;
    traj[n++] = s;

    while (t <= cfg->predict_time && n < MAX_TRAJ) {
        s.yaw += o * cfg->dt;
        s.x += v * cosf(s.yaw) * cfg->dt;
        s.y += v * sinf(s.yaw) * cfg->dt;
        s.v = v;
        s.omega = o;
        traj[n++] = s;
        t += cfg->dt;
    }
    return n;
}

/* ===================== COSTS ===================== */
float goal_cost(RobotState *s, float gx, float gy) {
    float dx = gx - s->x;
    float dy = gy - s->y;
    return fabsf(normalize_angle(atan2f(dy, dx) - s->yaw));
}

float obstacle_cost(RobotState traj[], int n, Obstacle obs[], int obs_n, Config *cfg) {
    float min_d = FLT_MAX;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < obs_n; j++) {
            float d = dist(traj[i].x, traj[i].y, obs[j].x, obs[j].y);
            if (d <= cfg->robot_radius + obs[j].r) return FLT_MAX;
            if (d < min_d) min_d = d;
        }
    }
    return 1.0f / min_d;
}

/* ===================== DWA ===================== */
void dwa_control(RobotState *state, Config *cfg, float gx, float gy, Obstacle obs[], int obs_n, float *best_v, float *best_o, RobotState best_traj[], int *best_n) {
    float vmin = fmaxf(cfg->min_speed, state->v - cfg->max_accel * cfg->dt);
    float vmax = fminf(cfg->max_speed, state->v + cfg->max_accel * cfg->dt);
    float omin = fmaxf(-cfg->max_omega, state->omega - cfg->max_omega_accel * cfg->dt);
    float omax = fminf(cfg->max_omega, state->omega + cfg->max_omega_accel * cfg->dt);

    float best_cost = FLT_MAX;

    for (float v = vmin; v <= vmax; v += cfg->v_res) {
        for (float o = omin; o <= omax; o += cfg->omega_res) {
            RobotState traj[MAX_TRAJ];
            int n = predict_trajectory(*state, v, o, cfg, traj);
            float cost = cfg->to_goal_gain * goal_cost(&traj[n-1], gx, gy) +
                         cfg->speed_gain * (cfg->max_speed - traj[n-1].v) +
                         cfg->obstacle_gain * obstacle_cost(traj, n, obs, obs_n, cfg);
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
void update_motion(RobotState *s, float v, float o, float dt) {
    s->yaw += o * dt;
    s->x += v * cosf(s->yaw) * dt;
    s->y += v * sinf(s->yaw) * dt;
    s->v = v;
    s->omega = o;
}

/* ===================== MAIN ===================== */
int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    SDL_Init(SDL_INIT_VIDEO);
    srand((unsigned int)time(NULL));

    SDL_Window *win = SDL_CreateWindow("DWA Manual + Auto Robot", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
    SDL_Renderer *r = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    Config cfg = {
        0.5f, 0.05f,               // max_speed, min_speed
        60.0f * PI / 180.0f,       // max_omega
        0.4f, 180.0f * PI / 180.0f, // max_accel, max_omega_accel
        0.02f, 1.0f * PI / 60.0f, // v_res, omega_res
        0.4f, 1.5f,                // dt, predict_time
        0.5f,                       // robot_radius
        0.3f, 0.6f, 0.5f           // to_goal_gain, speed_gain, obstacle_gain
    };

    RobotState state = {0, 0, 0, 0, 0};
    Obstacle obs[MAX_OBS];
    int obs_n;
    build_rectangle_room(obs, &obs_n);

    float goals[4][2] = {
        {ROOM_MIN_X + 2, ROOM_MIN_Y + 2},
        {ROOM_MAX_X - 2, ROOM_MIN_Y + 2},
        {ROOM_MAX_X - 2, ROOM_MAX_Y - 2},
        {ROOM_MIN_X + 2, ROOM_MAX_Y - 2}
    };

    int goal_id = 0;
    Uint32 start = SDL_GetTicks();
    int running = 1;

    while (running) {
        float gx = goals[goal_id][0];
        float gy = goals[goal_id][1];

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;

            // MANUAL CONTROL (arrow keys)
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_UP) state.v += 0.05f;
                if (e.key.keysym.sym == SDLK_DOWN) state.v -= 0.05f;
                if (e.key.keysym.sym == SDLK_LEFT) state.omega -= 0.05f;
                if (e.key.keysym.sym == SDLK_RIGHT) state.omega += 0.05f;
            }
        }

        if (dist(state.x, state.y, gx, gy) < 0.7f)
            goal_id = (goal_id + 1) % 4;

        float v, o;
        RobotState best_traj[MAX_TRAJ];
        int best_n;
        dwa_control(&state, &cfg, gx, gy, obs, obs_n, &v, &o, best_traj, &best_n);
        update_motion(&state, v, o, cfg.dt);

        SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
        SDL_RenderClear(r);

        // Draw predicted trajectory
        SDL_SetRenderDrawColor(r, 255, 0, 0, 255);
        for (int i = 0; i < best_n - 1; i++)
            draw_line(r, best_traj[i].x, best_traj[i].y, best_traj[i+1].x, best_traj[i+1].y);

        // Draw obstacles
        SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
        for (int i = 0; i < obs_n; i++) {
            int ox, oy;
            world_to_screen(obs[i].x, obs[i].y, &ox, &oy);
            draw_circle(r, ox, oy, obs[i].r * PPM);
        }

        // Draw robot
        int rx, ry;
        world_to_screen(state.x, state.y, &rx, &ry);
        SDL_SetRenderDrawColor(r, 0, 0, 255, 255);
        draw_circle(r, rx, ry, cfg.robot_radius * PPM);

        // Draw goal
        int gx_s, gy_s;
        world_to_screen(gx, gy, &gx_s, &gy_s);
        SDL_SetRenderDrawColor(r, 0, 255, 0, 255);
        draw_filled_circle(r, gx_s, gy_s, 6);

        SDL_RenderPresent(r);
        SDL_Delay(30);

        // End simulation after SIM_TIME
        if ((SDL_GetTicks() - start) / 1000.0f > SIM_TIME)
            break;
    }

    SDL_Quit();
    return 0;
}
