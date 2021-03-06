/*
 * g.c
 * Playing with forces
 * @ibykow, 2015
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <sys/time.h>

#include "debug.h"

#define INIT_WIDTH 1800
#define INIT_HEIGHT 920

#define USE_DOUBLE_BUFFER 1

#define THRESHOLD_FACTOR 1
#define G_CONST 0.003
#define NUM_PARTICLES 400
#define MAX_COLLIDED (((NUM_PARTICLES * NUM_PARTICLES) >> 1) + NUM_PARTICLES)
#define PRIMARY_MASS 500000
#define PRIMARY_RADIUS 12
#define SECONDARY_MASS 5000

#define MIN_R 3
#define MAX_R 8
#define MIN_V 10
#define MAX_V 20

// #define ELASTIC_COLLISIONS

#ifndef PI
#define PI 3.141592654
#endif
#define PI2 (PI*2)
#define NUM_SIN_ANGLES 6000
#define CIRCLE_TRIG_COUNT 20

#define TEXT_BUF_SIZE 256

#define DIRECTIONS 4

enum keys_e {
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT
};

static const char direction_key_chars[DIRECTIONS] = "wsad";
static int direction_keys_pressed[DIRECTIONS] = {0, 0, 0, 0};

struct color_s {
    float r, g, b;
};

struct particle_s {
    struct game_s *g;
    struct color_s c;
    float x, y, r, m, vx, vy, fx, fy;
};

struct game_s {
    GLint w, h;
    int split;
    const float min_r, max_r, G;
    struct particle_s particles[NUM_PARTICLES], *collided[MAX_COLLIDED][2];
    size_t num_collided;
} game = {
    INIT_WIDTH, INIT_HEIGHT, 0, MIN_R, MAX_R, G_CONST, 0, 0, 0
}, *g = &game;

static float sin_table[NUM_SIN_ANGLES];
static int last_ch_code = 0;
static int left_button_down = 0;

static float _sin(float angle)
{
    return (sin_table[(GLint) angle]);
}

static float _cos(float angle)
{
    return (sin_table[((GLint) angle + (NUM_SIN_ANGLES / 4)) % NUM_SIN_ANGLES]);
}

static void maru(GLfloat x, GLfloat y, GLfloat radius)
{
	int i;

	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(x, y); // center of circle

	for(i = 0; i <= CIRCLE_TRIG_COUNT; i++)
		glVertex2f(x + (radius * _cos(i * NUM_SIN_ANGLES / CIRCLE_TRIG_COUNT)),
            y + (radius * _sin(i * NUM_SIN_ANGLES / CIRCLE_TRIG_COUNT)));

	glEnd();
}

void say(float x, float y, void *font, char *str, float r, float g, float b)
{
    glColor3f(r, g, b);

    glRasterPos2f(x, y);

    while (*str)
        glutBitmapCharacter(font, *str++);


}

static int rand_int(int min, int max)
{
    if (max <= min)
        return min;

    int diff = max - min, mod = RAND_MAX % diff, lim = (RAND_MAX / diff) * diff, ret = 0;

    if (mod) {
        while ((ret = rand()) >= lim)
            ;
    }

    return (ret % diff) + min;
}

static void color_init_rnd(struct color_s *c)
{
    c->r = (float) 1.0 - (rand() * 2.0 / (float) RAND_MAX / 3);
    c->g = (float) 1.0 - (rand() * 2.0 / (float) RAND_MAX / 3);
    c->b = (float) 1.0 - (rand() * 2.0 / (float) RAND_MAX / 3);
}

static void particle_init_rnd(struct particle_s *o, struct game_s *g)
{
    o->g = g;
    o->x = (float) rand_int(0, g->w);
    o->y = (float) rand_int(0, g->h);
    o->vx = (float) rand_int(MIN_V, MAX_V) * (o->y < (g->h >> 1) ? -1 : 1);
    o->vy = (float) 0;
    o->r = (float) rand_int(g->min_r, g->max_r);
    o->m = o->r;
    o->fx = 0;
    o->fy = 0;

    color_init_rnd(&o->c);

    INFO("x: %f, y: %f, r: %f", o->x, o->y, o->r);
}

static void particle_draw(struct particle_s *o)
{
    glColor3f(o->c.r, o->c.g, o->c.b);
    maru(o->x, o->y, o->r);
}

static void particle_update_forces(struct particle_s *a, struct particle_s *b)
{
    float dx = a->x - b->x, dy = a->y - b->y, dxdx = dx * dx,
        dydy = dy * dy, rr = dxdx + dydy,
        r = sqrt(fabs(rr)), mG = g->G / rr, bound = a->r + b->r,
        a_rate = b->m / a->m, b_rate = 1 / a_rate;

    if (r < bound) {
        g->collided[g->num_collided][0] = a;
        g->collided[g->num_collided][1] = b;
        g->num_collided++;
    } else {
        float fx = mG * dx, fy = mG * dy;

        a->fx -= fx * a_rate;
        b->fx += fx * b_rate;
        a->fy -= fy * a_rate;
        b->fy += fy * b_rate;
    }
}

static void particle_collide(struct particle_s *a, struct particle_s *b)
{
    float m = a->m + b->m;

    #ifdef ELASTIC_COLLISIONS
    float   d = a->m - b->m,
            avx = d * a->vx / m + b->m * b->m * b->vx / m,
            bvx = a->m * a->m * a->vx / m - d * b->vx / m,
            avy = d * a->vy / m + b->m * b->m * b->vy / m,
            bvy = a->m * a->m * a->vy / m - d * b->vy / m;

    a->vx = avx;
    b->vx = bvx;
    a->vy = avy;
    b->vy = bvy;
    #else
    a->vx = b->vx = (a->m * a->vx + b->m * b->vx) / m;
    a->vy = b->vy = (a->m * a->vy + b->m * b->vy) / m;
    #endif
}

static void particle_update_pos(struct particle_s *a)
{
    a->vx += a->fx;
    a->fx = 0;
    a->x += a->vx;

    a->vy += a->fy;
    a->fy = 0;
    a->y += a->vy;

    if (a->x < 0) {
        a->x = a->g->w - 100;
        a->vx /= THRESHOLD_FACTOR;
    } else if (a->x > a->g->w) {
        a->x = (float) ((int) a->x % (int) a->g->w);
        a->vx /= THRESHOLD_FACTOR;
    }

    if (a->y < 0) {
        a->y = a->g->h + a->y;
        a->vy /= THRESHOLD_FACTOR;
    } else if ((a->y > a->g->h)) {
        a->y = (float) ((int) a->y % (int) a->g->h);
        a->vy /= THRESHOLD_FACTOR;
    }

}

static void game_update(struct game_s *g)
{
    size_t i, j;

    g->num_collided = 0;

    for (i = 0; i < NUM_PARTICLES; i++)
        for (j = i + 1; j < NUM_PARTICLES; j++)
            particle_update_forces(g->particles + i, g->particles + j);

    if (!g->split)
        for (i = 0; i < g->num_collided; i++)
            particle_collide(g->collided[i][0], g->collided[i][1]);

    if (direction_keys_pressed[KEY_UP])
        g->particles[0].vy += 0.1;

    if (direction_keys_pressed[KEY_DOWN])
        g->particles[0].vy -= 0.1;

    if (direction_keys_pressed[KEY_LEFT])
        g->particles[0].vx -= 0.1;

    if (direction_keys_pressed[KEY_RIGHT])
        g->particles[0].vx += 0.1;


    for (i = 0; i < NUM_PARTICLES; i++)
        particle_update_pos(g->particles + i);
}

static void game_draw(struct game_s *g)
{
    size_t i;
    char text_buffer[TEXT_BUF_SIZE];

    glClear(GL_COLOR_BUFFER_BIT);

    if (snprintf(text_buffer, TEXT_BUF_SIZE, "collisions: %lu",
        g->num_collided) >= TEXT_BUF_SIZE)
        text_buffer[TEXT_BUF_SIZE - 1] = '\0';

    say(10, 25, GLUT_BITMAP_HELVETICA_12, text_buffer, 0.1, 1.0, 0.1);

    if (snprintf(text_buffer, TEXT_BUF_SIZE, "x: %0.2f y: %0.2f",
        g->particles[0].x, g->particles[0].y) >= TEXT_BUF_SIZE)
        text_buffer[TEXT_BUF_SIZE - 1] = '\0';

    say(10, 10, GLUT_BITMAP_HELVETICA_12, text_buffer, 0.1, 1.0, 0.1);

    say(g->w - 100, 10, GLUT_BITMAP_HELVETICA_12, "Ilia Bykow, 2015", 1.0, 1.0, 1.0);

    for (i = 0; i < NUM_PARTICLES; i++)
        particle_draw(g->particles + i);

}

static void reshape_cb(int width, int height)
{
    g->w = width;
    g->h = height;

    glViewport(0, 0, g->w, g->h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-0.5, g->w + 0.5, -0.5, g->h + 0.5);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
}

static void idle_cb(void)
{
    game_update(g);
    glutPostRedisplay();
}

static void disp_cb(void)
{
    game_draw(g);

    #ifdef USE_DOUBLE_BUFFER
        glutSwapBuffers();
    #else
        glFlush();
    #endif
}

static void vis_cb(int state)
{
    glutIdleFunc(state == GLUT_VISIBLE ? idle_cb : NULL);
}

static void game_init_particles(struct game_s *g)
{
    size_t i;

    g->particles[0] = (struct particle_s) {
        .g = g, .c = { .r = 1.0, .g = 1.0, .b = 1.0 },
        .x = g->w >> 1, .y = g->h >> 1, .r = PRIMARY_RADIUS,
        .m = PRIMARY_MASS, .vx = 0, .vy = 0, .fx = 0, .fy = 0
    };

    // g->particles[1] = (struct particle_s) {
    //     .g = g, .c = { .r = 0.0, .g = 0.0, .b = 1.0 },
    //     .x = g->w - 50, .y = 50 , .r = 4,
    //     .m = SECONDARY_MASS, .vx = 200, .vy = 0, .fx = 0, .fy = 0
    // };

    for (i = 1; i < NUM_PARTICLES; i++)
        particle_init_rnd(g->particles + i, g);

    g->num_collided = 0;
}

static void sin_table_init()
{
    size_t i;
    float angle = 0;
    for (i = 0; i < NUM_SIN_ANGLES; i++) {
        sin_table[i] = sin(angle);
        angle += PI / (NUM_SIN_ANGLES / 2.0);
    }
}

static void handle_args(int argc, char **argv)
{
    return;
}

static void init(int argc, char **argv)
{
    float angle;
    GLint n;

    srand((unsigned int) time(NULL));

    sin_table_init();

    game_init_particles(g);

    glutInitWindowSize(g->w, g->h);
    glutInit(&argc, argv);
    handle_args(argc, argv);

    glutInitDisplayMode(GLUT_RGB |
        (USE_DOUBLE_BUFFER ? GLUT_DOUBLE : GLUT_SINGLE));

    glutCreateWindow("Particles");

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glDisable(GL_DITHER);
}

static void keyup_cb(unsigned char key, int x, int y)
{
    if (key == direction_key_chars[KEY_UP])
        direction_keys_pressed[KEY_UP] = 0;

    if (key == direction_key_chars[KEY_DOWN])
        direction_keys_pressed[KEY_DOWN] = 0;

    if (key == direction_key_chars[KEY_LEFT])
        direction_keys_pressed[KEY_LEFT] = 0;

    if (key == direction_key_chars[KEY_RIGHT])
        direction_keys_pressed[KEY_RIGHT] = 0;

}

static void keybrd_cb(unsigned char key, int x, int y)
{
    if (key == direction_key_chars[KEY_UP])
        direction_keys_pressed[KEY_UP] = 1;

    if (key == direction_key_chars[KEY_DOWN])
        direction_keys_pressed[KEY_DOWN] = 1;

    if (key == direction_key_chars[KEY_LEFT])
        direction_keys_pressed[KEY_LEFT] = 1;

    if (key == direction_key_chars[KEY_RIGHT])
        direction_keys_pressed[KEY_RIGHT] = 1;

    switch (key) {
    case 't':
        g->particles[0].m *= -1;
        break;
    case 'r':
        game_init_particles(g);
        break;
    case 'y':
        g->split ^= 1;
        break;
    case 27:
        INFO("Quitting")
        exit(0);
    }

    last_ch_code = (int) key;
}

static void mouse_move_cb(int x, int y)
{
    if (!left_button_down)
        return;

    g->particles[0].x = (float) x;
    g->particles[0].y = (float) g->h - y;

    g->particles[0].vx = 0;
    g->particles[0].vy = 0;
}

static void mouse_click_cb(int b, int s, int x, int y)
{
    if (b == GLUT_LEFT_BUTTON) {
        left_button_down = (s == GLUT_DOWN);
        mouse_move_cb(x, y);
    }
}

int main(int argc, char **argv)
{
    INFO("Initializing");
    init(argc, argv);

    INFO("Setting callbacks");
    glutReshapeFunc(reshape_cb);
    glutKeyboardFunc(keybrd_cb);
    glutKeyboardUpFunc(keyup_cb);
    glutMouseFunc(mouse_click_cb);
    glutMotionFunc(mouse_move_cb);
    glutVisibilityFunc(vis_cb);
    glutDisplayFunc(disp_cb);

    INFO("Entering full screen");
    glutFullScreen();

    INFO("Entering main loop");
    glutMainLoop();
    return 0;                         /* ANSI C requires main to return int. */
}
