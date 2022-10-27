/*
#include <math.h>
#include <SDL2/SDL.h>
#include <stdint.h>
#include <time.h>

#include "common.h"

#define EPSILON 1e-6
#define DOT(u, v) ((u[0] * v[0]) + (u[1] * v[1]) + (u[2] * v[2]))
#define MAGNITUDE(v) sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2))

// Triangles in screen space, the visible area of which is (0, 0) (screen
// width, screen height). The Z-value is used only to populate the Z-buffer.
typedef struct {
    int16_t x, y;
    double z;
} screen_vertex;

#define NUM_SCREEN_TRIANGLES 10000
typedef struct {
    screen_vertex v[3];
    uint8_t r, g, b;
} screen_triangle_t;
screen_triangle_t screen_triangles[NUM_SCREEN_TRIANGLES];
int num_screen_triangles = 0;

// Any triangle can be decomposed into one or two triangles with a flat top or
// bottom (a "span"). A span is simple and fast to draw.
#define NUM_SPANS 20000
typedef struct {
    int16_t y_lo, y_hi;
    screen_vertex ref;
    double dx_dy_lo;
    double dx_dy_hi;
    double dz_dy_lo;
    double dz_dx_lo;
    screen_triangle_t *parent;
} span_t;
span_t spans[NUM_SPANS];
int num_spans;

int num_pointup_spans;
int num_pointdown_spans;
int num_degenerate_spans;
int num_onespan_triangles;
int num_twospan_triangles;
int num_degenerate_triangles;

void add_span(screen_vertex *a, screen_vertex *b, screen_vertex *c, screen_vertex *hi, screen_vertex *lo, screen_triangle_t *parent) {
    ASSERT((hi == NULL) != (lo == NULL));
    screen_vertex *hi_or_lo = (hi != NULL ? hi : lo);
    // Of the other two vertices, which is on the left, and which is on the
    // right?
    span_t *span = &spans[num_spans];
    screen_vertex *other1 = (a == hi_or_lo ? b : a);
    screen_vertex *other2 = (other1 == b ? c : (b == hi_or_lo ? c : b));
    screen_vertex *x_lo_vert, *x_hi_vert;
    if (other1->x < other2->x) {
        x_lo_vert = other1;
        x_hi_vert = other2;
    } else if (other2->x < other1->x) {
        x_lo_vert = other2;
        x_hi_vert = other1;
    } else {
        // We may receive degenerate spans, just make a note.
        num_degenerate_spans++;
        return;
    }
    if (hi != NULL) {
        num_pointdown_spans++;
        span->y_lo = other1->y; // or other2[1], doesn't matter.
        span->y_hi = hi->y;
    } else {
        num_pointup_spans++;
        span->y_lo = lo->y;
        span->y_hi = other1->y; // or other2[1], doesn't matter.
    }
    memcpy(&(span->ref), hi_or_lo, sizeof(span->ref));
    span->dx_dy_lo = (span->ref.x - x_lo_vert->x) / (double)(span->ref.y - x_lo_vert->y);
    span->dx_dy_hi = (span->ref.x - x_hi_vert->x) / (double)(span->ref.y - x_hi_vert->y);
    span->dz_dy_lo = (span->ref.z - x_lo_vert->z) / (double)(span->ref.y - x_lo_vert->y);
    span->dz_dx_lo = (x_hi_vert->z - x_lo_vert->z) / (double)(x_hi_vert->x - x_lo_vert->x);
    span->parent = parent;
    ASSERT(++num_spans < NUM_SPANS);
}

int ray_plane(double *plane_coord, double *plane_n, double *ray_coord, double *ray_v, double *intersection) {
    // See https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection,
    // "Algebraic form".
    double denom = DOT(ray_v, plane_n);
    if ((denom > -EPSILON) && (denom < EPSILON)) {
        // The ray and the plane are parallel.
        return 0;
    }
    double val[3] = {
        plane_coord[0] - ray_coord[0],
        plane_coord[1] - ray_coord[1],
        plane_coord[2] - ray_coord[2]
    };
    double d = DOT(val, plane_n) / denom;
    intersection[0] = ray_coord[0] + (ray_v[0] * d);
    intersection[1] = ray_coord[1] + (ray_v[1] * d);
    intersection[2] = ray_coord[2] + (ray_v[2] * d);
    return 1;
}

void rotate(double *in, double yaw, double pitch, double roll, double *out) {
    // 3x3 rotation matrix for yaw, pitch, and roll, yoinked from:
    // https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
    out[0] =
        (in[0] * cos(yaw) * cos(roll))
      + (in[1] * ((cos(roll) * sin(pitch) * sin(yaw)) - (sin(roll) * cos(pitch))))
      + (in[2] * ((cos(pitch) * sin(yaw) * cos(roll)) + (sin(pitch) * sin(roll))));
    out[1] =
        (in[0] * sin(roll) * cos(yaw))
      + (in[1] * ((sin(pitch) * sin(yaw) * sin(roll)) + (cos(pitch) * cos(roll))))
      + (in[2] * ((sin(yaw) * sin(roll) * cos(pitch)) - (cos(roll) * sin(pitch))));
    out[2] =
        (in[0] * -sin(yaw))
      + (in[1] * cos(yaw) * sin(pitch))
      + (in[2] * cos(pitch) * cos(yaw));
}

double old_t = 0;
double time_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double t = ts.tv_sec + (ts.tv_nsec / 1.0e9);
    ASSERT(t > old_t);
    old_t = t;
    return t;
}

#define MAX_VERTS 10000
double verts[MAX_VERTS][3];
int n_verts = 0;
#define MAX_TRIS 10000
int tris[MAX_TRIS][3];
int n_tris = 0;

void load_obj(char *fn, double scale) {
    FILE *fp = fopen(fn, "r");
    ASSERT(fp != NULL);
    char line[128];
    int n_verts_at_begin = n_verts;
    while (fgets(line, sizeof(line), fp) != NULL) {
        ASSERT(strlen(line) > 0);
        if (line[strlen(line) - 1] == '\n') {
            line[strlen(line) - 1] = '\0';
        }
        char *token = strtok(line, " ");
        char type = '\0';
        int off = 0;
        while (token != NULL) {
            switch (type) {
            case '\0':
                ASSERT(strlen(token) == 1);
                type = token[0];
                break;
            case 'v':
                ASSERT(off < 4);
                ASSERT(sscanf(token, "%lf", &verts[n_verts][off]) == 1);
                verts[n_verts][off] *= scale;
                off++;
                break;
            case 'f':
                ASSERT(off < 3);
                int i;
                ASSERT(sscanf(token, "%i", &i) == 1);
                tris[n_tris][off++] = i - 1 + n_verts_at_begin;
                break;
            // Intentionally no other cases: ignore other
            // directives/comments/etc.
            }
            token = strtok(NULL, " ");
        }
        switch (type) {
        case 'v':
            ASSERT(++n_verts < MAX_VERTS);
            break;
        case 'f':
            ASSERT(++n_tris < MAX_TRIS);
            break;
        }
    }
    ASSERT(feof(fp) != 0);
    ASSERT(ferror(fp) == 0);
}

int main(void) {
    load_obj("cow-nonormals.obj", 1);

    ASSERT(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) == 0);
    SDL_Window *sdl_window = SDL_CreateWindow("aspng", 0, 0, 100, 100, SDL_WINDOW_RESIZABLE);
    ASSERT(sdl_window != NULL);
    SDL_Renderer *sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_SOFTWARE);
    ASSERT(sdl_renderer != NULL);

    int did_print = 0;
    double *depth_buffer = NULL;
    int show_depth = 0;
    int free_flight = 0;

    double camera_pos[3] = {0, 0, -10};
    double camera_yaw = 0;
    double camera_pitch = 0;
    double camera_roll = 0;
    double since = time_now();
    int frames_since = 0;

    int do_quit = 0;
    while (!do_quit) {
        double now = time_now();
        if (now - since >= 1) {
            printf("fps: %f\n", frames_since / (now - since));
            since = now;
            frames_since = 0;
        }
        frames_since++;

        // The camera is a set of three orthogonal vectors forward, up, and
        // right.
        double forward[3] = {0, 0, 1};
        double up[3] = {0, 1, 0};
        double right[3] = {1, 0, 0};
        double camera_fwd[3];
        rotate(forward, camera_yaw, camera_pitch, camera_roll, camera_fwd);
        double camera_up[3];
        rotate(up, camera_yaw, camera_pitch, camera_roll, camera_up);
        double camera_right[3];
        rotate(right, camera_yaw, camera_pitch, camera_roll, camera_right);

        SDL_Surface *window_surface = SDL_GetWindowSurface(sdl_window);
        if (depth_buffer == NULL) {
            depth_buffer = malloc(sizeof(double) * window_surface->w * window_surface->h);
        }

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
            case SDL_KEYDOWN:
                switch (e.key.keysym.sym) {
                case SDLK_ESCAPE:
                    do_quit = 1;
                    break;
                case SDLK_d:
                    show_depth = 1 - show_depth;
                    break;
                case SDLK_f:
                    free_flight = 1 - free_flight;
                    break;
                }
                break;
            case SDL_WINDOWEVENT:
                switch (e.window.event) {
                case SDL_WINDOWEVENT_RESIZED:
                    free(depth_buffer);
                    window_surface = SDL_GetWindowSurface(sdl_window);
                    depth_buffer = malloc(sizeof(double) * window_surface->w * window_surface->h);
                    break;
                }
            }
            did_print = 0;
        }

        const uint8_t *keystate = SDL_GetKeyboardState(NULL);
        if (free_flight) {
            if (keystate[SDL_SCANCODE_LEFT])
                camera_yaw -= 0.01;
            if (keystate[SDL_SCANCODE_RIGHT])
                camera_yaw += 0.01;
            if (keystate[SDL_SCANCODE_UP])
                camera_pitch += 0.01;
            if (keystate[SDL_SCANCODE_DOWN])
                camera_pitch -= 0.01;
            if (keystate[SDL_SCANCODE_Z])
                camera_roll -= 0.01;
            if (keystate[SDL_SCANCODE_X])
                camera_roll += 0.01;
            if (keystate[SDL_SCANCODE_A]) {
                camera_pos[0] -= camera_fwd[0] * 0.1;
                camera_pos[1] -= camera_fwd[1] * 0.1;
                camera_pos[2] -= camera_fwd[2] * 0.1;
            }
            if (keystate[SDL_SCANCODE_S]) {
                camera_pos[0] += camera_fwd[0] * 0.1;
                camera_pos[1] += camera_fwd[1] * 0.1;
                camera_pos[2] += camera_fwd[2] * 0.1;
            }
        } else {
            // Rotate the camera around the origin.
            double now = time_now();
            double theta = 0.25 * now;
            camera_pos[0] = 10. * sin(theta);
            camera_pos[1] = 0;
            camera_pos[2] = 10. * cos(theta);

            // Point the camera at the origin.
            camera_yaw = M_PI - theta;
            camera_pitch = 0;
            camera_roll = M_PI;
        }

        // TODO clear the screen: shouldn't have to do this out of band
        for (int x = 0; x < window_surface->w; x++) {
            for (int y = 0; y < window_surface->h; y++) {
                uint32_t pixel = SDL_MapRGBA(window_surface->format, 0, 0, 0, 0xff);
                int off = (y * window_surface->w) + x;
                ((uint32_t *)window_surface->pixels)[off] = pixel;
                depth_buffer[off] = DBL_MAX;
            }
        }

        // The screen exists on a plane "in front of" the camera.
        double screen_center[3] = {
            camera_pos[0] + camera_fwd[0],
            camera_pos[1] + camera_fwd[1],
            camera_pos[2] + camera_fwd[2]
        };

        int n_inside_0_count = 0, n_inside_1_count = 0, n_inside_2_count = 0, n_inside_3_count = 0;

        num_screen_triangles = 0;
        for (int i = 0; i < n_tris; i++) {
            // Turn each arbitrary triangle into triangles suitable for
            // rendering, by clipping away the parts that cannot be correctly
            // rendered.
            double triangle[3][3];
            int n_inside = 0, inside[3],
                n_outside = 0, outside[3];
            for (int j = 0; j < 3; j++) {
                triangle[j][0] = verts[tris[i][j]][0];
                triangle[j][1] = verts[tris[i][j]][1];
                triangle[j][2] = verts[tris[i][j]][2];

                // Calculate signed distance to figure out which side of the
                // screen plane the vertex is on.
                double screen_to_v[3] = {
                    triangle[j][0] - screen_center[0],
                    triangle[j][1] - screen_center[1],
                    triangle[j][2] - screen_center[2],
                };
                double d = DOT(screen_to_v, camera_fwd);
                if (d < 0) {
                    inside[n_inside++] = j;
                    ASSERT(n_inside <= 3);
                } else {
                    outside[n_outside++] = j;
                    ASSERT(n_outside <= 3);
                }
            }

            int n_clipped_triangles = 0;
            double clipped_triangles[2][3][3];
            if (n_inside == 0) {
                // This triangle is fine as-is.
                n_clipped_triangles = 1;
                memcpy(clipped_triangles[0][0], triangle[0], sizeof(double[3]));
                memcpy(clipped_triangles[0][1], triangle[1], sizeof(double[3]));
                memcpy(clipped_triangles[0][2], triangle[2], sizeof(double[3]));
                n_inside_0_count++;
            } else if (n_inside == 1) {
                ASSERT(n_outside == 2);
                // If one point is inside (v3), then when we clip the triangle
                // it becomes a quadrilateral. The quadrilateral has two of the
                // original vertices, and an additional 2 vertices (named u1,
                // u2) at the point where (v3, v1) and (v3, v2) intersect the
                // screen plane.
                double v3[3];
                memcpy(v3, triangle[inside[0]], sizeof(double[3]));
                double v1[3];
                memcpy(v1, triangle[outside[0]], sizeof(double[3]));
                double v1_to_v3[3] = {v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]};
                double u1[3];
                // v1 and v3 are on opposite sides of the camera plane, so
                // v3-v1 and the camera plane cannot be parallel.
                ASSERT(ray_plane(screen_center, camera_fwd, v1, v1_to_v3, u1));
                double v2[3];
                memcpy(v2, triangle[outside[1]], sizeof(double[3]));
                double v2_to_v3[3] = {v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]};
                double u2[3];
                // Ditto, see above.
                ASSERT(ray_plane(screen_center, camera_fwd, v2, v2_to_v3, u2));
                // We can't render a quadrilateral, but we can split it into
                // two triangles. Since we know that the perimeter of the
                // quadrilateral is formed by visiting v1, v2, u2, u1 in that
                // order, we know that we can form two triangles (v1, v2, u2)
                // and (u2, u1, v1).
                n_clipped_triangles = 2;
                memcpy(clipped_triangles[0][0], v1, sizeof(double[3]));
                memcpy(clipped_triangles[0][1], v2, sizeof(double[3]));
                memcpy(clipped_triangles[0][2], u2, sizeof(double[3]));
                memcpy(clipped_triangles[1][0], u2, sizeof(double[3]));
                memcpy(clipped_triangles[1][1], u1, sizeof(double[3]));
                memcpy(clipped_triangles[1][2], v1, sizeof(double[3]));
                n_inside_1_count++;
            } else if (n_inside == 2) {
                ASSERT(n_outside == 1);
                // if two points inside (v1, v2), then make one triangle (v3, u1, u2)
                double v3[3];
                memcpy(v3, triangle[outside[0]], sizeof(double[3]));
                double v1[3];
                memcpy(v1, triangle[inside[0]], sizeof(double[3]));
                double v1_to_v3[3] = {v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2]};
                double u1[3];
                // v1 and v3 are on opposite sides of the camera plane, so
                // v3-v1 and the camera plane cannot be parallel.
                ASSERT(ray_plane(screen_center, camera_fwd, v1, v1_to_v3, u1));
                double v2[3];
                memcpy(v2, triangle[inside[1]], sizeof(double[3]));
                double v2_to_v3[3] = {v3[0] - v2[0], v3[1] - v2[1], v3[2] - v2[2]};
                double u2[3];
                // Ditto, see above.
                ASSERT(ray_plane(screen_center, camera_fwd, v2, v2_to_v3, u2));
                n_clipped_triangles = 1;
                memcpy(clipped_triangles[0][0], v3, sizeof(double[3]));
                memcpy(clipped_triangles[0][1], u1, sizeof(double[3]));
                memcpy(clipped_triangles[0][2], u2, sizeof(double[3]));
                n_inside_2_count++;
            } else if (n_inside == 3) {
                // Just don't render this triangle.
                ASSERT(n_outside == 0);
                n_inside_3_count++;
            } else {
                ASSERT(0);
            }
            ASSERT((n_clipped_triangles >= 0) && (n_clipped_triangles <= 2));

                // Anything on the same side of the screen plane is the camera is "inside the camera".
                //We can't render (partially or
                // completely) inside objects correctly, and we have to do something about it. Example:

                    /*
                              v1 . . . . . . . v2
                               .               .
                                .             .
                    _*___________.___________.____ camera plane
                                  .         .
                                   .       .
                           \cam/    .     .
                                     .   .
                                      . .
                                       v3

                    in this case, cam->v3 does intersect the plane, but the
                    intersection point is to the left (see *), which results in
                    an unintuitive and weird render, given that the other
                    vertices of the triangle will be projected onto the right of the screen.

                    however, not rendering any part of the triangle would also
                    look bad. to make this triangle renderable, we can remove the part of
                    the triangle that won't render properly, which in this case
                    results in a quadrilateral. We can split it into two
                    nonoverlapping triangles.

                              v1 . . . . . . . v2
                               .'  ,           .
                                .     ' .     .
                    _____________. . . . . .'.____ camera plane


                           \cam/

                    there are three cases in total for how to do this culling,
                    depending on how many vertices (in this case, 1) are
                    "inside".
                    */

                // side_of_plane(screen_center, camera_fwd, point) is negative
                // if point is between the camera and the screen plane
                // ("inside" the camera).

            // Project each clipped triangle into screen space.
            for (int j = 0; j < n_clipped_triangles; j++) {
            for (int k = 0; k < 3; k++) {
                double v[3];
                memcpy(v, clipped_triangles[j][k], sizeof(double[3]));

                // Project this vertex into screen space: draw a ray from the
                // vertex to the camera, intersecting with a plane (the
                // screen).
                double camera_pos_to_v[3] = {
                    v[0] - camera_pos[0],
                    v[1] - camera_pos[1],
                    v[2] - camera_pos[2],
                };
                double poi[3];
                if (ray_plane(screen_center, camera_fwd, camera_pos, camera_pos_to_v, poi)) {
                    // Find the intersection, relative to the plane center.
                    double poi_rel[3] = {
                        poi[0] - screen_center[0],
                        poi[1] - screen_center[1],
                        poi[2] - screen_center[2]
                    };
                    // Put this vertex into screenspace.
                    double half_screen_w = window_surface->w / 2,
                           half_screen_h = window_surface->h / 2;
                    screen_triangles[num_screen_triangles].v[k].x = half_screen_w + (DOT(poi_rel, camera_right) * half_screen_w);
                    screen_triangles[num_screen_triangles].v[k].y = half_screen_h + (DOT(poi_rel, camera_up) * half_screen_h);
                    // If the vertex is behind the camera, then the distance
                    // should be negative.  i dont think this is right TODO maybe remove this assert
                    double z = DOT(camera_pos_to_v, camera_fwd);
                    ASSERT(z >= 0);
                    screen_triangles[num_screen_triangles].v[k].z = z;

                    // For debugging purposes, give every triangle a different color.
                    screen_triangles[num_screen_triangles].r = (i * 13) % 0xff;
                    screen_triangles[num_screen_triangles].g = (i * 101) % 0xff;
                    screen_triangles[num_screen_triangles].b = (i * 211) % 0xff;
                } else {
                    // If the ray doesn't project onto the screen, it's because
                    // the ray is parallel to the screen, so it will be
                    // perpendicular to the screen normal.
                    ASSERT(0);
                }
            }
            ASSERT(++num_screen_triangles < NUM_SCREEN_TRIANGLES);
            }
        }

        // Turn each screen triangle into one or two spans.
        num_spans = 0;
        num_pointup_spans = 0;
        num_pointdown_spans = 0;
        num_degenerate_spans = 0;
        num_onespan_triangles = 0;
        num_twospan_triangles = 0;
        num_degenerate_triangles = 0;
        for (int i = 0; i < num_screen_triangles; i++) {
            screen_vertex *a = &screen_triangles[i].v[0];
            screen_vertex *b = &screen_triangles[i].v[1];
            screen_vertex *c = &screen_triangles[i].v[2];
            // Look for (one) top ("hi") vertex.
            screen_vertex *hi = NULL;
            if ((a->y > b->y) && (a->y > c->y)) {
                hi = a;
            } else if ((b->y > a->y) && (b->y > c->y)) {
                hi = b;
            } else if ((c->y > a->y) && (c->y > b->y)) {
                hi = c;
            }
            // Look for (one) bottom ("lo") vertex.
            screen_vertex *lo = NULL;
            if ((a->y < b->y) && (a->y < c->y)) {
                lo = a;
            } else if ((b->y < a->y) && (b->y < c->y)) {
                lo = b;
            } else if ((c->y < a->y) && (c->y < b->y)) {
                lo = c;
            }
            // If there's neither a hi nor lo vertex, then it's
            // degenerate.
            if ((hi == NULL) && (lo == NULL)) {
                num_degenerate_triangles++;
                continue;
            }
            // If there's only a hi or lo vertex, then there is only one
            // span.
            if ((hi == NULL) != (lo == NULL)) {
                if (hi != NULL) {
                    add_span(a, b, c, hi, NULL, &screen_triangles[i]);
                } else if (lo != NULL) {
                    add_span(a, b, c, NULL, lo, &screen_triangles[i]);
                } else ASSERT(0);
                num_onespan_triangles++;
            }
            // If there is both a hi and lo vertex, then we need to draw
            // two spans.
            if ((hi != NULL) && (lo != NULL)) {
                // First, we need to find the vertex ('mid') which isn't the hi
                // or lo vertex.
                screen_vertex *mid = NULL;
                if ((a != hi) && (a != lo)) mid = a;
                else if ((b != hi) && (b != lo)) mid = b;
                else mid = c;
                ASSERT(mid != NULL);
                // Find the point on the edge linking hi and lo which is at the
                // same y-coordinate as 'mid'.
                screen_vertex split = {
                    .x = lo->x + (((hi->x - lo->x) / (double)(hi->y - lo->y)) * (mid->y - lo->y)),
                    .y = mid->y,
                    .z = lo->z + (((hi->z - lo->z) / (double)(hi->y - lo->y)) * (mid->y - lo->y))
                };
                // Create two spans!
                add_span(hi, mid, &split, hi, NULL, &screen_triangles[i]);
                add_span(lo, mid, &split, NULL, lo, &screen_triangles[i]);
                num_twospan_triangles++;
            }
        }

        if (!did_print) {
            did_print = 1;
            printf("******\n");
            printf("yaw = %f, pitch = %f, roll = %f\n", camera_yaw, camera_pitch, camera_roll);
            printf("camera: %f, %f, %f\n", camera_pos[0], camera_pos[1], camera_pos[2]);
            printf("forward: %f, %f, %f .. %f\n", camera_fwd[0], camera_fwd[1], camera_fwd[2], MAGNITUDE(camera_fwd));
            printf("up: %f, %f, %f .. %f\n", camera_up[0], camera_up[1], camera_up[2], MAGNITUDE(camera_up));
            printf("right: %f, %f, %f .. %f\n", camera_right[0], camera_right[1], camera_right[2], MAGNITUDE(camera_right));
            printf("0 = %i, 1 = %i, 2 = %i, 3 = %i\n", n_inside_0_count, n_inside_1_count, n_inside_2_count, n_inside_3_count);
            printf("%i screen triangles (%i onespans, %i twospans, %i degenerates)\n",
                num_screen_triangles, num_onespan_triangles, num_twospan_triangles, num_degenerate_triangles);
            printf("%i spans (%i pointups, %i pointdowns, %i degenerates)\n",
                num_spans, num_pointup_spans, num_pointdown_spans, num_degenerate_spans);
        }

        // Draw all spans to the screen, respecting the z-buffer.
        double min_z = DBL_MAX;
        double max_z = -DBL_MAX;
        for (int y = 0; y < window_surface->h; y++) {
            for (int i = 0; i < num_spans; i++) {
                span_t *span = &spans[i];
                if ((span->y_lo <= y) && (y <= span->y_hi)) {
                    int16_t x_fill_lo = span->ref.x + (span->dx_dy_lo * (y - span->ref.y));
                    int16_t x_fill_hi = span->ref.x + (span->dx_dy_hi * (y - span->ref.y));
                    ASSERT(x_fill_lo <= x_fill_hi);
                    if (x_fill_lo < 0)
                        x_fill_lo = 0;
                    if (x_fill_hi > window_surface->w - 1)
                        x_fill_hi = window_surface->w - 1;
                    double z_lo = span->ref.z + (span->dz_dy_lo * (y - span->ref.y));
                    for (int16_t x = x_fill_lo; x <= x_fill_hi; x++) {
                        // Do a z-check before we draw the pixel.
                        int off = (y * window_surface->w) + x;
                        double z = z_lo + (span->dz_dx_lo * (x - x_fill_lo));
                        if ((z < depth_buffer[off]) && (z >= 0)) {
                            depth_buffer[off] = z;
                            if (z > max_z) {
                                max_z = z;
                            }
                            if (z < min_z) {
                                min_z = z;
                            }
                            uint32_t pixel = SDL_MapRGBA(
                                window_surface->format,
                                span->parent->r,
                                span->parent->g,
                                span->parent->b,
                                0xff
                            );
                            ((uint32_t *)window_surface->pixels)[off] = pixel;
                        }
                    }
                }
            }
        }

        if (show_depth) {
            for (int y = 0; y < window_surface->h; y++) {
                for (int x = 0; x < window_surface->w; x++) {
                    double z = depth_buffer[(y * window_surface->w) + x];
                    double ramped = 255 * (1 - ((z - min_z) / (max_z - min_z)));
                    uint32_t pixel = SDL_MapRGBA(window_surface->format, 0, ramped, 0, 0xff);
                    ((uint32_t *)window_surface->pixels)[(y * window_surface->w) + x] = pixel;
                }
            }
        }

        SDL_UpdateWindowSurface(sdl_window);
    }
}
*/

fn main() {
    println!("Hello, world!");
}
