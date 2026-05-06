#include <errno.h>
#include <float.h>
#include <limits.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef struct Drone {
    int id;
    float x;
    float y;
    float z;
} Drone;

typedef struct PairResult {
    const Drone *a;
    const Drone *b;
    double dist_sq;
} PairResult;

static const size_t MIN_DRONE_COUNT = 2U;
static const size_t MAX_STRIP_NEIGHBORS = 15U;
static const size_t BRUTE_FORCE_THRESHOLD = 3U;

static double distance_squared(const Drone *first, const Drone *second) {
    double dx = (double)first->x - (double)second->x;
    double dy = (double)first->y - (double)second->y;
    double dz = (double)first->z - (double)second->z;
    return dx * dx + dy * dy + dz * dz;
}

static int compare_drone_x(const void *lhs, const void *rhs) {
    const Drone *left = (const Drone *)lhs;
    const Drone *right = (const Drone *)rhs;

    if (left->x < right->x) {
        return -1;
    }
    if (left->x > right->x) {
        return 1;
    }
    if (left->y < right->y) {
        return -1;
    }
    if (left->y > right->y) {
        return 1;
    }
    if (left->z < right->z) {
        return -1;
    }
    if (left->z > right->z) {
        return 1;
    }
    return left->id - right->id;
}

static int compare_drone_ptr_y(const void *lhs, const void *rhs) {
    const Drone *const *left = (const Drone *const *)lhs;
    const Drone *const *right = (const Drone *const *)rhs;
    const Drone *a = *left;
    const Drone *b = *right;

    if (a->y < b->y) {
        return -1;
    }
    if (a->y > b->y) {
        return 1;
    }
    if (a->z < b->z) {
        return -1;
    }
    if (a->z > b->z) {
        return 1;
    }
    return a->id - b->id;
}

static PairResult brute_force_closest(const Drone *start, size_t count) {
    PairResult best;
    const Drone *outer = NULL;

    best.a = NULL;
    best.b = NULL;
    best.dist_sq = DBL_MAX;

    for (outer = start; outer < start + count; ++outer) {
        const Drone *inner = outer + 1;
        for (; inner < start + count; ++inner) {
            double current = distance_squared(outer, inner);
            if (current < best.dist_sq) {
                best.dist_sq = current;
                best.a = outer;
                best.b = inner;
            }
        }
    }

    return best;
}

static PairResult best_of(PairResult left, PairResult right) {
    if (left.dist_sq <= right.dist_sq) {
        return left;
    }
    return right;
}

static PairResult closest_pair_recursive(const Drone *start, size_t count) {
    PairResult best;
    size_t middle_index;
    const Drone *middle;
    const Drone *scan;
    const Drone **strip;
    const Drone **strip_write;
    const Drone **strip_begin;
    const Drone **strip_end;
    size_t strip_size;

    if (count <= BRUTE_FORCE_THRESHOLD) {
        return brute_force_closest(start, count);
    }

    middle_index = count / 2U;
    middle = start + middle_index;

    best = best_of(
        closest_pair_recursive(start, middle_index),
        closest_pair_recursive(middle, count - middle_index)
    );

    strip = (const Drone **)malloc(count * sizeof(const Drone *));
    if (strip == NULL) {
        fprintf(stderr, "Memory allocation failure while building strip.\n");
        exit(EXIT_FAILURE);
    }

    strip_write = strip;
    for (scan = start; scan < start + count; ++scan) {
        double dx = (double)scan->x - (double)middle->x;
        if (dx * dx < best.dist_sq) {
            *strip_write = scan;
            ++strip_write;
        }
    }

    strip_begin = strip;
    strip_end = strip_write;
    strip_size = (size_t)(strip_end - strip_begin);

    qsort(strip_begin, strip_size, sizeof(const Drone *), compare_drone_ptr_y);

    for (; strip_begin < strip_end; ++strip_begin) {
        const Drone **neighbor = strip_begin + 1;
        size_t neighbor_count = 0U;

        for (; neighbor < strip_end && neighbor_count < MAX_STRIP_NEIGHBORS; ++neighbor, ++neighbor_count) {
            double candidate;
            double dy = (double)(*neighbor)->y - (double)(*strip_begin)->y;
            if (dy * dy >= best.dist_sq) {
                break;
            }

            candidate = distance_squared(*strip_begin, *neighbor);
            if (candidate < best.dist_sq) {
                best.dist_sq = candidate;
                best.a = *strip_begin;
                best.b = *neighbor;
            }
        }
    }

    free(strip);
    return best;
}

static float random_coordinate(float min, float max) {
    /* Pseudo-random generation is used for simulation/demo input only. */
    double ratio = (double)rand() / (double)RAND_MAX;
    return (float)(min + ratio * (double)(max - min));
}

static Drone *generate_swarm(size_t count) {
    Drone *swarm = (Drone *)malloc(count * sizeof(Drone));
    Drone *cursor;
    size_t id;

    if (swarm == NULL) {
        return NULL;
    }

    cursor = swarm;
    id = 0U;
    for (; id < count; ++id, ++cursor) {
        cursor->id = (int)id;
        cursor->x = random_coordinate(-10000.0f, 10000.0f);
        cursor->y = random_coordinate(-10000.0f, 10000.0f);
        cursor->z = random_coordinate(-10000.0f, 10000.0f);
    }

    return swarm;
}

static size_t parse_count(const char *text) {
    char *end = NULL;
    unsigned long value;

    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno != 0) {
        fprintf(stderr, "Invalid drone count '%s': conversion overflow/underflow.\n", text);
        exit(EXIT_FAILURE);
    }
    if (end == text) {
        fprintf(stderr, "Invalid drone count '%s': no digits were found.\n", text);
        exit(EXIT_FAILURE);
    }
    if (*end != '\0') {
        fprintf(stderr, "Invalid drone count '%s': unexpected trailing characters.\n", text);
        exit(EXIT_FAILURE);
    }
    if (value < MIN_DRONE_COUNT) {
        fprintf(stderr, "Invalid drone count '%s': expected an integer >= %zu.\n", text, MIN_DRONE_COUNT);
        exit(EXIT_FAILURE);
    }
#if ULONG_MAX > SIZE_MAX
    if (value > (unsigned long)SIZE_MAX) {
        fprintf(stderr, "Invalid drone count '%s': value is too large for this platform.\n", text);
        exit(EXIT_FAILURE);
    }
#endif

    return (size_t)value;
}

int main(int argc, char **argv) {
    size_t drone_count = 10000U;
    Drone *swarm;
    PairResult closest;

    if (argc >= 2) {
        drone_count = parse_count(*(argv + 1));
    }

    /* rand() is used only to generate demo input data for local simulation. */
    srand((unsigned int)time(NULL));

    swarm = generate_swarm(drone_count);
    if (swarm == NULL) {
        fprintf(stderr, "Unable to allocate memory for %zu drones.\n", drone_count);
        return EXIT_FAILURE;
    }

    qsort(swarm, drone_count, sizeof(Drone), compare_drone_x);
    closest = closest_pair_recursive(swarm, drone_count);

    if (closest.a == NULL || closest.b == NULL) {
        fprintf(stderr, "Unable to compute closest pair.\n");
        free(swarm);
        return EXIT_FAILURE;
    }

    printf("Closest pair among %zu drones:\n", drone_count);
    printf("  Drone %d at (%f, %f, %f)\n", closest.a->id, closest.a->x, closest.a->y, closest.a->z);
    printf("  Drone %d at (%f, %f, %f)\n", closest.b->id, closest.b->x, closest.b->y, closest.b->z);
    printf("  Distance = %.10f\n", sqrt(closest.dist_sq));

    free(swarm);
    return EXIT_SUCCESS;
}
