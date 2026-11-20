#include "micromouse.h"

#define MAZE_CELL_COUNT (MICROMOUSE_MAZE_WIDTH * MICROMOUSE_MAZE_HEIGHT)
#define DISTANCE_UNREACHABLE 0xFFFFU

static uint8_t walls[MICROMOUSE_MAZE_HEIGHT][MICROMOUSE_MAZE_WIDTH];
static uint8_t goal_x;
static uint8_t goal_y;
static bool initialized;

static const int8_t delta_x[4] = { 0, 1, 0, -1 };
static const int8_t delta_y[4] = { -1, 0, 1, 0 };

static bool in_bounds(int16_t x, int16_t y)
{
    return x >= 0 && x < (int16_t)MICROMOUSE_MAZE_WIDTH &&
           y >= 0 && y < (int16_t)MICROMOUSE_MAZE_HEIGHT;
}

static uint8_t opposite(uint8_t direction)
{
    return (uint8_t)((direction + 2U) % 4U);
}

static uint16_t cell_index(uint8_t x, uint8_t y)
{
    return (uint16_t)y * MICROMOUSE_MAZE_WIDTH + x;
}

bool micromouse_init(uint8_t x, uint8_t y)
{
    if (x >= MICROMOUSE_MAZE_WIDTH || y >= MICROMOUSE_MAZE_HEIGHT) {
        return false;
    }

    for (uint8_t row = 0; row < MICROMOUSE_MAZE_HEIGHT; row++) {
        for (uint8_t column = 0; column < MICROMOUSE_MAZE_WIDTH; column++) {
            walls[row][column] = 0;
            if (row == 0) walls[row][column] |= MICROMOUSE_WALL_NORTH;
            if (column == MICROMOUSE_MAZE_WIDTH - 1U)
                walls[row][column] |= MICROMOUSE_WALL_EAST;
            if (row == MICROMOUSE_MAZE_HEIGHT - 1U)
                walls[row][column] |= MICROMOUSE_WALL_SOUTH;
            if (column == 0) walls[row][column] |= MICROMOUSE_WALL_WEST;
        }
    }

    goal_x = x;
    goal_y = y;
    initialized = true;
    return true;
}

bool micromouse_set_walls(uint8_t x, uint8_t y, uint8_t wall_mask)
{
    if (!initialized || x >= MICROMOUSE_MAZE_WIDTH ||
        y >= MICROMOUSE_MAZE_HEIGHT || (wall_mask & 0xF0U)) {
        return false;
    }

    for (uint8_t direction = 0; direction < 4; direction++) {
        int16_t neighbor_x = (int16_t)x + delta_x[direction];
        int16_t neighbor_y = (int16_t)y + delta_y[direction];
        uint8_t bit = (uint8_t)(1U << direction);
        bool has_wall = (wall_mask & bit) != 0;

        if (!in_bounds(neighbor_x, neighbor_y)) {
            walls[y][x] |= bit;
            continue;
        }

        if (has_wall) {
            walls[y][x] |= bit;
            walls[neighbor_y][neighbor_x] |= (uint8_t)(1U << opposite(direction));
        } else {
            walls[y][x] &= (uint8_t)~bit;
            walls[neighbor_y][neighbor_x] &=
                (uint8_t)~(1U << opposite(direction));
        }
    }
    return true;
}

static void calculate_distances(
    uint16_t distances[MICROMOUSE_MAZE_HEIGHT][MICROMOUSE_MAZE_WIDTH])
{
    uint16_t queue[MAZE_CELL_COUNT];
    uint16_t read_index = 0;
    uint16_t write_index = 0;

    for (uint8_t y = 0; y < MICROMOUSE_MAZE_HEIGHT; y++) {
        for (uint8_t x = 0; x < MICROMOUSE_MAZE_WIDTH; x++) {
            distances[y][x] = DISTANCE_UNREACHABLE;
        }
    }

    distances[goal_y][goal_x] = 0;
    queue[write_index++] = cell_index(goal_x, goal_y);

    while (read_index < write_index) {
        uint16_t current = queue[read_index++];
        uint8_t x = (uint8_t)(current % MICROMOUSE_MAZE_WIDTH);
        uint8_t y = (uint8_t)(current / MICROMOUSE_MAZE_WIDTH);

        for (uint8_t direction = 0; direction < 4; direction++) {
            int16_t next_x = (int16_t)x + delta_x[direction];
            int16_t next_y = (int16_t)y + delta_y[direction];
            if ((walls[y][x] & (1U << direction)) ||
                !in_bounds(next_x, next_y) ||
                distances[next_y][next_x] != DISTANCE_UNREACHABLE) {
                continue;
            }

            distances[next_y][next_x] = (uint16_t)(distances[y][x] + 1U);
            queue[write_index++] = cell_index((uint8_t)next_x, (uint8_t)next_y);
        }
    }
}

bool micromouse_plan_next(uint8_t x, uint8_t y,
                          micromouse_direction_t *direction)
{
    uint16_t distances[MICROMOUSE_MAZE_HEIGHT][MICROMOUSE_MAZE_WIDTH];
    if (!initialized || direction == 0 ||
        x >= MICROMOUSE_MAZE_WIDTH || y >= MICROMOUSE_MAZE_HEIGHT) {
        return false;
    }

    calculate_distances(distances);
    uint16_t best_distance = distances[y][x];
    bool found = false;

    for (uint8_t candidate = 0; candidate < 4; candidate++) {
        int16_t next_x = (int16_t)x + delta_x[candidate];
        int16_t next_y = (int16_t)y + delta_y[candidate];
        if ((walls[y][x] & (1U << candidate)) || !in_bounds(next_x, next_y)) {
            continue;
        }

        if (distances[next_y][next_x] < best_distance) {
            best_distance = distances[next_y][next_x];
            *direction = (micromouse_direction_t)candidate;
            found = true;
        }
    }
    return found;
}
