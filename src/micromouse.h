/* Hardwareunabhaengige Maze-Karte und Flood-Fill-Routenplanung. */
#ifndef MICROMOUSE_H
#define MICROMOUSE_H

#include <stdbool.h>
#include <stdint.h>

#define MICROMOUSE_MAZE_WIDTH  16U
#define MICROMOUSE_MAZE_HEIGHT 16U

typedef enum {
    MICROMOUSE_NORTH = 0,
    MICROMOUSE_EAST,
    MICROMOUSE_SOUTH,
    MICROMOUSE_WEST
} micromouse_direction_t;

#define MICROMOUSE_WALL_NORTH (1U << MICROMOUSE_NORTH)
#define MICROMOUSE_WALL_EAST  (1U << MICROMOUSE_EAST)
#define MICROMOUSE_WALL_SOUTH (1U << MICROMOUSE_SOUTH)
#define MICROMOUSE_WALL_WEST  (1U << MICROMOUSE_WEST)

bool micromouse_init(uint8_t goal_x, uint8_t goal_y);
bool micromouse_set_walls(uint8_t x, uint8_t y, uint8_t wall_mask);
bool micromouse_plan_next(uint8_t x, uint8_t y,
                          micromouse_direction_t *direction);

#endif
