#include "micromouse.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    micromouse_direction_t direction;

    assert(!micromouse_plan_next(0, 0, &direction));
    assert(!micromouse_init(MICROMOUSE_MAZE_WIDTH, 0));
    assert(micromouse_init(2, 0));

    assert(micromouse_plan_next(0, 0, &direction));
    assert(direction == MICROMOUSE_EAST);

    assert(micromouse_set_walls(0, 0, MICROMOUSE_WALL_EAST));
    assert(micromouse_plan_next(0, 0, &direction));
    assert(direction == MICROMOUSE_SOUTH);

    assert(micromouse_init(0, 0));
    assert(micromouse_set_walls(0, 0, MICROMOUSE_WALL_EAST));
    assert(micromouse_plan_next(1, 0, &direction));
    assert(direction == MICROMOUSE_SOUTH);

    assert(micromouse_init(2, 0));
    assert(micromouse_set_walls(0, 0, 0));
    assert(micromouse_plan_next(0, 0, &direction));
    assert(direction == MICROMOUSE_EAST);
    assert(!micromouse_plan_next(2, 0, &direction));

    assert(!micromouse_set_walls(0, 0, 0x10));
    assert(!micromouse_set_walls(MICROMOUSE_MAZE_WIDTH, 0, 0));
    assert(!micromouse_plan_next(0, 0, 0));

    assert(micromouse_init(2, 2));
    assert(micromouse_set_walls(0, 0, 0x0F));
    assert(!micromouse_plan_next(0, 0, &direction));

    puts("micromouse tests passed");
    return 0;
}
