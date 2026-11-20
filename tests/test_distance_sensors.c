#include "distance_sensors.h"
#include "platform.h"
#include "vl53l0x.h"

#include <assert.h>
#include <stdio.h>

static bool xshut_enabled[TOF_SENSOR_COUNT];
static uint8_t initialized_addresses[TOF_SENSOR_COUNT];
static uint8_t started_address;
static uint8_t polls;
static uint8_t polls_before_ready = 1;

void tof_xshut_set(tof_sensor_id_t sensor, bool enabled)
{
    xshut_enabled[sensor] = enabled;
}

void delay_ms(uint32_t ms)
{
    (void)ms;
}

void uart_print(const char *text)
{
    (void)text;
}

void uart_print_number(uint32_t number)
{
    (void)number;
}

bool vl53l0x_init(vl53l0x_t *sensor, uint8_t address)
{
    sensor->address = address;
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        if (address == (uint8_t)(0x30U + i)) initialized_addresses[i] = address;
    }
    return true;
}

bool vl53l0x_start(vl53l0x_t *sensor)
{
    started_address = sensor->address;
    polls = 0;
    return true;
}

bool vl53l0x_read_if_ready(vl53l0x_t *sensor, uint16_t *mm, bool *ready)
{
    *ready = polls++ >= polls_before_ready;
    if (*ready) *mm = (uint16_t)(1000U + sensor->address);
    return true;
}

static distance_reading_t get_reading(distance_sensor_id_t id)
{
    distance_reading_t reading;
    assert(distance_sensors_get(id, &reading));
    return reading;
}

int main(void)
{
    distance_sensors_init();
    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        assert(xshut_enabled[i]);
        assert(initialized_addresses[i] == (uint8_t)(0x30U + i));
    }

    distance_sensors_update(0);
    assert(started_address == 0x30U);
    distance_sensors_update(5);
    assert(!get_reading(DISTANCE_SENSOR_LEFT).valid);
    distance_sensors_update(10);
    distance_reading_t left = get_reading(DISTANCE_SENSOR_LEFT);
    assert(left.initialized && left.valid && left.distance_mm == 1048U);

    distance_sensors_update(10);
    assert(started_address == 0x31U);
    distance_sensors_update(15);
    distance_sensors_update(20);
    distance_reading_t front = get_reading(DISTANCE_SENSOR_FRONT);
    assert(front.initialized && front.valid && front.distance_mm == 1049U);

    polls_before_ready = UINT8_MAX;
    distance_sensors_update(20);
    assert(started_address == 0x32U);
    distance_sensors_update(25);
    distance_sensors_update(120);
    distance_reading_t right = get_reading(DISTANCE_SENSOR_RIGHT);
    assert(right.initialized && !right.valid);

    assert(!distance_sensors_get((distance_sensor_id_t)-1, &right));
    assert(!distance_sensors_get(DISTANCE_SENSOR_COUNT, &right));
    assert(!distance_sensors_get(DISTANCE_SENSOR_LEFT, 0));

    puts("distance sensor tests passed");
    return 0;
}
