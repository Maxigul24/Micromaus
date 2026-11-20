/* Ansteuerung und Messwertverwaltung der drei VL53L0X-Distanzsensoren. */
#ifndef DISTANCE_SENSORS_H
#define DISTANCE_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DISTANCE_SENSOR_LEFT,
    DISTANCE_SENSOR_FRONT,
    DISTANCE_SENSOR_RIGHT,
    DISTANCE_SENSOR_COUNT
} distance_sensor_id_t;

typedef struct {
    bool initialized;
    bool valid;
    uint16_t distance_mm;
} distance_reading_t;

void distance_sensors_init(void);
void distance_sensors_update(uint32_t now);
bool distance_sensors_get(distance_sensor_id_t id, distance_reading_t *reading);

#endif
