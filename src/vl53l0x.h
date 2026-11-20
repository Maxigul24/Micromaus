/* vl53l0x.h - Minimaltreiber fuer mehrere VL53L0X */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_OUT_OF_RANGE 8190   /* Messwert wenn kein Objekt erkannt */

typedef struct {
    uint8_t address;
    uint8_t stop_var;
} vl53l0x_t;

bool vl53l0x_init(vl53l0x_t *sensor, uint8_t address);
bool vl53l0x_start(vl53l0x_t *sensor);
bool vl53l0x_read_if_ready(vl53l0x_t *sensor, uint16_t *mm, bool *ready);

#endif
