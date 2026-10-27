/* vl53l0x.h - Minimaltreiber VL53L0X (Adresse 0x29, Einzelmessung ~30 ms) */
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_OUT_OF_RANGE 8190   /* Messwert wenn kein Objekt erkannt */

bool vl53l0x_init(void);
bool vl53l0x_read_mm(uint16_t *mm);

#endif

