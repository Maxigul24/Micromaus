/* lsm6dsox.h - Minimaltreiber Gyro Z-Achse (I2C3, gemeinsam mit den VL53L0X) */
#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdbool.h>
#include <stdint.h>

/* Gyro: 104 Hz Ausgaberate, +-500 dps -> 17,5 mdps pro LSB. */
#define LSM6DSOX_ODR_HZ           104
#define LSM6DSOX_MDPS_PRO_LSB_X10 175

/* Sucht den Chip auf 0x6A/0x6B und konfiguriert den Gyro. */
bool lsm6dsox_init(void);
/* neu = true, wenn seit dem letzten Aufruf ein neuer Messwert vorliegt. */
bool lsm6dsox_gyro_z(int16_t *raw, bool *neu);

#endif
