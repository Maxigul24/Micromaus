/* lsm6dsox.c - Registerwerte laut ST-Datenblatt LSM6DSOX (DS13604). */
#include "lsm6dsox.h"
#include "platform.h"

#define REG_WHO_AM_I   0x0F /* liefert 0x6C */
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_CTRL9_XL   0x18
#define REG_STATUS     0x1E
#define REG_OUTZ_L_G   0x26
#define WHO_AM_I_WERT  0x6C

static uint8_t adresse;

static bool schreibe(uint8_t reg, uint8_t wert)
{
    uint8_t b[2] = { reg, wert };
    return i2c_write(adresse, b, 2);
}

bool lsm6dsox_init(void)
{
    static const uint8_t kandidaten[2] = { 0x6A, 0x6B }; /* SA0 auf GND bzw. VDD */
    uint8_t id = 0;

    adresse = 0;
    for (uint8_t i = 0; i < 2; i++) {
        if (i2c_read(kandidaten[i], REG_WHO_AM_I, &id, 1) && id == WHO_AM_I_WERT) {
            adresse = kandidaten[i];
            break;
        }
    }
    if (adresse == 0) {
        return false;
    }

    return schreibe(REG_CTRL3_C, 0x44) &&  /* BDU + Auto-Increment */
           schreibe(REG_CTRL9_XL, 0xE2) && /* I3C aus (empfohlen bei reinem I2C) */
           schreibe(REG_CTRL2_G, 0x44);    /* ODR 104 Hz, 500 dps */
}

bool lsm6dsox_gyro_z(int16_t *raw, bool *neu)
{
    uint8_t status;
    uint8_t b[2];

    *neu = false;
    if (adresse == 0 || !i2c_read(adresse, REG_STATUS, &status, 1)) {
        return false;
    }
    if (!(status & 0x02)) { /* GDA: neuer Gyro-Wert */
        return true;
    }
    if (!i2c_read(adresse, REG_OUTZ_L_G, b, 2)) {
        return false;
    }
    *raw = (int16_t)((uint16_t)b[1] << 8 | b[0]);
    *neu = true;
    return true;
}
