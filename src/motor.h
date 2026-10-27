/* motor.h - Ansteuerung der zwei DRV8313 Motortreiber */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_1 = 0,   /* DRV1 */
    MOTOR_2 = 1    /* DRV2 */
} motor_id_t;

void motor_init(void);
void motor_set(motor_id_t m, uint16_t winkel, uint16_t staerke);
void motor_off(motor_id_t m);

#endif
