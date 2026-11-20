/* Motorsteuerung: Open-Loop-Drehfeld mit Rampe, optional Staerke-Regelung per Encoder. */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "motor.h"

#include <stdbool.h>
#include <stdint.h>

/* Alle Drehzahlen in RPM am Rad (Abtrieb). */
typedef struct {
    int32_t target_rpm;    /* aktuell gefahrener Rampenwert, vorzeichenbehaftet */
    int32_t measured_rpm;  /* Betrag aus dem Encoder */
    int32_t error_rpm;     /* |target| - measured */
    bool feedback_ready;
    bool blocked;          /* nur Closed Loop: Feld gestoppt, Freigabe mit set_speed(0) */
} motor_control_status_t;

void motor_control_init(bool closed_loop, uint32_t cycles);
void motor_control_start(int32_t left_ticks, int32_t right_ticks,
                         uint32_t now, uint32_t cycles);
/* Positiv = vorwaerts. Die Rampe faehrt den Wert mit RAMPE_ABTRIEB_RPM_PRO_S an. */
void motor_control_set_speed(motor_id_t motor, int32_t abtrieb_rpm);
void motor_control_update(int32_t left_ticks, int32_t right_ticks,
                          uint32_t now, uint32_t cycles);
bool motor_control_get_status(motor_id_t motor,
                              motor_control_status_t *status);

#endif
