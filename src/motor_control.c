/*
 * Open-Loop-Drehfeld (Synchronbetrieb) mit Rampe.
 *
 * Kommutierung ueber den Encoder (FOC) ist hier nicht moeglich: der AMT102 sitzt
 * hinter dem Getriebe, das Spiel verfaelscht den Rotorwinkel. Ueblich ist dann
 * Open Loop mit drehzahlabhaengiger Spannung (U = I*R + Ke*v) und den Encoder
 * nur zur Ueberwachung zu nutzen.
 *
 * closed_loop = true:  Amplitude steigt mit der Drehzahl (Gegen-EMK-Ausgleich),
 *                      Blockier-/Taktverlust-Erkennung per Encoder stoppt den Motor.
 * closed_loop = false: feste Amplitude, Encoder nur Anzeige.
 */
#include "motor_control.h"
#include "motor.h"
#include "micromaus_config.h"

#define STAERKE_START              260 /* Stillstand: deckt I*R, empirisch */
#define STAERKE_MAX                500 /* volle Sinus-Aussteuerung */
#define SYSTEMTAKT_HZ              16000000U
#define KOMMUTIERUNG_CYCLES        (SYSTEMTAKT_HZ / 20000U)
#define MESSINTERVALL_MS           500U
#define BLOCKIER_MIN_RPM           20 /* darunter ist die Messung zu grob */

typedef struct {
    motor_id_t motor;
    int direction;
    int64_t angle_microdeg;
    int32_t strength;
    int32_t last_ticks;
    int32_t previous_field_rpm;
    volatile int32_t setpoint_rpm;   /* vom Hauptprogramm gesetzt */
    int32_t ramp_millirpm;           /* aktuell gefahrene Rad-Drehzahl * 1000 */
    uint8_t feedback_enabled;
    uint32_t last_commutation;
    uint32_t last_control_ms;
    motor_control_status_t status;
} motor_control_t;

static motor_control_t left;
static motor_control_t right;

static void init_one(motor_control_t *control, motor_id_t motor,
                     int direction, bool closed_loop, uint32_t cycles)
{
    control->motor = motor;
    control->direction = direction;
    control->angle_microdeg = 0;
    control->strength = STAERKE_START;
    control->last_ticks = 0;
    control->previous_field_rpm = 0;
    control->setpoint_rpm = 0;
    control->ramp_millirpm = 0;
    control->feedback_enabled = closed_loop;
    control->last_commutation = cycles;
    control->last_control_ms = 0;
    control->status.target_rpm = 0;
    control->status.measured_rpm = 0;
    control->status.error_rpm = 0;
    control->status.feedback_ready = false;
    control->status.blocked = false;

    motor_set(motor, 0, STAERKE_START);
}

static void start_one(motor_control_t *control, int32_t ticks,
                      uint32_t now, uint32_t cycles)
{
    control->last_ticks = ticks;
    control->last_control_ms = now;
    control->last_commutation = cycles;
    control->status.feedback_ready = false;
}

static void update_one(motor_control_t *control, int32_t ticks,
                       uint32_t now, uint32_t cycles)
{
    uint32_t dt_cycles = cycles - control->last_commutation;

    if (control->status.blocked) {
        control->setpoint_rpm = 0; /* falls main gerade parallel einen Sollwert schrieb */
    }

    if (dt_cycles >= KOMMUTIERUNG_CYCLES) {
        uint32_t dt_us = dt_cycles / (SYSTEMTAKT_HZ / 1000000U);
        int64_t step = (int64_t)RAMPE_ABTRIEB_RPM_PRO_S * dt_us / 1000;
        int64_t goal = (int64_t)control->setpoint_rpm * 1000;
        int64_t ramp = control->ramp_millirpm;
        if (ramp < goal) {
            ramp = (ramp + step > goal) ? goal : ramp + step;
        } else if (ramp > goal) {
            ramp = (ramp - step < goal) ? goal : ramp - step;
        }
        control->ramp_millirpm = (int32_t)ramp;

        if (control->feedback_enabled) {
            int64_t ramp_abs = ramp < 0 ? -ramp : ramp;
            int64_t s = STAERKE_START +
                        ramp_abs * STAERKE_PRO_RPM_X10 / 10000;
            control->strength = (int32_t)(s > STAERKE_MAX ? STAERKE_MAX : s);
        }

        int32_t field_rpm = (int32_t)(control->direction * ramp *
                                      GETRIEBE_UNTERSETZUNG_X10 / 10000);
        control->angle_microdeg +=
            ((int64_t)(control->previous_field_rpm + field_rpm) *
             POLPAARE * 3 * dt_us);
        control->angle_microdeg %= 360000000;
        if (control->angle_microdeg < 0) {
            control->angle_microdeg += 360000000;
        }

        motor_set(control->motor,
                  (uint16_t)(control->angle_microdeg / 1000000),
                  (uint16_t)control->strength);
        control->previous_field_rpm = field_rpm;
        control->last_commutation = cycles;
        control->status.target_rpm = control->ramp_millirpm / 1000;
    }

    uint32_t dt_ms = now - control->last_control_ms;
    if (dt_ms < MESSINTERVALL_MS) {
        return;
    }

    int64_t delta_ticks = (int64_t)ticks - control->last_ticks;
    if (delta_ticks < 0) {
        delta_ticks = -delta_ticks;
    }
    int32_t target_abs = control->status.target_rpm < 0 ?
                         -control->status.target_rpm : control->status.target_rpm;
    int32_t measured_rpm =
        (int32_t)((delta_ticks * 60000) /
                  ((int64_t)ENCODER_COUNTS_PER_REV * dt_ms));
    control->status.measured_rpm = measured_rpm;
    control->status.error_rpm = target_abs - measured_rpm;
    control->status.feedback_ready = true;
    control->last_ticks = ticks;
    control->last_control_ms = now;

    /* Unter 1/3 des Sollwerts: Rotor ist aus dem Takt oder blockiert. Mehr Spannung
     * faengt ihn bei voller Feldfrequenz nicht wieder ein, also Feld stoppen.
     * 1/3 statt 1/2, weil die Messung waehrend der Rampe nachhinkt. */
    if (control->feedback_enabled && target_abs >= BLOCKIER_MIN_RPM &&
        measured_rpm * 3 < target_abs) {
        control->status.blocked = true;
        control->setpoint_rpm = 0;
        control->ramp_millirpm = 0;
        control->previous_field_rpm = 0;
        control->status.target_rpm = 0;
    }
}

void motor_control_init(bool closed_loop, uint32_t cycles)
{
    init_one(&left, MOTOR_1, +1, closed_loop, cycles);
    init_one(&right, MOTOR_2, -1, closed_loop, cycles);
}

void motor_control_start(int32_t left_ticks, int32_t right_ticks,
                         uint32_t now, uint32_t cycles)
{
    start_one(&left, left_ticks, now, cycles);
    start_one(&right, right_ticks, now, cycles);
}

void motor_control_set_speed(motor_id_t motor, int32_t abtrieb_rpm)
{
    motor_control_t *control = motor == MOTOR_1 ? &left :
                               motor == MOTOR_2 ? &right : 0;
    if (control == 0) {
        return;
    }
    /* Nach einer Blockade erst wieder anfahren, wenn 0 angefordert wurde. */
    if (control->status.blocked) {
        if (abtrieb_rpm != 0) {
            return;
        }
        control->status.blocked = false;
    }
    control->setpoint_rpm = abtrieb_rpm;
}

void motor_control_update(int32_t left_ticks, int32_t right_ticks,
                          uint32_t now, uint32_t cycles)
{
    update_one(&left, left_ticks, now, cycles);
    update_one(&right, right_ticks, now, cycles);
}

bool motor_control_get_status(motor_id_t motor,
                              motor_control_status_t *status)
{
    if (status == 0 || (uint32_t)motor > (uint32_t)MOTOR_2) {
        return false;
    }

    const motor_control_t *control =
        motor == MOTOR_1 ? &left : &right;
    *status = control->status;
    return true;
}
