#include "motor.h"
#include "motor_control.h"

#include <assert.h>
#include <stdio.h>

static uint16_t last_angle[2];
static uint16_t last_strength[2];

void motor_set(motor_id_t motor, uint16_t angle, uint16_t strength)
{
    last_angle[motor] = angle;
    last_strength[motor] = strength;
}

int main(void)
{
    motor_control_status_t status;

    /* Ohne Sollwert steht das Feld. */
    motor_control_init(false, 0);
    motor_control_start(0, 0, 0, 0);
    motor_control_update(0, 0, 100, 1600000U);
    assert(last_angle[MOTOR_1] == 0U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.target_rpm == 0);
    assert(!status.feedback_ready);

    /* Rampe: 120 RPM/s -> nach 100 ms 12 RPM Rad = 52 RPM Motor, 3 Polpaare. */
    motor_control_init(false, 0);
    motor_control_start(0, 0, 0, 0);
    motor_control_set_speed(MOTOR_1, 60);
    motor_control_set_speed(MOTOR_2, 60);
    motor_control_update(0, 0, 100, 1600000U);
    assert(last_angle[MOTOR_1] == 46U);
    assert(last_angle[MOTOR_2] == 313U); /* Gegenlauf */
    assert(last_strength[MOTOR_1] == 260U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.target_rpm == 12);

    /* Rampe begrenzt auf den Sollwert. */
    motor_control_update(0, 0, 200, 17600000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.target_rpm == 60);

    /* Bremsen und Rueckwaerts. */
    motor_control_set_speed(MOTOR_1, -60);
    motor_control_update(0, 0, 300, 33600000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.target_rpm == -60);

    /* Open Loop regelt nicht, misst aber. */
    motor_control_update(0, 0, 600, 41600000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.feedback_ready);
    assert(status.measured_rpm == 0);
    assert(status.error_rpm == 60);
    motor_control_update(0, 0, 1200, 51200000U);
    assert(last_strength[MOTOR_1] == 260U);

    /* Closed Loop: 4096 Ticks in 500 ms = 60 RPM -> kein Fehler. */
    motor_control_init(true, 0);
    motor_control_start(0, 0, 0, 0);
    motor_control_set_speed(MOTOR_1, 60);
    motor_control_set_speed(MOTOR_2, 60);
    motor_control_update(4096, -4096, 500, 8000000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.target_rpm == 60);
    assert(status.measured_rpm == 60);
    assert(status.error_rpm == 0);
    assert(motor_control_get_status(MOTOR_2, &status));
    assert(status.measured_rpm == 60);

    /* Amplitude nach Drehzahl: 260 + 0,6 * 60 RPM. */
    assert(last_strength[MOTOR_1] == 296U);
    assert(last_strength[MOTOR_2] == 296U);
    assert(!status.blocked);

    /* Rad blockiert -> Feld stoppt, Sollwert wird ignoriert bis 0 angefordert. */
    motor_control_update(4096, -4096, 1000, 16000000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.blocked);
    assert(status.target_rpm == 0);
    motor_control_set_speed(MOTOR_1, 60);
    motor_control_update(4096, -4096, 1100, 17600000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(status.target_rpm == 0);
    assert(last_strength[MOTOR_1] == 260U);

    motor_control_set_speed(MOTOR_1, 0);
    motor_control_set_speed(MOTOR_1, 60);
    motor_control_update(4096, -4096, 1200, 19200000U);
    assert(motor_control_get_status(MOTOR_1, &status));
    assert(!status.blocked);
    assert(status.target_rpm == 12);

    puts("motor control tests passed");
    return 0;
}
