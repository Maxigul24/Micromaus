/*
 * main.c - SimpleFOC Open-Loop Motorsteuerung (DRV2)
 *
 * Dreht den Motor 5 s vorwaerts, 2 s Pause, 5 s rueckwaerts.
 * Basiert exakt auf dem Open-Loop-Algorithmus von SimpleFOC:
 *   - Kontinuierliche elektrische Winkelberechnung
 *   - Sanfte Drehung ohne Taktverlust
 */
#include "platform.h"
#include "motor.h"

/*
 * EINSTELLUNGEN:
 * Wenn das Netzteil in die Strombegrenzung geht (Spannung faellt ab),
 * zittert der Treiber wegen Unterspannungsabschaltung (UVLO unter 8V).
 * Daher STAERKE moderat waehlen (150..250 ist ideal zum Testen).
 */
#define STAERKE         260    /* Etwas mehr Kraft fuer Losbrechmoment */
#define DREHZAHL_RPM    30     /* Langsamer Start (30 U/min), damit er nicht abrutscht */
#define POLPAARE        3      /* 3 Polpaare */

static void drehe_motor(motor_id_t m, int richtung, uint32_t dauer_ms)
{
    /* Schritt pro Millisekunde in Milligrad:
     * mdeg_pro_ms = (RPM * POLPAARE * 360 * 1000) / (60 * 1000) = RPM * POLPAARE * 6 */
    uint32_t step_mdeg = DREHZAHL_RPM * POLPAARE * 6;
    uint32_t winkel_mdeg = 0;
    uint32_t start = millis();

    while ((millis() - start) < dauer_ms) {
        if (richtung > 0) {
            winkel_mdeg = (winkel_mdeg + step_mdeg) % 360000;
        } else {
            winkel_mdeg = (winkel_mdeg + 360000 - step_mdeg) % 360000;
        }

        motor_set(m, (uint16_t)(winkel_mdeg / 1000), STAERKE);
        delay_ms(1);
    }

    motor_off(m);
}

int main(void)
{
    platform_init();
    motor_init();

    uart_print("\r\n==========================================\r\n");
    uart_print("  SimpleFOC Open-Loop Motortest (DRV2)    \r\n");
    uart_print("==========================================\r\n");

    while (1) {
        uart_print("DRV2: Vorwaerts (5 s)...\r\n");
        pc13_set(true);
        drehe_motor(MOTOR_2, +1, 5000);

        uart_print("DRV2: Pause (2 s)...\r\n");
        pc13_set(false);
        delay_ms(2000);

        uart_print("DRV2: Rueckwaerts (5 s)...\r\n");
        pc13_set(true);
        drehe_motor(MOTOR_2, -1, 5000);

        uart_print("DRV2: Pause (2 s)...\r\n");
        pc13_set(false);
        delay_ms(2000);
    }
}
