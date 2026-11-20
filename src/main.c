/* main.c - Initialisierung, Hauptschleife und serielle Diagnose */
#include "platform.h"
#include "motor.h"
#include "motor_control.h"
#include "encoder.h"
#include "distance_sensors.h"
#include "micromaus_config.h"
#include "floodfill_fahrt.h"
#include "stm32f4xx.h"

#define FAHRMODUS_TEST             0U
#define FAHRMODUS_FLOODFILL        1U
#define FAHRMODUS                  FAHRMODUS_TEST /* hier umschalten und neu flashen */

/* 0: Open Loop, feste Amplitude, Encoder nur Anzeige.
 * 1: Closed Loop, Amplitude nach Drehzahl + Blockier-Erkennung (braucht zaehlende Encoder). */
#define CLOSED_LOOP_ENABLED        0U
#define TEST_RPM                   90    /* ~280 mm/s am Rad */
#define TEST_FAHRZEIT_MS           4000U /* inkl. Beschleunigen */
#define TEST_ZYKLUS_MS             7000U /* Fahrzeit + Bremsen + Pause */
#define DIAGNOSEINTERVALL_MS       1000U
#define AUSRICHTZEIT_MS            300U
#define KOMMUTIERUNG_TAKT_HZ       5000U /* 200 us Raster, ~5 Grad el. pro Schritt bei 64 Hz Feld */

/* Kommutierung im Interrupt, damit blockierende UART-Ausgaben das Drehfeld nicht anhalten. */
void TIM5_IRQHandler(void)
{
    TIM5->SR = ~TIM_SR_UIF;
    motor_control_update(encoder_left_get_ticks(), encoder_right_get_ticks(),
                         millis(), DWT->CYCCNT);
}

static void kommutierung_timer_start(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->PSC = (16000000U / 1000000U) - 1U; /* 1 MHz Timertakt (APB1 = HSI 16 MHz) */
    TIM5->ARR = (1000000U / KOMMUTIERUNG_TAKT_HZ) - 1U;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM5_IRQn, 3); /* unter den Encoder-EXTIs (2), damit keine Flanke verloren geht */
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 = TIM_CR1_CEN;
}

static void print_distance(distance_sensor_id_t id)
{
    distance_reading_t reading;
    if (!distance_sensors_get(id, &reading) ||
        !reading.initialized || !reading.valid) {
        uart_print("--");
        return;
    }

    uart_print_number(reading.distance_mm);
    uart_print("mm");
}

static void print_motor_status(motor_id_t motor)
{
    motor_control_status_t status;
    if (!motor_control_get_status(motor, &status) || !status.feedback_ready) {
        uart_print("Encoder-Feedback ausstehend");
        return;
    }

    uart_print("Soll=");
    uart_print_int(status.target_rpm);
    uart_print(" Ist=");
    uart_print_int(status.measured_rpm);
    uart_print(" Error=");
    uart_print_int(status.error_rpm);
    uart_print(" RPM");
    if (status.blocked) {
        uart_print(" BLOCKIERT");
    }
}

/* Beschleunigen, halten, bremsen, Pause; jeder zweite Zyklus rueckwaerts. */
static void testmodus_update(uint32_t elapsed_ms)
{
    uint32_t zyklus = elapsed_ms / TEST_ZYKLUS_MS;
    int32_t rpm = (elapsed_ms % TEST_ZYKLUS_MS) < TEST_FAHRZEIT_MS ? TEST_RPM : 0;
    if (zyklus % 2U) {
        rpm = -rpm;
    }
    motor_control_set_speed(MOTOR_1, rpm);
    motor_control_set_speed(MOTOR_2, rpm);
}

int main(void)
{
    platform_init();
    distance_sensors_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    motor_init();
    encoder_init();
    encoder_left_reset(0);
    encoder_right_reset(0);

    motor_control_init(CLOSED_LOOP_ENABLED != 0U, DWT->CYCCNT);
    delay_ms(AUSRICHTZEIT_MS);
    uint32_t start = millis();
    uint32_t cycles = DWT->CYCCNT;
    int32_t left_ticks = encoder_left_get_ticks();
    int32_t right_ticks = encoder_right_get_ticks();
    motor_control_start(left_ticks, right_ticks, start, cycles);
    kommutierung_timer_start();
    uint32_t last_diagnostic_ms = start;

    uart_print("\r\n==========================================\r\n");
    uart_print("  Micromaus: zwei Encoder, Gegenlauf      \r\n");
    uart_print("==========================================\r\n");
    uart_print("Regelmodus: ");
    uart_print(CLOSED_LOOP_ENABLED ? "Closed Loop\r\n" : "Open Loop mit Rampe\r\n");
    uart_print("Encoder PPR: ");
    uart_print_number(ENCODER_PPR);
    uart_print(" | Counts/Umdrehung: ");
    uart_print_number(ENCODER_COUNTS_PER_REV);
    uart_print(" | Getriebe x10: 1:");
    uart_print_number(GETRIEBE_UNTERSETZUNG_X10);
    uart_print(" | Polpaare: ");
    uart_print_number(POLPAARE);
    uart_print("\r\nLinks: Encoder AL=PA15, BL=PB3; Motor 1 vorwaerts.\r\n");
    uart_print("Rechts: Encoder AR=PB7, BR=PB6; Motor 2 rueckwaerts.\r\n");
    uart_print(FAHRMODUS == FAHRMODUS_TEST ? "Modus: Test\r\n" : "Modus: Floodfill\r\n");

    pc13_set(true);
    if (FAHRMODUS == FAHRMODUS_FLOODFILL) {
        floodfill_fahren();
    }
    while (1) {
        uint32_t now = millis();
        if (FAHRMODUS == FAHRMODUS_TEST) {
            testmodus_update(now - start);
        }
        left_ticks = encoder_left_get_ticks();
        right_ticks = encoder_right_get_ticks();
        distance_sensors_update(now); /* blockierendes I2C ist ok, Kommutierung laeuft in TIM5 */

        if (now - last_diagnostic_ms >= DIAGNOSEINTERVALL_MS) {
            uart_print("L ticks=");
            uart_print_int(left_ticks);
            uart_print(" AB=");
            uart_print_number(encoder_left_get_state());
            uart_print(" | R ticks=");
            uart_print_int(right_ticks);
            uart_print(" AB=");
            uart_print_number(encoder_right_get_state());
            uart_print(" | ToF L=");
            print_distance(DISTANCE_SENSOR_LEFT);
            uart_print(" V=");
            print_distance(DISTANCE_SENSOR_FRONT);
            uart_print(" R=");
            print_distance(DISTANCE_SENSOR_RIGHT);
            uart_print(" | Rad L ");
            print_motor_status(MOTOR_1);
            uart_print(" | R ");
            print_motor_status(MOTOR_2);
            uart_print("\r\n");
            last_diagnostic_ms = now;
        }
    }
}
