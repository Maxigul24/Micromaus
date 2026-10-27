/*
 * motor.c - Ansteuerung der zwei DRV8313 Motortreiber
 *
 *          PWM U  PWM V  PWM W   EN U  EN V  EN W   Timer
 *   DRV1:  PA6    PA7    PB0     PB5   PC5   PB1    TIM3
 *   DRV2:  PA0    PA1    PA2     PB4   PC3   PB2    TIM2
 *
 * Prinzip: Jede Phase bekommt eine Sinus-Spannung, um 120 Grad versetzt.
 * Dadurch entsteht ein drehendes Magnetfeld, dem der Motor folgt.
 */
#include "motor.h"
#include "stm32f4xx.h"
#include <math.h>

#define PWM_MAX 800   /* 16 MHz / 800 = 20 kHz PWM */

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} pin_t;

static TIM_TypeDef *const timer[2] = { TIM3, TIM2 };
static const uint8_t timer_af[2]   = { 2, 1 };   /* Alternate Function Nummer */

static const pin_t pwm_pin[2][3] = {
    { { GPIOA, 6 }, { GPIOA, 7 }, { GPIOB, 0 } },
    { { GPIOA, 0 }, { GPIOA, 1 }, { GPIOA, 2 } },
};

static const pin_t en_pin[2][3] = {
    { { GPIOB, 5 }, { GPIOC, 5 }, { GPIOB, 1 } },
    { { GPIOB, 4 }, { GPIOC, 3 }, { GPIOB, 2 } },
};

static int16_t sinus[360];   /* sin(Grad) * 1000 */

/* mode: 1 = Ausgang, 2 = Alternate Function (Timer) */
static void pin_mode(pin_t p, uint32_t mode)
{
    p.port->MODER &= ~(3U << (p.pin * 2));
    p.port->MODER |= mode << (p.pin * 2);
}

static void pin_af(pin_t p, uint32_t af)
{
    uint32_t reg = p.pin / 8;
    uint32_t pos = (p.pin % 8) * 4;
    p.port->AFR[reg] &= ~(0xFU << pos);
    p.port->AFR[reg] |= af << pos;
}

static void pin_write(pin_t p, int high)
{
    if (high) {
        p.port->BSRR = 1U << p.pin;
    } else {
        p.port->BSRR = 1U << (p.pin + 16);
    }
}

void motor_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    for (int i = 0; i < 360; i++) {
        sinus[i] = (int16_t)(1000.0f * sinf(i * 3.14159f / 180.0f));
    }

    for (int m = 0; m < 2; m++) {
        TIM_TypeDef *t = timer[m];
        t->ARR   = PWM_MAX - 1;
        t->CCMR1 = (6U << 4) | (6U << 12);   /* CH1 + CH2: PWM-Modus */
        t->CCMR2 = (6U << 4);                /* CH3: PWM-Modus */
        t->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
        t->CR1   = TIM_CR1_CEN;              /* Timer starten */

        for (int ph = 0; ph < 3; ph++) {
            pin_af(pwm_pin[m][ph], timer_af[m]);
            pin_mode(pwm_pin[m][ph], 2);

            pin_write(en_pin[m][ph], 0);     /* Treiber zuerst aus */
            pin_mode(en_pin[m][ph], 1);
        }
    }
}

/*
 * winkel:  elektrischer Winkel 0..359 Grad
 * staerke: 0..500 (Sinus-Amplitude in Promille, 500 = maximal)
 */
void motor_set(motor_id_t m, uint16_t winkel, uint16_t staerke)
{
    TIM_TypeDef *t = timer[m];

    for (int ph = 0; ph < 3; ph++) {
        uint16_t w = (winkel + ph * 120) % 360;
        int32_t duty = 500 + (staerke * sinus[w]) / 1000;
        uint32_t ccr = (duty * PWM_MAX) / 1000;

        if (ph == 0) t->CCR1 = ccr;
        if (ph == 1) t->CCR2 = ccr;
        if (ph == 2) t->CCR3 = ccr;

        pin_write(en_pin[m][ph], 1);
    }
}

void motor_off(motor_id_t m)
{
    TIM_TypeDef *t = timer[m];
    t->CCR1 = 0;
    t->CCR2 = 0;
    t->CCR3 = 0;

    for (int ph = 0; ph < 3; ph++) {
        pin_write(en_pin[m][ph], 0);
    }
}
