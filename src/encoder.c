/*
 * encoder.c - Quadratur-Encoder-Treiber fuer AMT102-V an P6
 *
 * Belegung laut Schaltplan:
 *   Stecker P6:
 *     Pin 1: BL  -> PB3 (STM32 Pin 55)
 *     Pin 2: 5V
 *     Pin 3: AL  -> PA15 (STM32 Pin 50)
 *     Pin 4: XL  -> nicht verbunden
 *     Pin 5: GND
 *
 * Da TIM2 bereits fuer die 3 PWM-Phasen von DRV2 verwendet wird,
 * liest diese Datei die Quadratur-Signale per EXTI (Flanken-Interrupt)
 * mit voller 4-fach Aufloesung ein.
 */
#include "encoder.h"
#include "stm32f4xx.h"

static volatile int32_t encoder_pos = 0;
static volatile uint8_t last_state = 0;

/* Quadratur-Uebergangstabelle (4-fach Dekodierung) */
static const int8_t quad_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline uint8_t read_ab(void)
{
    /* AL auf PA15 (Bit 1), BL auf PB3 (Bit 0) */
    uint8_t a = (GPIOA->IDR & (1U << 15)) ? 2 : 0;
    uint8_t b = (GPIOB->IDR & (1U << 3))  ? 1 : 0;
    return (uint8_t)(a | b);
}

static inline void process_edge(void)
{
    uint8_t current = read_ab();
    uint8_t idx = (uint8_t)((last_state << 2) | current);
    encoder_pos += quad_table[idx & 0x0F];
    last_state = current;
}

/* EXTI-Handler fuer PB3 (BL) */
void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & (1U << 3)) {
        EXTI->PR = (1U << 3);   /* Flag loeschen */
        process_edge();
    }
}

/* EXTI-Handler fuer PA15 (AL) */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << 15)) {
        EXTI->PR = (1U << 15);  /* Flag loeschen */
        process_edge();
    }
}

void encoder_init(void)
{
    /* Clocks fuer GPIOA, GPIOB und SYSCFG */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PA15 (AL) als Eingang mit Pull-Up */
    GPIOA->MODER &= ~(3U << (15 * 2));
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3U << (15 * 2))) | (1U << (15 * 2));

    /* PB3 (BL) als Eingang mit Pull-Up */
    GPIOB->MODER &= ~(3U << (3 * 2));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~(3U << (3 * 2))) | (1U << (3 * 2));

    /* Initialen Zustand einlesen */
    last_state = read_ab();

    /* EXTI3 auf PB3 leiten (SYSCFG_EXTICR1: Bits 15..12 = 0001 fuer Port B) */
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(0xFU << (3 * 4))) | (1U << (3 * 4));

    /* EXTI15 auf PA15 leiten (SYSCFG_EXTICR4: Bits 15..12 = 0000 fuer Port A) */
    SYSCFG->EXTICR[3] &= ~(0xFU << ((15 % 4) * 4));

    /* Flankenerkennung bei steigender UND fallender Flanke */
    EXTI->IMR  |= (1U << 3) | (1U << 15);
    EXTI->RTSR |= (1U << 3) | (1U << 15);
    EXTI->FTSR |= (1U << 3) | (1U << 15);

    /* Interrupts im NVIC aktivieren */
    NVIC_SetPriority(EXTI3_IRQn, 2);
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

int32_t encoder_get_ticks(void)
{
    return encoder_pos;
}

void encoder_reset(int32_t val)
{
    encoder_pos = val;
}
