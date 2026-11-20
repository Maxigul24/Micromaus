/*
 * encoder.c - Quadratur-Encoder-Treiber fuer beide AMT102-V Encoder
 *
 * Links:  AL -> PA15, BL -> PB3
 * Rechts: AR -> PB7,  BR -> PB6
 *
 * Da TIM2 bereits fuer die 3 PWM-Phasen von DRV2 verwendet wird,
 * liest diese Datei die Quadratur-Signale per EXTI (Flanken-Interrupt)
 * mit voller 4-fach Aufloesung ein.
 */
#include "encoder.h"
#include "stm32f4xx.h"

static volatile int32_t encoder_left_pos = 0;
static volatile int32_t encoder_right_pos = 0;
static volatile uint8_t encoder_left_last_state = 0;
static volatile uint8_t encoder_right_last_state = 0;

/* Quadratur-Uebergangstabelle (4-fach Dekodierung) */
static const int8_t quad_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static inline uint8_t read_left_ab(void)
{
    /* AL auf PA15 (Bit 1), BL auf PB3 (Bit 0). */
    uint8_t a = (GPIOA->IDR & (1U << 15)) ? 2 : 0;
    uint8_t b = (GPIOB->IDR & (1U << 3))  ? 1 : 0;
    return (uint8_t)(a | b);
}

static inline uint8_t read_right_ab(void)
{
    /* AR auf PB7 (Bit 1), BR auf PB6 (Bit 0). */
    uint8_t a = (GPIOB->IDR & (1U << 7)) ? 2 : 0;
    uint8_t b = (GPIOB->IDR & (1U << 6)) ? 1 : 0;
    return (uint8_t)(a | b);
}

static inline void process_edge(volatile int32_t *position,
                                volatile uint8_t *last_state,
                                uint8_t current)
{
    uint8_t idx = (uint8_t)((*last_state << 2) | current);
    *position += quad_table[idx & 0x0F];
    *last_state = current;
}

/* EXTI-Handler fuer PB3 (BL). */
void EXTI3_IRQHandler(void)
{
    if (EXTI->PR & (1U << 3)) {
        EXTI->PR = (1U << 3);
        process_edge(&encoder_left_pos, &encoder_left_last_state,
                     read_left_ab());
    }
}

/* EXTI-Handler fuer PB6/PB7 (BR/AR). */
void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1U << 6) | (1U << 7));
    if (pending != 0) {
        EXTI->PR = pending;
        process_edge(&encoder_right_pos, &encoder_right_last_state,
                     read_right_ab());
    }
}

/* EXTI-Handler fuer PA15 (AL). */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1U << 15)) {
        EXTI->PR = (1U << 15);
        process_edge(&encoder_left_pos, &encoder_left_last_state,
                     read_left_ab());
    }
}

void encoder_init(void)
{
    /* Clocks fuer GPIOA, GPIOB und SYSCFG */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PA15 (AL), PB3 (BL), PB7 (AR) und PB6 (BR) als Eingange mit Pull-Up. */
    GPIOA->MODER &= ~(3U << (15 * 2));
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3U << (15 * 2))) | (1U << (15 * 2));
    GPIOB->MODER &= ~((3U << (3 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->PUPDR = (GPIOB->PUPDR & ~((3U << (3 * 2)) |
                                    (3U << (6 * 2)) |
                                    (3U << (7 * 2)))) |
                   (1U << (3 * 2)) | (1U << (6 * 2)) | (1U << (7 * 2));

    encoder_left_last_state = read_left_ab();
    encoder_right_last_state = read_right_ab();

    /* EXTI3 auf PB3 leiten (SYSCFG_EXTICR1: Bits 15..12 = 0001 fuer Port B) */
    SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~(0xFU << (3 * 4))) | (1U << (3 * 4));

    /* EXTI15 auf PA15 leiten (SYSCFG_EXTICR4: Bits 15..12 = 0000 fuer Port A) */
    SYSCFG->EXTICR[3] &= ~(0xFU << ((15 % 4) * 4));

    /* EXTI6 und EXTI7 auf PB6/PB7 leiten (SYSCFG_EXTICR2). */
    SYSCFG->EXTICR[1] =
        (SYSCFG->EXTICR[1] & ~((0xFU << ((6 % 4) * 4)) |
                               (0xFU << ((7 % 4) * 4)))) |
        (1U << ((6 % 4) * 4)) | (1U << ((7 % 4) * 4));

    /* Flankenerkennung bei steigender UND fallender Flanke */
    const uint32_t encoder_lines = (1U << 3) | (1U << 6) |
                                   (1U << 7) | (1U << 15);
    EXTI->PR = encoder_lines;
    EXTI->IMR  |= encoder_lines;
    EXTI->RTSR |= encoder_lines;
    EXTI->FTSR |= encoder_lines;

    /* Interrupts im NVIC aktivieren */
    NVIC_SetPriority(EXTI3_IRQn, 2);
    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

int32_t encoder_left_get_ticks(void)
{
    return encoder_left_pos;
}

int32_t encoder_right_get_ticks(void)
{
    return encoder_right_pos;
}

uint8_t encoder_left_get_state(void)
{
    return read_left_ab();
}

uint8_t encoder_right_get_state(void)
{
    return read_right_ab();
}

void encoder_left_reset(int32_t value)
{
    encoder_left_pos = value;
}

void encoder_right_reset(int32_t value)
{
    encoder_right_pos = value;
}
