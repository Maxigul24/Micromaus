/* Encoder 1 (links): AL = PA15, BL = PB3. Encoder 2 (rechts): AR = PB7, BR = PB6. */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Initialisiert beide Quadratur-Encoder mit EXTI-Interrupts und 4-fach Dekodierung. */
void encoder_init(void);

int32_t encoder_left_get_ticks(void);
int32_t encoder_right_get_ticks(void);
uint8_t encoder_left_get_state(void);
uint8_t encoder_right_get_state(void);

void encoder_left_reset(int32_t value);
void encoder_right_reset(int32_t value);

#endif
