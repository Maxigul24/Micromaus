/* encoder.h - Auslesen des AMT102-V Encoders an Stecker P6 (AL = PA15, BL = PB3) */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Initialisiert PA15 und PB3 mit EXTI-Interrupts fuer 4-fach Quadratur-Dekodierung */
void encoder_init(void);

/* Gibt die aktuellen Encoder-Ticks zurueck (vorwaerts zaehlt hoch, rueckwaerts runter) */
int32_t encoder_get_ticks(void);

/* Setzt den Zaehler auf einen bestimmten Wert zurueck */
void encoder_reset(int32_t val);

#endif
