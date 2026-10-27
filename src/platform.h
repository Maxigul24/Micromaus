/* platform.h - Hardware-Funktionen fuer STM32F401 */
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

void     platform_init(void);
uint32_t millis(void);
void     delay_ms(uint32_t ms);

/* I2C Zugriff */
bool i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);
bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);

/* Debug UART (PC6, 115200 Baud) */
void uart_print(const char *s);
void uart_print_number(uint32_t n);
void uart_print_int(int32_t n);

/* Signalausgang PC13 (HIGH wenn < 30mm, sonst LOW) */
void pc13_set(bool high);

#endif
