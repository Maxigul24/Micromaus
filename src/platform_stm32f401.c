/*
 * platform_stm32f401.c - Treiber fuer STM32F401RET6
 *
 * Pinbelegung laut Schaltplan:
 *   PA8:  I2C3 SCL (P2 Pin 2)
 *   PC9:  I2C3 SDA (P2 Pin 3)
 *   PC2:  TOF_XSHUT_L (P2 Pin 4)
 *   PC0:  TOF_XSHUT_V
 *   PC1:  TOF_XSHUT_R
 *   PC14: Output (HIGH wenn Distanz < 30mm, sonst LOW)
 *   PC13: Heartbeat Output
 *   PC6:  USART6 TX (115200 Baud)
 */
#include "platform.h"
#include "stm32f4xx.h"

#define SYSCLK_HZ 16000000UL

static volatile uint32_t ticks;

void SysTick_Handler(void)
{
    ticks++;
}

uint32_t millis(void)
{
    return ticks;
}

void delay_ms(uint32_t ms)
{
    uint32_t t = ticks;
    while ((ticks - t) < ms) {}
}

/* ---------- I2C Low-Level (PA8 SCL, PC9 SDA) ---------- */
static inline void i2c_delay(void)
{
    /* ~2.5us bei 16MHz -> ca. 100-150 kHz I2C Takt */
    for (volatile int i = 0; i < 14; i++) {
        __NOP();
    }
}

static inline void scl_hi(void)
{
    GPIOA->BSRR = (1U << 8);
    /* Clock stretching Support: warten bis Slave SCL freigibt */
    uint32_t timeout = 10000;
    while (!(GPIOA->IDR & (1U << 8)) && --timeout) {}
}

static inline void scl_lo(void)
{
    GPIOA->BSRR = (1U << (8 + 16));
}

static inline void sda_hi(void)
{
    GPIOC->BSRR = (1U << 9);
}

static inline void sda_lo(void)
{
    GPIOC->BSRR = (1U << (9 + 16));
}

static inline uint8_t sda_read(void)
{
    return (GPIOC->IDR & (1U << 9)) ? 1 : 0;
}

static void i2c_start(void)
{
    sda_hi();
    scl_hi();
    i2c_delay();
    sda_lo();
    i2c_delay();
    scl_lo();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_lo();
    i2c_delay();
    scl_hi();
    i2c_delay();
    sda_hi();
    i2c_delay();
}

static bool i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if (byte & (1U << i)) {
            sda_hi();
        } else {
            sda_lo();
        }
        i2c_delay();
        scl_hi();
        i2c_delay();
        scl_lo();
        i2c_delay();
    }

    /* ACK Bit lesen */
    sda_hi();
    i2c_delay();
    scl_hi();
    i2c_delay();
    bool ack = (sda_read() == 0);
    scl_lo();
    i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t byte = 0;
    sda_hi();
    for (int i = 7; i >= 0; i--) {
        i2c_delay();
        scl_hi();
        i2c_delay();
        if (sda_read()) {
            byte |= (1U << i);
        }
        scl_lo();
    }

    /* ACK / NACK senden */
    if (ack) {
        sda_lo();
    } else {
        sda_hi();
    }
    i2c_delay();
    scl_hi();
    i2c_delay();
    scl_lo();
    sda_hi();
    i2c_delay();
    return byte;
}

bool i2c_write(uint8_t addr, const uint8_t *data, uint8_t len)
{
    i2c_start();
    if (!i2c_write_byte((uint8_t)(addr << 1))) {
        i2c_stop();
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) {
            i2c_stop();
            return false;
        }
    }
    i2c_stop();
    return true;
}

bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    /* Registeradresse schreiben */
    i2c_start();
    if (!i2c_write_byte((uint8_t)(addr << 1))) {
        i2c_stop();
        return false;
    }
    if (!i2c_write_byte(reg)) {
        i2c_stop();
        return false;
    }

    /* Repeated Start fuer Lese-Zugriff */
    i2c_start();
    if (!i2c_write_byte((uint8_t)((addr << 1) | 1))) {
        i2c_stop();
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        bool ack = (i < (len - 1));
        data[i] = i2c_read_byte(ack);
    }
    i2c_stop();
    return true;
}

/* ---------- GPIO & UART Initialisierung ---------- */
void platform_init(void)
{
    /* 1ms SysTick */
    SysTick_Config(SYSCLK_HZ / 1000);

    /* Clocks aktivieren: GPIOA, GPIOC, PWR, USART6 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    /* --- PC6 als USART6 TX (AF8) --- */
    GPIOC->MODER  = (GPIOC->MODER & ~(3U << (6 * 2))) | (2U << (6 * 2));
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(0xFU << (6 * 4))) | (8U << (6 * 4));
    USART6->BRR = SYSCLK_HZ / 115200;
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* --- PC13 als Schaltausgang (Push-Pull, initial LOW) --- */
    GPIOC->MODER = (GPIOC->MODER & ~(3U << (13 * 2))) | (1U << (13 * 2));
    GPIOC->OTYPER &= ~(1U << 13);
    GPIOC->BSRR = (1U << (13 + 16));

    /* --- TOF XSHUT Pins (PC0: V, PC1: R, PC2: L) als Push-Pull Ausgaenge --- */
    GPIOC->MODER = (GPIOC->MODER & ~((3U << 0) | (3U << 2) | (3U << 4)))
                                 | ((1U << 0) | (1U << 2) | (1U << 4));
    /* Zunaechst alle Sensoren im Reset halten (LOW) */
    GPIOC->BSRR = (1U << (0 + 16)) | (1U << (1 + 16)) | (1U << (2 + 16));

    /* --- I2C3 Pins (PA8 = SCL, PC9 = SDA) Open-Drain mit Pull-Up --- */
    GPIOA->MODER = (GPIOA->MODER & ~(3U << (8 * 2))) | (1U << (8 * 2));
    GPIOA->OTYPER |= (1U << 8);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(3U << (8 * 2))) | (1U << (8 * 2));
    GPIOA->OSPEEDR |= (2U << (8 * 2));

    GPIOC->MODER = (GPIOC->MODER & ~(3U << (9 * 2))) | (1U << (9 * 2));
    GPIOC->OTYPER |= (1U << 9);
    GPIOC->PUPDR  = (GPIOC->PUPDR & ~(3U << (9 * 2))) | (1U << (9 * 2));
    GPIOC->OSPEEDR |= (2U << (9 * 2));

    /* I2C Bus Freigabe / 9 Taktpulse Recovery */
    sda_hi();
    scl_hi();
    for (int i = 0; i < 9; i++) {
        scl_lo();
        i2c_delay();
        scl_hi();
        i2c_delay();
    }
    i2c_stop();

    /* Sensor-Reset Sequenz: 10ms LOW, dann XSHUT HIGH */
    delay_ms(10);
    /* PC2 (TOF_XSHUT_L, Stecker P2) und PC0/PC1 freigeben */
    GPIOC->BSRR = (1U << 0) | (1U << 1) | (1U << 2);
    delay_ms(15);
}

void pc13_set(bool high)
{
    if (high) {
        GPIOC->BSRR = (1U << 13);
    } else {
        GPIOC->BSRR = (1U << (13 + 16));
    }
}

void uart_print(const char *s)
{
    while (*s) {
        while (!(USART6->SR & USART_SR_TXE)) {}
        USART6->DR = (uint8_t)*s++;
    }
}

void uart_print_number(uint32_t n)
{
    char buf[12];
    int i = 11;
    buf[i] = '\0';
    do {
        buf[--i] = '0' + (n % 10);
        n /= 10;
    } while (n);
    uart_print(&buf[i]);
}

void uart_print_int(int32_t n)
{
    if (n < 0) {
        uart_print("-");
        uart_print_number((uint32_t)(-n));
    } else {
        uart_print_number((uint32_t)n);
    }
}

