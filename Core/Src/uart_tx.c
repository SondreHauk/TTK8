/*
 * uart_tx.c - interrupt-driven, non-blocking USART1 output, see uart_tx.h.
 *
 * The BSP configures USART1 (115200 8N1) in BSP_COM_Init(); this module only
 * switches on its TX FIFO and the "TX FIFO not full" interrupt, and writes
 * TDR directly. The BSP's blocking __io_putchar() is no longer used because
 * _write() below replaces the weak one in syscalls.c.
 *
 * USART1_IRQHandler is defined here, not in stm32u3xx_it.c, so CubeMX
 * regeneration does not touch it. Do not enable the USART1 interrupt in
 * CubeMX, or the handler will be defined twice.
 */
#include "uart_tx.h"

#include "main.h"

extern int __io_putchar(int ch);           /* BSP, blocking; used before uart_tx_init() */

#if (UART_TX_BUF_SIZE & (UART_TX_BUF_SIZE - 1u)) != 0
#error "UART_TX_BUF_SIZE must be a power of two"
#endif

#define MASK (UART_TX_BUF_SIZE - 1u)

static char              buf[UART_TX_BUF_SIZE];
static volatile uint32_t head;              /* written by the producer (main loop) */
static volatile uint32_t tail;              /* written by the USART1 interrupt */
static volatile uint32_t dropped;
static volatile uint8_t  ready;

static USART_TypeDef *const uart = USART1;

static inline uint32_t used(void) { return head - tail; }

static inline void kick(void)
{
    /* Setting TXFNFIE is atomic (single RMW from thread mode, and the ISR
       only clears it), and is enough to start an idle transmitter. */
    SET_BIT(uart->CR1, USART_CR1_TXEIE_TXFNFIE);
}

void uart_tx_init(void)
{
    UART_HandleTypeDef *h = &hcom_uart[COM1];

    /* 8-byte TX FIFO: up to 8 bytes per interrupt instead of 1. */
    (void)HAL_UARTEx_SetTxFifoThreshold(h, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_EnableFifoMode(h);

    head = tail = 0;
    dropped = 0;
    HAL_NVIC_SetPriority(USART1_IRQn, UART_TX_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    ready = 1;
}

size_t uart_tx_free(void)
{
    return UART_TX_BUF_SIZE - used();
}

/* Copy len bytes that are known to fit. Single producer: only head moves. */
static void put(const char *data, size_t len)
{
    uint32_t h = head;
    for (size_t i = 0; i < len; i++) buf[(h + i) & MASK] = data[i];
    __DMB();
    head = h + (uint32_t)len;
    kick();
}

size_t uart_tx_try_write(const char *data, size_t len)
{
    if (!ready) return 0;
    if (len > uart_tx_free()) {
        dropped += (uint32_t)len;
        return 0;
    }
    put(data, len);
    return len;
}

static int can_wait(void)
{
    /* Thread mode with interrupts enabled: the USART1 interrupt can drain. */
    return __get_IPSR() == 0u && __get_PRIMASK() == 0u;
}

void uart_tx_flush(void)
{
    if (!ready || !can_wait()) return;
    while (used() != 0u || !(uart->ISR & USART_ISR_TC)) { }
}

uint32_t uart_tx_dropped(void)
{
    return dropped;
}

void USART1_IRQHandler(void)
{
    uint32_t t = tail;
    while ((uart->ISR & USART_ISR_TXE_TXFNF) && t != head) {
        uart->TDR = (uint8_t)buf[t & MASK];
        t++;
    }
    tail = t;
    if (t == head) CLEAR_BIT(uart->CR1, USART_CR1_TXEIE_TXFNFIE);
}

/* printf/puts end up here (replaces the weak _write in syscalls.c). */
int _write(int file, char *ptr, int len)
{
    (void)file;
    if (len <= 0) return 0;

    if (!ready) {                            /* before uart_tx_init: blocking BSP path */
        for (int i = 0; i < len; i++) (void)__io_putchar(ptr[i]);
        return len;
    }

    size_t left = (size_t)len;
    while (left > 0u) {
        size_t room = uart_tx_free();
        if (room == 0u) {
            if (!can_wait()) {               /* interrupt context: never wait */
                dropped += (uint32_t)left;
                break;
            }
            __WFI();                         /* USART1 interrupt makes room */
            continue;
        }
        size_t n = left < room ? left : room;
        put(ptr, n);
        ptr  += n;
        left -= n;
    }
    return len;
}
