#ifndef UART_TX_H
#define UART_TX_H

/*
 * uart_tx - non-blocking output on the ST-LINK virtual COM port (USART1).
 *
 * Bytes go into a RAM ring buffer and are sent from the USART1 TX interrupt,
 * which has the lowest priority in the project. Nothing ever waits on the
 * UART inside an interrupt, and the MAX30003 interrupts can run at any time
 * during output.
 *
 * uart_tx_init() also replaces _write(), so printf() goes through the ring:
 *   - from the main loop (thread mode, interrupts on) printf waits for room
 *     with WFI while the interrupts keep running;
 *   - from an interrupt, or with interrupts masked, bytes that do not fit are
 *     dropped and counted (see uart_tx_dropped()). Do not print from
 *     interrupts anyway.
 */

#include <stddef.h>
#include <stdint.h>

#ifndef UART_TX_BUF_SIZE
#define UART_TX_BUF_SIZE      4096u     /* power of two */
#endif
#ifndef UART_TX_IRQ_PRIORITY
#define UART_TX_IRQ_PRIORITY  3u        /* below EXTI3/EXTI8 (2) and TIM6 (1) */
#endif

/* Call once after BSP_COM_Init(COM1, ...). Until then printf is not routed. */
void uart_tx_init(void);

/* Free bytes in the ring. */
size_t uart_tx_free(void);

/* Queue len bytes without waiting. Returns 0 and queues nothing if they do
 * not all fit (the line is counted as dropped), else len. */
size_t uart_tx_try_write(const char *data, size_t len);

/* Wait (main loop only) until everything queued has left the UART. */
void uart_tx_flush(void);

/* Bytes dropped because the ring was full, since start. */
uint32_t uart_tx_dropped(void);

#endif
