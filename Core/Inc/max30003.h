/*
 * max30003.h - register-level driver for the MAX30003 ECG analog front end.
 *
 * SPI: mode 0, MSB first, 8-bit frames, <= 12 MHz. Every transaction is a
 * command byte (A[6:0] << 1 | R/W) followed by 24 data bits, framed by CSB.
 * CSB is driven as a GPIO (MAX_CSB in CubeMX, Arduino D10).
 * FCLK (32.768 kHz) must be running before max30003_init() is called.
 *
 * The functions are not reentrant: call them from one context at a time
 * (ecg_acq.c masks the INTB/INT2B interrupts while it uses the bus from the
 * main loop).
 */
#ifndef MAX30003_H
#define MAX30003_H

#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "max30003_config.h"

/* ---- Registers (datasheet "User Command and Register Map") ---- */
#define MAX30003_REG_NO_OP          0x00u
#define MAX30003_REG_STATUS         0x01u
#define MAX30003_REG_EN_INT         0x02u
#define MAX30003_REG_EN_INT2        0x03u
#define MAX30003_REG_MNGR_INT       0x04u
#define MAX30003_REG_MNGR_DYN       0x05u
#define MAX30003_REG_SW_RST         0x08u
#define MAX30003_REG_SYNCH          0x09u
#define MAX30003_REG_FIFO_RST       0x0Au
#define MAX30003_REG_INFO           0x0Fu
#define MAX30003_REG_CNFG_GEN       0x10u
#define MAX30003_REG_CNFG_CAL       0x12u
#define MAX30003_REG_CNFG_EMUX      0x14u
#define MAX30003_REG_CNFG_ECG       0x15u
#define MAX30003_REG_CNFG_RTOR1     0x1Du
#define MAX30003_REG_CNFG_RTOR2     0x1Eu
#define MAX30003_REG_ECG_FIFO_BURST 0x20u
#define MAX30003_REG_ECG_FIFO       0x21u
#define MAX30003_REG_RTOR           0x25u

/* ---- STATUS / EN_INT / EN_INT2 bits ---- */
#define MAX30003_EINT       (1uL << 23)   /* ECG FIFO at or above EFIT */
#define MAX30003_EOVF       (1uL << 22)   /* ECG FIFO overflow */
#define MAX30003_FSTINT     (1uL << 21)   /* fast recovery engaged */
#define MAX30003_DCLOFFINT  (1uL << 20)   /* DC lead-off */
#define MAX30003_LONINT     (1uL << 11)
#define MAX30003_RRINT      (1uL << 10)   /* new R event, RTOR updated */
#define MAX30003_SAMP       (1uL << 9)
#define MAX30003_PLLINT     (1uL << 8)    /* PLL not locked to FCLK */
#define MAX30003_LDOFF_MASK 0x0Fu         /* LDOFF_PH, _PL, _NH, _NL */

#define MAX30003_INTB_TYPE_CMOS  0x1u     /* EN_INT[1:0] */

/* ---- ECG FIFO word: sample[23:6] (18-bit two's complement), ETAG[5:3], PTAG[2:0] ---- */
#define MAX30003_ETAG_VALID      0x0u
#define MAX30003_ETAG_FAST       0x1u
#define MAX30003_ETAG_VALID_EOF  0x2u
#define MAX30003_ETAG_FAST_EOF   0x3u
#define MAX30003_ETAG_EMPTY      0x6u
#define MAX30003_ETAG_OVERFLOW   0x7u

#define MAX30003_FIFO_DEPTH      32u

static inline uint32_t max30003_etag(uint32_t word) { return (word >> 3) & 0x7u; }

/* Sign-extend the left-justified 18-bit sample in word[23:6]. */
static inline int32_t max30003_sample(uint32_t word) { return ((int32_t)(word << 8)) >> 14; }

/* RTOR register -> interval between the last two R events, in ms. */
static inline uint32_t max30003_rtor_ms(uint32_t rtor_reg)
{
    return (((rtor_reg >> 10) & 0x3FFFu) * MAX30003_RTOR_LSB_US + 500u) / 1000u;
}

/* ---- Return codes ---- */
#define MAX30003_OK           0
#define MAX30003_ERR_SPI     (-1)   /* HAL SPI error or timeout */
#define MAX30003_ERR_ID      (-2)   /* INFO register does not identify a MAX30003: check wiring/power */
#define MAX30003_ERR_PLL     (-3)   /* PLL does not lock: check FCLK on D9 */

/* Keep the handle; no bus traffic. */
void max30003_attach(SPI_HandleTypeDef *hspi);

/* Software reset, identity check, full configuration from max30003_config.h,
 * SYNCH and FIFO reset. Leaves the device recording. Returns MAX30003_OK or
 * MAX30003_ERR_*; on MAX30003_ERR_PLL the device is configured but the data
 * is not usable until FCLK is present. */
int max30003_init(void);

int max30003_write(uint8_t reg, uint32_t data);
int max30003_read(uint8_t reg, uint32_t *data);

/* SYNCH: restart ECG recording and reset the FIFO. */
int max30003_synch(void);
/* FIFO_RST: empty the FIFO and clear EOVF, recording continues. */
int max30003_fifo_reset(void);

/* Burst-read FIFO words into words[0..n-1] until a word tagged EOF, EMPTY or
 * OVERFLOW has been read, or max words. Returns n (the terminating word is
 * included) or MAX30003_ERR_SPI. */
int max30003_read_fifo(uint32_t *words, size_t max);

/* INFO register as read by max30003_init() (0 before init). */
uint32_t max30003_info(void);

#endif
