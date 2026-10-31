#ifndef ECG_ACQ_H
#define ECG_ACQ_H

/*
 * ecg_acq - continuous ECG acquisition from the MAX30003 in fixed-length
 * blocks, for block-wise R-peak detection.
 *
 * INTB (FIFO threshold) and INT2B (R event) are serviced in their EXTI
 * interrupts, so samples keep coming in while the main loop runs the detector
 * on the previous block. Two block buffers are used in turn.
 */

#include <stddef.h>
#include <stdint.h>

#include "max30003.h"

#ifndef ECG_ACQ_BLOCK_SECONDS
#define ECG_ACQ_BLOCK_SECONDS  10u        /* detector block length; at most RDETECT_MAX_SAMPLES / fs */
#endif
#define ECG_ACQ_FS_HZ          MAX30003_FS_HZ
#define ECG_ACQ_BLOCK_SAMPLES  (ECG_ACQ_BLOCK_SECONDS * ECG_ACQ_FS_HZ)

/* Error flags returned by ecg_acq_take_errors() */
#define ECG_ACQ_ERR_SPI        (1u << 0)  /* SPI transfer failed */
#define ECG_ACQ_ERR_OVERFLOW   (1u << 1)  /* MAX30003 FIFO overflowed: samples lost, FIFO reset */
#define ECG_ACQ_ERR_OVERRUN    (1u << 2)  /* a block was full before the previous one was released: block dropped */

/* Call once after MX_SPI1_Init() and after FCLK (TIM3 CH1) is running.
 * Returns MAX30003_OK or a MAX30003_ERR_* code; acquisition runs on
 * MAX30003_OK and MAX30003_ERR_PLL. */
int ecg_acq_init(SPI_HandleTypeDef *hspi);

/* 0: no new block.
 * > 0: a block is ready; *samples points to that many samples and
 *      *first_index is the running index of its first sample (0 at start,
 *      ECG_ACQ_FS_HZ per second). The buffer stays valid and untouched until
 *      ecg_acq_release() is called, which must happen within one block time. */
int ecg_acq_poll(const int32_t **samples, uint32_t *first_index);
void ecg_acq_release(void);

/* 1 and *rr_ms set if the MAX30003's own R-to-R detector reported a new
 * interval since the last call, else 0. */
int ecg_acq_rtor(uint32_t *rr_ms);

/* Error flags (ECG_ACQ_ERR_*) since the last call, then cleared. */
uint32_t ecg_acq_take_errors(void);

/* Drop the partly filled block, SYNCH the MAX30003 and start again at index 0. */
void ecg_acq_restart(void);

/* Stop acquisition: the INTB/INT2B interrupts no longer touch the MAX30003
 * (no SPI from interrupt context), and ecg_acq_poll() returns 0. Use it
 * around long printf output. The MAX30003 keeps sampling into its FIFO. */
void ecg_acq_stop(void);

/* Start again after ecg_acq_stop(): as ecg_acq_restart(), and any block held
 * by the main loop is given up. */
void ecg_acq_start(void);

/* Sleep (WFI) until the next interrupt unless there is something to collect.
 * TIM3 must keep FCLK running, so Stop modes are not used. */
void ecg_acq_sleep(void);

#endif
