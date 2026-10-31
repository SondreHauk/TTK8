#ifndef ECG_STREAM_H
#define ECG_STREAM_H

/*
 * ecg_stream - paced playback of the ECG and the heart rate to a serial
 * plotter, one line per sample:
 *
 *     ecg:<uV>,hr:<bpm>,r:<0|1>
 *
 *   ecg  ECG in microvolts at the MAX30003 input
 *   hr   beat-to-beat heart rate, 60 * fs / RR, updated at each R-peak found
 *        by rdetect and held until the next one (0 before the first interval
 *        and after ECG_STREAM_HR_TIMEOUT_S without a beat)
 *   r    1 on the sample of an R-peak, else 0
 *
 * The detector only sees whole blocks, so the stream runs a fixed
 * ECG_STREAM_DELAY_SAMPLES behind acquisition. Playback is locked to the
 * sample counter of ecg_acq (not to SysTick, which is stopped in sleep), so
 * lines leave at exactly the acquisition rate, in groups of EFIT samples.
 * Output is queued with uart_tx_try_write() and never blocks.
 */

#include <stddef.h>
#include <stdint.h>

#include "ecg_acq.h"

#ifndef ECG_STREAM_MARGIN_SAMPLES
#define ECG_STREAM_MARGIN_SAMPLES  ECG_ACQ_FS_HZ        /* 1 s for rdetect to finish a block */
#endif
#define ECG_STREAM_DELAY_SAMPLES   (ECG_ACQ_BLOCK_SAMPLES + ECG_STREAM_MARGIN_SAMPLES)
#ifndef ECG_STREAM_HR_TIMEOUT_S
#define ECG_STREAM_HR_TIMEOUT_S    3u
#endif

/* Forget everything queued; the next block starts a new stream. */
void ecg_stream_reset(void);

/* Queue a detected block: n raw MAX30003 samples starting at running index
 * first, and the count R-peaks (indices within the block) rdetect found. */
void ecg_stream_block(uint32_t first, const int32_t *ecg, int n,
                      const uint32_t *peaks, int count);

/* Send every queued sample that is at least ECG_STREAM_DELAY_SAMPLES older
 * than acquired (ecg_acq_count()), as far as the UART ring has room. */
void ecg_stream_pump(uint32_t acquired);

/* Samples dropped from playback since the last reset (ring full or a block
 * that was dropped by ecg_acq). */
uint32_t ecg_stream_dropped(void);

#endif
