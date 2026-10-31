/*
 * ecg_stream.c - paced ECG + heart rate output for a serial plotter, see
 * ecg_stream.h. Runs in the main loop only; no interrupt touches it.
 */
#include "ecg_stream.h"

#include <stdio.h>

#include "max30003_config.h"
#include "uart_tx.h"

/* Room for the rest of the previous block plus a whole new one. */
#define RING_SAMPLES   (2u * ECG_ACQ_BLOCK_SAMPLES)
#define PEAK_QUEUE     64u                       /* power of two, > peaks in RING_SAMPLES */
#define LINE_MAX       40u

#if ECG_STREAM_MARGIN_SAMPLES > ECG_ACQ_BLOCK_SAMPLES
#error "ECG_STREAM_MARGIN_SAMPLES must not exceed one block"
#endif

/* ECG LSB = VREF / (2^17 * gain), VREF = 1 V (MAX30003 datasheet). */
#define ECG_GAIN       (20 << MAX30003_ECG_GAIN)
#define UV_DIV         (131072LL * ECG_GAIN)

static int32_t  ring[RING_SAMPLES];              /* microvolts */
static uint32_t ring_first;                      /* running index of ring[ring_rd] */
static uint32_t ring_rd;
static uint32_t ring_n;

static uint32_t peak_q[PEAK_QUEUE];              /* running indices, ascending */
static uint32_t peak_rd, peak_n;

static int      started;
static uint32_t next_first;                      /* expected first index of the next block */
static uint32_t last_peak;
static int      have_last_peak;
static uint32_t hr_x10;
static uint32_t dropped;

static int32_t to_uv(int32_t v)
{
    int64_t x = (int64_t)v * 1000000LL;
    return (int32_t)((x >= 0 ? x + UV_DIV / 2 : x - UV_DIV / 2) / UV_DIV);
}

void ecg_stream_reset(void)
{
    ring_first = ring_rd = ring_n = 0;
    peak_rd = peak_n = 0;
    started = 0;
    next_first = 0;
    have_last_peak = 0;
    hr_x10 = 0;
    dropped = 0;
}

static void drop_oldest(uint32_t k)
{
    if (k > ring_n) k = ring_n;
    ring_rd = (ring_rd + k) % RING_SAMPLES;
    ring_first += k;
    ring_n -= k;
    dropped += k;
    while (peak_n > 0u && (int32_t)(peak_q[peak_rd] - ring_first) < 0) {
        peak_rd = (peak_rd + 1u) & (PEAK_QUEUE - 1u);
        peak_n--;
    }
}

void ecg_stream_block(uint32_t first, const int32_t *ecg, int n,
                      const uint32_t *peaks, int count)
{
    if (n <= 0) return;

    if (!started || first != next_first) {
        /* Start, or ecg_acq dropped a block: playback restarts at this block
           and no RR interval is taken across the gap. */
        if (started) dropped += ring_n + (first - next_first);
        started = 1;
        ring_first = first;
        ring_rd = ring_n = 0;
        peak_rd = peak_n = 0;
        have_last_peak = 0;
        hr_x10 = 0;
    }
    next_first = first + (uint32_t)n;

    if (ring_n + (uint32_t)n > RING_SAMPLES) drop_oldest(ring_n + (uint32_t)n - RING_SAMPLES);
    for (int i = 0; i < n; i++) {
        ring[(ring_rd + ring_n) % RING_SAMPLES] = to_uv(ecg[i]);
        ring_n++;
    }
    for (int i = 0; i < count; i++) {
        if (peak_n == PEAK_QUEUE) break;         /* cannot happen with sane ECG */
        peak_q[(peak_rd + peak_n) & (PEAK_QUEUE - 1u)] = first + peaks[i];
        peak_n++;
    }
}

void ecg_stream_pump(uint32_t acquired)
{
    char line[LINE_MAX];

    while (ring_n > 0u && (int32_t)(acquired - ring_first) >= (int32_t)ECG_STREAM_DELAY_SAMPLES) {
        uint32_t idx = ring_first;
        int r = 0;

        if (uart_tx_free() < LINE_MAX) return;   /* UART busy: try again next wake-up */

        if (peak_n > 0u && peak_q[peak_rd] == idx) {
            r = 1;
            if (have_last_peak) {
                uint32_t rr = idx - last_peak;
                hr_x10 = (600u * ECG_ACQ_FS_HZ + rr / 2u) / rr;
            }
            last_peak = idx;
            have_last_peak = 1;
            peak_rd = (peak_rd + 1u) & (PEAK_QUEUE - 1u);
            peak_n--;
        } else if (have_last_peak && idx - last_peak > ECG_STREAM_HR_TIMEOUT_S * ECG_ACQ_FS_HZ) {
            hr_x10 = 0;                          /* no beat for a while: lead off or asystole */
        }

        int len = snprintf(line, sizeof line, "ecg:%ld,hr:%lu.%lu,r:%d\r\n",
                           (long)ring[ring_rd], (unsigned long)(hr_x10 / 10u),
                           (unsigned long)(hr_x10 % 10u), r);
        if (len > 0) (void)uart_tx_try_write(line, (size_t)len);

        ring_rd = (ring_rd + 1u) % RING_SAMPLES;
        ring_first++;
        ring_n--;
    }
}

uint32_t ecg_stream_dropped(void)
{
    return dropped;
}
