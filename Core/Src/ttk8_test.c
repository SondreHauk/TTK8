/*
 * ttk8_test.c - TTK8 loopback test, see ttk8_test.h.
 *
 * Output, one record per line, comma separated (read by tools/ttk8_loopback.py):
 *   #TTK8,1               start of a run (format version 1)
 *   I,<key>,<value>       settings and the MAX30003 INFO register
 *   S                     streaming starts
 *   G,<n>,<uV>            generated sample n at the ECG rate, microvolt
 *                         relative to the 0.9 V baseline, as quantised by the DAC
 *   E,<n>,<counts>        received sample n, signed 18-bit MAX30003 code
 *   P,<index>             detected R-peak, index into the E samples
 *   H,<bpm>               estimated heart rate, mean over all RR intervals
 *   X,<key>,<value>       counters (acquisition errors, timing, ...)
 *   ERR,<text>            the run was aborted
 *   END                   end of the run
 *
 * The MAX30003 is read through ecg_acq: INTB falls when EFIT samples wait in
 * the FIFO, and the EXTI interrupt burst-reads them into ecg_acq's blocks.
 * Both signals are captured into RAM first and sent after the capture, so
 * the UART cannot hold up the readout. X,capture_ms and X,dac_ticks give the
 * real duration of the capture and the number of DAC updates in it (both
 * should be TTK8_DURATION_S * 1000).
 */
#include <stdio.h>
#include <string.h>

#include "ttk8_test.h"
#include "ecg_acq.h"
#include "ecg_sim.h"
#include "max30003.h"
#include "rdetect.h"

/* Only built for the loopback test, so that the normal application can be
   built with ECG_SIM_ENABLE 0 (real electrodes). */
#ifdef TTK8_LOOPBACK

#if MAX30003_FS_HZ != RDETECT_FS_HZ
#error "MAX30003 sample rate (max30003_config.h) and RDETECT_FS_HZ (rdetect_config.h) differ"
#endif
#if TTK8_DURATION_S * MAX30003_FS_HZ > RDETECT_MAX_SAMPLES
#error "TTK8_DURATION_S is longer than rdetect accepts (RDETECT_MAX_SAMPLES)"
#endif
#if TTK8_DURATION_S % ECG_ACQ_BLOCK_SECONDS != 0
#error "TTK8_DURATION_S must be a multiple of ECG_ACQ_BLOCK_SECONDS"
#endif
#if !ECG_SIM_ENABLE
#error "The TTK8 test needs ECG_SIM_ENABLE"
#endif

#define N_SAMPLES        (TTK8_DURATION_S * MAX30003_FS_HZ)
#define SIM_DECIM        (ECG_SIM_RATE_HZ / MAX30003_FS_HZ)
#define GEN_TAIL_WAIT_MS 100u
#define CAPTURE_SLACK_MS 5000u        /* give up this long after the capture should have ended */

extern __IO uint32_t BspButtonState;

static int32_t  ecg[N_SAMPLES];
static int32_t  gen[N_SAMPLES];
static uint32_t peaks[RDETECT_MAX_PEAKS];

static void drain_generated(uint32_t *n_gen)
{
    int32_t uv;
    while (*n_gen < N_SAMPLES && ecg_sim_pop(&uv))
    {
        gen[(*n_gen)++] = uv;
    }
}

static void run_once(SPI_HandleTypeDef *hspi)
{
    const int32_t *block;
    uint32_t first, errors = 0, blocks = 0;
    uint32_t n_rx = 0, n_gen = 0, t0, t_start, t_end, tick_start, tick_end;

    printf("#TTK8,1\r\n");
    printf("I,fs,%lu\r\n", (unsigned long)MAX30003_FS_HZ);
    printf("I,bpm,%lu\r\n", (unsigned long)ecg_sim_bpm());
    printf("I,samples,%lu\r\n", (unsigned long)N_SAMPLES);
    printf("I,gain,%lu\r\n", 20ul << MAX30003_ECG_GAIN);
    printf("I,r_amp_uv,%lu\r\n", (unsigned long)ECG_SIM_R_UV);
    printf("I,baseline_mv,%lu\r\n", (unsigned long)ECG_SIM_BASELINE_MV);

    /* Criterion 6: register access. ecg_acq_init configures the MAX30003
       with max30003_init and starts the INTB-driven acquisition. */
    int rc = ecg_acq_init(hspi);
    ecg_acq_stop();                      /* no FIFO reads (SPI in interrupt) while printing */
    printf("I,info,0x%06lx\r\n", (unsigned long)max30003_info());
    if (rc == MAX30003_ERR_PLL)
    {
        printf("# PLL not locked: check FCLK on D9 (PC6)\r\n");
    }
    else if (rc != MAX30003_OK)
    {
        printf("ERR,max30003_init %d\r\n", rc);
        printf("END\r\n");
        return;
    }
    printf("I,regs,ok\r\n");

    /* No printf from here until the capture is over. */
    ecg_acq_start();

    /* Let the AFE settle on the running signal; throw that part away. */
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TTK8_SETTLE_MS)
    {
        if (ecg_acq_poll(&block, &first) > 0) ecg_acq_release();
    }

    /* Start both records at the same instant: ecg_acq_restart synchronises
       the MAX30003 and starts a new block at sample 0. */
    __disable_irq();
    ecg_acq_restart();
    ecg_sim_record(SIM_DECIM);
    tick_start = ecg_sim_ticks();
    __enable_irq();
    (void)ecg_acq_take_errors();
    t_start = HAL_GetTick();

    while (n_rx < N_SAMPLES)
    {
        drain_generated(&n_gen);
        errors |= ecg_acq_take_errors();
        if ((HAL_GetTick() - t_start) > TTK8_DURATION_S * 1000u + CAPTURE_SLACK_MS)
        {
            ecg_acq_stop();
            printf("ERR,capture timeout after %lu samples\r\n", (unsigned long)n_rx);
            break;
        }
        int n = ecg_acq_poll(&block, &first);
        if (n <= 0) continue;
        if (first != n_rx) errors |= ECG_ACQ_ERR_OVERRUN;   /* a block was dropped */
        memcpy(&ecg[n_rx], block, (size_t)n * sizeof ecg[0]);
        n_rx += (uint32_t)n;
        blocks++;
        ecg_acq_release();
    }
    t_end = HAL_GetTick();
    tick_end = ecg_sim_ticks();
    ecg_acq_stop();                      /* off until the next run */

    /* DAC (TIM6) and FCLK (TIM3) run off the same clock, so the generated
       record is at most a sample or two behind. */
    t0 = HAL_GetTick();
    while (n_gen < N_SAMPLES && (HAL_GetTick() - t0) < GEN_TAIL_WAIT_MS)
    {
        drain_generated(&n_gen);
    }
    ecg_sim_record(0);

    printf("S\r\n");
    for (uint32_t i = 0; i < N_SAMPLES; i++)
    {
        if (i < n_gen) printf("G,%lu,%ld\r\n", (unsigned long)i, (long)gen[i]);
        if (i < n_rx) printf("E,%lu,%ld\r\n", (unsigned long)i, (long)ecg[i]);
    }

    /* Criterion 2: heart rate from the received ECG. */
    int count = rdetect_detect(ecg, n_rx, peaks, RDETECT_MAX_PEAKS);
    if (count < 0)
    {
        printf("ERR,rdetect %d\r\n", count);
    }
    else
    {
        for (int i = 0; i < count; i++)
        {
            printf("P,%lu\r\n", (unsigned long)peaks[i]);
        }
        if (count >= 2 && peaks[count - 1] > peaks[0])
        {
            uint64_t span = peaks[count - 1] - peaks[0];
            uint32_t centi = (uint32_t)((60ull * 100ull * MAX30003_FS_HZ * (uint64_t)(count - 1) + span / 2u) / span);
            printf("H,%lu.%02lu\r\n", (unsigned long)(centi / 100u), (unsigned long)(centi % 100u));
        }
        else
        {
            printf("ERR,too few peaks for a heart rate (%d)\r\n", count);
        }
    }

    printf("X,overflow,%u\r\n", (errors & ECG_ACQ_ERR_OVERFLOW) ? 1u : 0u);
    printf("X,overrun,%u\r\n", (errors & ECG_ACQ_ERR_OVERRUN) ? 1u : 0u);
    printf("X,spi_errors,%u\r\n", (errors & ECG_ACQ_ERR_SPI) ? 1u : 0u);
    printf("X,gen_dropped,%lu\r\n", (unsigned long)ecg_sim_dropped());
    printf("X,gen_samples,%lu\r\n", (unsigned long)n_gen);
    printf("X,capture_ms,%lu\r\n", (unsigned long)(t_end - t_start));
    printf("X,dac_ticks,%lu\r\n", (unsigned long)(tick_end - tick_start));
    printf("X,rx_samples,%lu\r\n", (unsigned long)n_rx);
    printf("X,blocks,%lu\r\n", (unsigned long)blocks);
    printf("END\r\n");
}

void ttk8_run(SPI_HandleTypeDef *hspi, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim)
{
    if (ecg_sim_start(hdac, htim, TTK8_BPM) != 0)
    {
        printf("ERR,ecg_sim_start\r\nEND\r\n");
    }

    for (;;)
    {
        BspButtonState = BUTTON_RELEASED;
        run_once(hspi);

        printf("# press USER to run again\r\n");
        while (BspButtonState != BUTTON_PRESSED)
        {
            __WFI();
        }
    }
}

#endif /* TTK8_LOOPBACK */
