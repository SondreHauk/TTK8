#ifndef TTK8_TEST_H
#define TTK8_TEST_H

/*
 * ttk8_test - loopback test of the MAX30003 test board, tests 2, 3, 5 and 6
 * of table 2 in the TTK8 report. Built when CMake option TTK8_LOOPBACK is on.
 *
 * ecg_sim plays a synthetic ECG on PA4 (A2 -> ASIG), the MAX30003 reads it
 * back (JP1 on ASIG, JP2 on 0V9), and both signals go to the PC over the
 * ST-LINK VCP for TTK8_DURATION_S seconds. The detector then runs on the
 * received ECG and the peaks and heart rate follow. tools/ttk8_loopback.py
 * in the Prosjektoppgave records and plots a run.
 */

#include "main.h"

#ifndef TTK8_BPM
#define TTK8_BPM            50u       /* heart rate of the synthetic ECG */
#endif

#ifndef TTK8_DURATION_S
#define TTK8_DURATION_S     30u       /* recorded length in seconds */
#endif

#ifndef TTK8_SETTLE_MS
#define TTK8_SETTLE_MS      2000u     /* run before recording, while the filters settle */
#endif

/* Configures the MAX30003, starts ecg_sim, then runs the test once and again
 * on every USER press. Never returns. FCLK (TIM3 CH1) must already run. */
void ttk8_run(SPI_HandleTypeDef *hspi, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim);

#endif
