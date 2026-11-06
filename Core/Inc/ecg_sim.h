#ifndef ECG_SIM_H
#define ECG_SIM_H

/*
 * ecg_sim - synthetic ECG on DAC1 OUT1 (PA4, Arduino A2 = ASIG on the
 * MAX30003 test board), for testing the R-peak detector without electrodes.
 *
 * With JP1 on ASIG and JP2 on 0V9, the MAX30003 measures
 *   ECGP - ECGN = DAC - 0.9 V,
 * so the waveform sits on a 0.9 V baseline (ratiometric to 3V3, like the
 * R3/R4 divider that makes 0V9) and is only a few mV high: at the default
 * 20 V/V gain the MAX30003 accepts +-32.5 mV.
 *
 * One beat is a sum of five Gaussians (P, Q, R, S, T), built once into a
 * table and played back one sample per TIM6 update (1 kHz).
 *
 * CubeMX: DAC1 OUT1 "Connected to external pin only", buffer on, trigger
 * none; TIM6 prescaler 95, period 999 (1 kHz), TIM6 global interrupt on.
 */

#include <stdint.h>

#include "main.h"

#ifndef ECG_SIM_ENABLE
#define ECG_SIM_ENABLE        1u      /* 0: DAC stays off, PA4 untouched */
#endif
#ifndef ECG_SIM_BPM
#define ECG_SIM_BPM           60u     /* heart rate; 30..240 */
#endif
#ifndef ECG_SIM_R_UV
#define ECG_SIM_R_UV          10000u  /* R-wave height in microvolts (10 mV = ~12 DAC codes) */
#endif
#ifndef ECG_SIM_BASELINE_MV
#define ECG_SIM_BASELINE_MV   900u    /* must match the 0V9 reference on ECGN */
#endif
#ifndef ECG_SIM_VREF_MV
#define ECG_SIM_VREF_MV       3300u   /* DAC reference (VREF+ = VDDA = 3V3 on the Nucleo) */
#endif

#define ECG_SIM_RATE_HZ       1000u   /* TIM6 update rate */
#define ECG_SIM_MAX_PERIOD    (60u * ECG_SIM_RATE_HZ / 30u)   /* table size: down to 30 bpm */

/* Build the beat table, start the DAC on 'baseline' and TIM6 with its
 * update interrupt. Returns 0, or -1 on a HAL error / bad bpm. */
int ecg_sim_start(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t bpm);

/* Change the heart rate while running (rebuilds the table). */
int ecg_sim_set_bpm(uint32_t bpm);
uint32_t ecg_sim_bpm(void);


#endif
