/*
 * max30003_config.h - settings of the MAX30003 ECG front end.
 *
 * Edit the values here before flashing, or override single ones from the
 * build system (-DMAX30003_ECG_GAIN=1); every macro is guarded by #ifndef.
 * Register and bit names refer to the MAX30003 datasheet (Rev 3).
 */
#ifndef MAX30003_CONFIG_H
#define MAX30003_CONFIG_H

/* ---- Sample rate ----
 * CNFG_GEN.FMSTR = 10 with CNFG_ECG.RATE = 10 gives 200 sps from the
 * 32.768 kHz FCLK, which is the rate rdetect is tuned for (RDETECT_FS_HZ).
 * Other combinations are listed in datasheet Table 22. */
#ifndef MAX30003_FMSTR
#define MAX30003_FMSTR            2u     /* 0b10: fMSTR = 32000 Hz, 200 Hz progressions */
#endif
#ifndef MAX30003_RATE
#define MAX30003_RATE             2u     /* 0b10: 200 sps with FMSTR = 0b10 */
#endif
#ifndef MAX30003_FS_HZ
#define MAX30003_FS_HZ            200u   /* must match FMSTR/RATE above */
#endif
#ifndef MAX30003_RTOR_LSB_US
#define MAX30003_RTOR_LSB_US      8000u  /* RTOR resolution: 8.000 ms for FMSTR = 01/10 (Table 22) */
#endif

/* ---- ECG channel (CNFG_ECG) ---- */
#ifndef MAX30003_ECG_GAIN
#define MAX30003_ECG_GAIN         0u     /* 0: 20 V/V, 1: 40 V/V, 2: 80 V/V, 3: 160 V/V */
#endif
#ifndef MAX30003_ECG_DHPF
#define MAX30003_ECG_DHPF         1u     /* 0: bypass (DC), 1: 0.5 Hz digital high-pass */
#endif
#ifndef MAX30003_ECG_DLPF
#define MAX30003_ECG_DLPF         1u     /* 0: bypass, 1: 40 Hz digital low-pass (only option at 200 sps) */
#endif
#ifndef MAX30003_ECG_INVERT
#define MAX30003_ECG_INVERT       0u     /* CNFG_EMUX.POL: 1 inverts the ECG */
#endif

/* ---- Lead bias (CNFG_GEN) ----
 * Resistive bias keeps electrode inputs inside the common-mode range. It is
 * harmless when the inputs are driven by the DAC through the JP1/JP2 jumpers. */
#ifndef MAX30003_RBIAS_ENABLE
#define MAX30003_RBIAS_ENABLE     1u
#endif
#ifndef MAX30003_RBIAS_VALUE
#define MAX30003_RBIAS_VALUE      1u     /* 0: 50 MOhm, 1: 100 MOhm, 2: 200 MOhm */
#endif

/* ---- DC lead-off detection (CNFG_GEN), reported through STATUS ---- */
#ifndef MAX30003_DCLOFF_ENABLE
#define MAX30003_DCLOFF_ENABLE    0u
#endif
#ifndef MAX30003_DCLOFF_IMAG
#define MAX30003_DCLOFF_IMAG      2u     /* 0: off, 1: 5 nA, 2: 10 nA, 3: 20 nA, 4: 50 nA, 5: 100 nA */
#endif
#ifndef MAX30003_DCLOFF_VTH
#define MAX30003_DCLOFF_VTH       0u     /* 0: VMID +-300 mV ... 3: VMID +-500 mV */
#endif

/* ---- FIFO interrupt ----
 * INTB goes low when EFIT samples are waiting. The FIFO holds 32 samples, so
 * 16 leaves 80 ms of margin at 200 sps before an overflow. */
#ifndef MAX30003_EFIT
#define MAX30003_EFIT             16u    /* 1..32 */
#endif

/* ---- Built-in R-to-R detector on INT2B ----
 * The MAX30003 has its own R-peak detector. With this on, every R event pulls
 * INT2B low and the firmware prints the interval it measured, for comparison
 * with rdetect. It does not influence rdetect. */
#ifndef MAX30003_RTOR_ENABLE
#define MAX30003_RTOR_ENABLE      1u
#endif

/* ---- Internal calibration source ----
 * 1: disconnect ECGP/ECGN and feed the channel a +-0.5 mV, ~1 Hz square wave
 * from VCALP/VCALN. Useful to check SPI, FCLK and the data path before an
 * analogue signal is connected. rdetect sees two edges per period. */
#ifndef MAX30003_USE_CAL_SIGNAL
#define MAX30003_USE_CAL_SIGNAL   0u
#endif

#if MAX30003_EFIT < 1 || MAX30003_EFIT > 32
#error "MAX30003_EFIT must be 1..32"
#endif

#endif
