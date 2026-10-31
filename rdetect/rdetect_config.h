/*
 * rdetect_config.h - tunable parameters of the energy R-peak detector.
 *
 * Mirrors the constants at the top of python/rdetect/detectors/energy.py.
 * Edit the values here before flashing, or override single ones from the
 * build system (-DRDETECT_ALPHA=3.0); every macro is guarded by #ifndef.
 */
#ifndef RDETECT_CONFIG_H
#define RDETECT_CONFIG_H

/* ---- Signal and memory ---- */

#ifndef RDETECT_FS_HZ
#define RDETECT_FS_HZ 200                   /* sample rate in Hz (integer) */
#endif

#ifndef RDETECT_MAX_SAMPLES
#define RDETECT_MAX_SAMPLES 12000               /* longest record rdetect_detect() accepts; sizes the
                                                    static work buffer (12000 = 60 s at 200 Hz) */
#endif

#ifndef RDETECT_SAMPLE_TYPE
#define RDETECT_SAMPLE_TYPE int32_t         /* element type of the ECG array handed to rdetect_detect() */
#endif

#ifndef RDETECT_USE_DOUBLE
#define RDETECT_USE_DOUBLE 0                /* 0: float arithmetic (uses the Cortex-M33 FPU)
                                               1: double arithmetic (software on the MCU, for PC checks) */
#endif

/* ---- Filter in front of the energy calculation ---- */

#define RDETECT_FILTER_LOWPASS  0
#define RDETECT_FILTER_HIGHPASS 1
#define RDETECT_FILTER_BANDPASS 2

#ifndef RDETECT_FILTER_TYPE
#define RDETECT_FILTER_TYPE RDETECT_FILTER_BANDPASS
#endif

#ifndef RDETECT_CUTOFF_HZ
#define RDETECT_CUTOFF_HZ 5.0               /* used by the low-pass and high-pass types */
#endif

#ifndef RDETECT_LOWER_CUTOFF_HZ
#define RDETECT_LOWER_CUTOFF_HZ 6.0         /* used by the band-pass type */
#endif

#ifndef RDETECT_UPPER_CUTOFF_HZ
#define RDETECT_UPPER_CUTOFF_HZ 36.0        /* used by the band-pass type */
#endif

#ifndef RDETECT_FILTER_ORDER
#define RDETECT_FILTER_ORDER 4              /* Butterworth order, as passed to scipy.signal.butter */
#endif

/* ---- Energy and threshold ---- */

#ifndef RDETECT_ENERGY_SEGMENT_MS
#define RDETECT_ENERGY_SEGMENT_MS 100       /* avg QRS length; must be a whole number of samples */
#endif

#ifndef RDETECT_MOVING_AVERAGE_WINDOW
#define RDETECT_MOVING_AVERAGE_WINDOW 3     /* in energy segments, must be odd */
#endif

#ifndef RDETECT_ALPHA
#define RDETECT_ALPHA 2.0                   /* threshold = ALPHA * mean(smoothed energy) */
#endif

/* ---- R-peak search around each energy peak ---- */

#ifndef RDETECT_SEARCH_AREA_MS
#define RDETECT_SEARCH_AREA_MS 150          /* 2.5 x average width of the QRS complex */
#endif

#ifndef RDETECT_PEAK_HP_CUTOFF_HZ
#define RDETECT_PEAK_HP_CUTOFF_HZ 5.0       /* high-pass applied to the raw ECG before the search */
#endif

#ifndef RDETECT_PEAK_HP_ORDER
#define RDETECT_PEAK_HP_ORDER 4
#endif

/* ---- False-peak removal ---- */

/* An RR interval shorter than NUM/DEN of the median interval is suspicious. */
#ifndef RDETECT_SHORT_RR_NUM
#define RDETECT_SHORT_RR_NUM 3
#endif

#ifndef RDETECT_SHORT_RR_DEN
#define RDETECT_SHORT_RR_DEN 4
#endif

#endif
