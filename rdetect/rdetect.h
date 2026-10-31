/*
 * rdetect.h - energy-based R-peak detector, C port of
 * python/rdetect/detectors/energy.py.
 *
 * Usage:
 *   static uint32_t peaks[RDETECT_MAX_PEAKS];
 *   int count = rdetect_detect(ecg, n, peaks, RDETECT_MAX_PEAKS);
 *
 * The detector keeps its work buffers in static storage: no malloc, but also
 * not reentrant. Parameters live in rdetect_config.h.
 */
#ifndef RDETECT_H
#define RDETECT_H

#include <stddef.h>
#include <stdint.h>

#include "rdetect_config.h"

typedef RDETECT_SAMPLE_TYPE rdetect_sample_t;

#define RDETECT_OK            0
#define RDETECT_ERR_CONFIG   (-1)   /* cutoff frequencies outside 0..fs/2 */
#define RDETECT_ERR_LENGTH   (-2)   /* record too short for the filters or longer than RDETECT_MAX_SAMPLES */
#define RDETECT_ERR_CAPACITY (-3)   /* more peaks than the output array holds */

#define RDETECT_SEGMENT_SAMPLES     ((RDETECT_ENERGY_SEGMENT_MS * RDETECT_FS_HZ) / 1000)
#define RDETECT_MAX_ENERGY_SEGMENTS ((RDETECT_MAX_SAMPLES + RDETECT_SEGMENT_SAMPLES - 1) / RDETECT_SEGMENT_SAMPLES)

/* Upper bound on the peaks one record can produce: an output array of this size never overflows. */
#define RDETECT_MAX_PEAKS           (RDETECT_MAX_ENERGY_SEGMENTS / 2 + 1)

/* Designs the filters from the config macros. Optional: rdetect_detect() calls
 * it on first use. Call it yourself to keep that one-off cost out of a
 * measurement. Returns RDETECT_OK or RDETECT_ERR_CONFIG. */
int rdetect_init(void);

/* Writes the sample indices of the detected R-peaks, in ascending order, to
 * peaks[0..count-1]. Returns count, or a negative RDETECT_ERR_* code. */
int rdetect_detect(const rdetect_sample_t *ecg, size_t n, uint32_t *peaks, size_t max_peaks);

#endif
