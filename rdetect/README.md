# rdetect (C)

C port of the detector in `python/rdetect/detectors/energy.py`, written for the
STM32U385 but plain C99 with no dependencies beyond `libm`.

| File | Contents |
| --- | --- |
| `rdetect_config.h` | Every tunable parameter, as a macro. Edit before flashing. |
| `rdetect.h` | The API: `rdetect_init()` and `rdetect_detect()`. |
| `rdetect.c` | The detector. |

## Usage

```c
#include "rdetect.h"

static int32_t  ecg[RDETECT_MAX_SAMPLES];   /* filled elsewhere, e.g. by DMA */
static uint32_t peaks[RDETECT_MAX_PEAKS];

int count = rdetect_detect(ecg, n_samples, peaks, RDETECT_MAX_PEAKS);
/* count >= 0: peaks[0..count-1] are R-peak sample indices, ascending
   count <  0: one of the RDETECT_ERR_* codes in rdetect.h */
```

`rdetect_detect()` designs the Butterworth filters on its first call. That
takes some double-precision maths, which is done in software on the MCU; call
`rdetect_init()` at start-up to keep it out of a power measurement.

The ECG array is only read. The detector is not reentrant: its work buffers are
static.

## Configuration

`rdetect_config.h` mirrors the constants at the top of the Python module.
Differences in how they are written:

- Durations are in whole milliseconds (`RDETECT_ENERGY_SEGMENT_MS`,
  `RDETECT_SEARCH_AREA_MS`), so the sample counts are compile-time integers.
- `RDETECT_MAX_SAMPLES` sets the longest record and with it the RAM use.
- `RDETECT_SAMPLE_TYPE` is the element type of the input array (`int32_t` by default).
- `THRESHOLD_WINDOW` has no macro: `generate_adaptive_threshold()` currently
  returns the constant `ALPHA * mean`, so the window has no effect in Python either.
- The Teager-Kaiser energy and outlier removal options, both switched off in
  Python, are not ported.

Each macro can also be set from the build system, for example
`-DRDETECT_ALPHA=3.0`.

## Memory

With the default configuration (60 s at 200 Hz, `float`), built for the
Cortex-M33 with `-Os`: about 4 kB of code and 54 kB of static RAM, plus the
48 kB ECG array owned by the caller. RAM scales with `RDETECT_MAX_SAMPLES`.

## Adding it to the firmware

This is a copy of `c/rdetect` from the Prosjektoppgave repository. In
`CMakeLists.txt` of this project:

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    rdetect/rdetect.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    rdetect
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
)
```

## Tests

The tests (`python/tests/test_c_port.py`, `c/rdetect/tests/`) live in the
Prosjektoppgave repository. Keep this copy identical to `c/rdetect` there, or
copy it over again after a change.

## Known difference from Python

When two suspiciously short RR intervals are exactly equal, the false-peak
filter here examines them in index order. numpy's `argsort` does not define the
order of ties, so the two implementations could in principle remove a different
peak in that case.
