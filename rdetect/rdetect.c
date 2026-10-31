/*
 * rdetect.c - energy-based R-peak detector, C port of
 * python/rdetect/detectors/energy.py.
 *
 * Pipeline (same order as detect() in the Python module):
 *   1. zero-phase Butterworth filter of the ECG        (scipy butter + filtfilt)
 *   2. energy of non-overlapping segments              (calculate_energy)
 *   3. threshold = ALPHA * mean(moving average)        (moving_average, generate_adaptive_threshold)
 *   4. local maxima of the thresholded energy          (apply_threshold, detect_peaks)
 *   5. zero-phase high-pass of the raw ECG, strongest
 *      local maximum around each energy peak           (detect_peaks_in_windows)
 *   6. removal of peaks that break the RR rhythm       (filter_false_peaks)
 *
 * The Python options that are switched off there (Teager-Kaiser energy,
 * outlier removal) are not ported.
 */
#include "rdetect.h"

#include <math.h>
#include <string.h>

/* ---- Compile-time checks of rdetect_config.h ---- */

#if (RDETECT_ENERGY_SEGMENT_MS * RDETECT_FS_HZ) % 1000 != 0
#error "RDETECT_ENERGY_SEGMENT_MS must be a whole number of samples at RDETECT_FS_HZ"
#endif
#if RDETECT_SEGMENT_SAMPLES < 2
#error "RDETECT_ENERGY_SEGMENT_MS must cover at least two samples"
#endif
#if RDETECT_MOVING_AVERAGE_WINDOW < 1 || RDETECT_MOVING_AVERAGE_WINDOW % 2 == 0
#error "RDETECT_MOVING_AVERAGE_WINDOW must be a positive odd integer"
#endif
#if RDETECT_FILTER_ORDER < 1 || RDETECT_PEAK_HP_ORDER < 1
#error "filter orders must be at least 1"
#endif
#if RDETECT_FILTER_TYPE != RDETECT_FILTER_LOWPASS && \
    RDETECT_FILTER_TYPE != RDETECT_FILTER_HIGHPASS && \
    RDETECT_FILTER_TYPE != RDETECT_FILTER_BANDPASS
#error "RDETECT_FILTER_TYPE must be RDETECT_FILTER_LOWPASS, _HIGHPASS or _BANDPASS"
#endif
#if RDETECT_SHORT_RR_NUM < 1 || RDETECT_SHORT_RR_DEN < 1
#error "RDETECT_SHORT_RR_NUM and RDETECT_SHORT_RR_DEN must be positive"
#endif

/* ---- Derived sizes ---- */

#if RDETECT_USE_DOUBLE
typedef double real_t;
#else
typedef float real_t;
#endif

/* A band-pass of order N has 2N poles; scipy pads filtfilt with 3 * (number of coefficients). */
#if RDETECT_FILTER_TYPE == RDETECT_FILTER_BANDPASS
#define MAIN_SECTIONS RDETECT_FILTER_ORDER
#define MAIN_PADLEN   (3 * (2 * RDETECT_FILTER_ORDER + 1))
#define MAIN_F1_HZ    RDETECT_LOWER_CUTOFF_HZ
#define MAIN_F2_HZ    RDETECT_UPPER_CUTOFF_HZ
#else
#define MAIN_SECTIONS ((RDETECT_FILTER_ORDER + 1) / 2)
#define MAIN_PADLEN   (3 * (RDETECT_FILTER_ORDER + 1))
#define MAIN_F1_HZ    RDETECT_CUTOFF_HZ
#define MAIN_F2_HZ    0.0
#endif
#define HP_SECTIONS   ((RDETECT_PEAK_HP_ORDER + 1) / 2)
#define HP_PADLEN     (3 * (RDETECT_PEAK_HP_ORDER + 1))

#define MAX_SECTIONS  (MAIN_SECTIONS > HP_SECTIONS ? MAIN_SECTIONS : HP_SECTIONS)
#define MAX_PADLEN    (MAIN_PADLEN > HP_PADLEN ? MAIN_PADLEN : HP_PADLEN)

#define SEARCH_HALF_SAMPLES (((RDETECT_SEARCH_AREA_MS * RDETECT_FS_HZ) / 1000) / 2)

#define PI 3.14159265358979323846

/* One second-order section, transposed direct form II. zi0/zi1 are its state
 * for a constant input of 1, dc its gain at 0 Hz (scipy lfilter_zi). */
typedef struct {
    real_t b0, b1, b2, a1, a2;
    real_t zi0, zi1, dc;
} biquad_t;

typedef struct {
    biquad_t sec[MAX_SECTIONS];
    int count;
    int padlen;
} filter_t;

static filter_t main_filter, peak_hp_filter;
static int initialised;

static real_t  work[RDETECT_MAX_SAMPLES];               /* filtered ECG */
static real_t  energy[RDETECT_MAX_ENERGY_SEGMENTS];
static int32_t cand[RDETECT_MAX_PEAKS];                 /* peak candidates, sample indices */
static int32_t rr[RDETECT_MAX_PEAKS];                   /* intervals between candidates */
static int32_t rr_sorted[RDETECT_MAX_PEAKS];

/* ---- Butterworth design (runs once, in double) ---- */

typedef struct { double re, im; } cplx_t;

static cplx_t c_sqrt(cplx_t a)
{
    cplx_t out = { 0.0, 0.0 };
    double r = sqrt(a.re * a.re + a.im * a.im);
    if (r == 0.0) return out;
    if (a.re >= 0.0) {
        double w = sqrt(0.5 * (r + a.re));
        out.re = w;
        out.im = a.im / (2.0 * w);
    } else {
        double w = sqrt(0.5 * (r - a.re));
        out.re = fabs(a.im) / (2.0 * w);
        out.im = a.im >= 0.0 ? w : -w;
    }
    return out;
}

/* Bilinear transform s -> z at scipy's normalised sample rate fs = 2: z = (4 + s) / (4 - s). */
static cplx_t bilinear(cplx_t s)
{
    cplx_t z;
    double nr = 4.0 + s.re, dr = 4.0 - s.re;
    double den = dr * dr + s.im * s.im;
    z.re = (nr * dr - s.im * s.im) / den;
    z.im = (s.im * dr + nr * s.im) / den;
    return z;
}

static void add_section(filter_t *f, double b0, double b1, double b2, double a1, double a2)
{
    biquad_t *s = &f->sec[f->count++];
    double dc = (b0 + b1 + b2) / (1.0 + a1 + a2);
    s->b0 = (real_t)b0;
    s->b1 = (real_t)b1;
    s->b2 = (real_t)b2;
    s->a1 = (real_t)a1;
    s->a2 = (real_t)a2;
    s->zi1 = (real_t)(b2 - a2 * dc);
    s->zi0 = (real_t)(b1 - a1 * dc + b2 - a2 * dc);
    s->dc = (real_t)dc;
}

/* Adds the section with denominator 1 + a1 z^-1 + a2 z^-2, scaled to unit gain
 * in the passband. (c0, s0) = (cos, sin) of the band-pass centre frequency. */
static void add_biquad(filter_t *f, int type, double a1, double a2, double c0, double s0)
{
    double g;
    if (type == RDETECT_FILTER_LOWPASS) {
        g = (1.0 + a1 + a2) / 4.0;
        add_section(f, g, 2.0 * g, g, a1, a2);
    } else if (type == RDETECT_FILTER_HIGHPASS) {
        g = (1.0 - a1 + a2) / 4.0;
        add_section(f, g, -2.0 * g, g, a1, a2);
    } else {
        double re = 1.0 + a1 * c0 + a2 * (c0 * c0 - s0 * s0);
        double im = -(a1 * s0 + 2.0 * a2 * c0 * s0);
        g = sqrt(re * re + im * im) / (2.0 * s0);
        add_section(f, g, 0.0, -g, a1, a2);
    }
}

/* Same filter as scipy.signal.butter(order, ..., btype), as second-order sections. */
static int design_butterworth(filter_t *f, int type, int order, double f1_hz, double f2_hz)
{
    const double nyquist = 0.5 * RDETECT_FS_HZ;
    double w1, bw = 0.0, wo2 = 0.0, c0 = 0.0, s0 = 0.0;
    int k;

    f->count = 0;
    if (!(f1_hz > 0.0 && f1_hz < nyquist)) return RDETECT_ERR_CONFIG;
    if (type == RDETECT_FILTER_BANDPASS && !(f2_hz > f1_hz && f2_hz < nyquist)) return RDETECT_ERR_CONFIG;

    w1 = 4.0 * tan(PI * f1_hz / RDETECT_FS_HZ);                 /* pre-warped cutoff */
    if (type == RDETECT_FILTER_BANDPASS) {
        double w2 = 4.0 * tan(PI * f2_hz / RDETECT_FS_HZ);
        double t;
        bw = w2 - w1;
        wo2 = w1 * w2;
        t = sqrt(wo2) / 4.0;                                    /* tan(centre frequency / 2) */
        c0 = (1.0 - t * t) / (1.0 + t * t);
        s0 = 2.0 * t / (1.0 + t * t);
    }

    /* Upper-half-plane poles of the analog prototype; the last one is real when order is odd. */
    for (k = 0; k < (order + 1) / 2; k++) {
        const int is_real = (2 * k + 1 == order);
        double theta = PI * (2 * k + 1) / (2.0 * order);
        cplx_t p = { -sin(theta), cos(theta) };
        if (is_real) { p.re = -1.0; p.im = 0.0; }

        if (type != RDETECT_FILTER_BANDPASS) {
            /* The high-pass poles w1 / p are the same set as the low-pass poles w1 * p. */
            cplx_t s = { w1 * p.re, w1 * p.im };
            cplx_t z = bilinear(s);
            if (!is_real) {
                add_biquad(f, type, -2.0 * z.re, z.re * z.re + z.im * z.im, 0.0, 0.0);
            } else if (type == RDETECT_FILTER_LOWPASS) {
                double g = (1.0 - z.re) / 2.0;
                add_section(f, g, g, 0.0, -z.re, 0.0);
            } else {
                double g = (1.0 + z.re) / 2.0;
                add_section(f, g, -g, 0.0, -z.re, 0.0);
            }
        } else {
            /* Each prototype pole splits in two: pl +- sqrt(pl^2 - wo^2). */
            cplx_t pl = { 0.5 * bw * p.re, 0.5 * bw * p.im };
            cplx_t sq = { pl.re * pl.re - pl.im * pl.im - wo2, 2.0 * pl.re * pl.im };
            cplx_t d = c_sqrt(sq);
            cplx_t sa = { pl.re + d.re, pl.im + d.im };
            cplx_t sb = { pl.re - d.re, pl.im - d.im };
            cplx_t z1 = bilinear(sa), z2 = bilinear(sb);
            if (is_real) {
                add_biquad(f, type, -(z1.re + z2.re), z1.re * z2.re - z1.im * z2.im, c0, s0);
            } else {
                add_biquad(f, type, -2.0 * z1.re, z1.re * z1.re + z1.im * z1.im, c0, s0);
                add_biquad(f, type, -2.0 * z2.re, z2.re * z2.re + z2.im * z2.im, c0, s0);
            }
        }
    }

    f->padlen = (type == RDETECT_FILTER_BANDPASS) ? 3 * (2 * order + 1) : 3 * (order + 1);
    return RDETECT_OK;
}

int rdetect_init(void)
{
    int err;

    initialised = 0;
    err = design_butterworth(&main_filter, RDETECT_FILTER_TYPE, RDETECT_FILTER_ORDER,
                             MAIN_F1_HZ, MAIN_F2_HZ);
    if (err != RDETECT_OK) return err;
    err = design_butterworth(&peak_hp_filter, RDETECT_FILTER_HIGHPASS, RDETECT_PEAK_HP_ORDER,
                             RDETECT_PEAK_HP_CUTOFF_HZ, 0.0);
    if (err != RDETECT_OK) return err;
    initialised = 1;
    return RDETECT_OK;
}

/* ---- Zero-phase filtering (scipy.signal.filtfilt, default padding) ---- */

/* State as if the input had been the constant u forever. */
static void filter_settle(const filter_t *f, real_t st[][2], real_t u)
{
    int k;
    for (k = 0; k < f->count; k++) {
        st[k][0] = f->sec[k].zi0 * u;
        st[k][1] = f->sec[k].zi1 * u;
        u *= f->sec[k].dc;
    }
}

static real_t filter_step(const filter_t *f, real_t st[][2], real_t x)
{
    int k;
    for (k = 0; k < f->count; k++) {
        const biquad_t *s = &f->sec[k];
        real_t y = s->b0 * x + st[k][0];
        st[k][0] = s->b1 * x - s->a1 * y + st[k][1];
        st[k][1] = s->b2 * x - s->a2 * y;
        x = y;
    }
    return x;
}

/* y = filtfilt(x). Both ends are extended by padlen odd-reflected samples,
 * which are generated on the fly; only the end extension of the forward pass
 * has to be kept, as the start of the backward pass. Needs n > f->padlen. */
static void filtfilt(const filter_t *f, const rdetect_sample_t *x, size_t n, real_t *y)
{
    real_t st[MAX_SECTIONS][2];
    real_t tail[MAX_PADLEN];
    const int pad = f->padlen;
    const real_t first2 = (real_t)2 * (real_t)x[0];
    const real_t last2 = (real_t)2 * (real_t)x[n - 1];
    size_t i;
    int j;

    filter_settle(f, st, first2 - (real_t)x[pad]);
    for (j = pad; j >= 1; j--) (void)filter_step(f, st, first2 - (real_t)x[j]);
    for (i = 0; i < n; i++) y[i] = filter_step(f, st, (real_t)x[i]);
    for (j = 0; j < pad; j++) tail[j] = filter_step(f, st, last2 - (real_t)x[n - 2 - (size_t)j]);

    filter_settle(f, st, tail[pad - 1]);
    for (j = pad - 1; j >= 0; j--) (void)filter_step(f, st, tail[j]);
    for (i = n; i-- > 0;) y[i] = filter_step(f, st, y[i]);
}

/* ---- Peak search ---- */

/* scipy.signal.find_peaks without conditions: next strict local maximum of
 * x[0..n-1] at or after *pos (start with *pos = 1), or -1. A flat top counts
 * once, at its middle; the first and last sample are never peaks. */
static int32_t next_local_max(const real_t *x, int32_t n, int32_t *pos)
{
    int32_t i = *pos;
    while (i < n - 1) {
        if (x[i - 1] < x[i]) {
            int32_t ahead = i + 1;
            while (ahead < n - 1 && x[ahead] == x[i]) ahead++;
            if (x[ahead] < x[i]) {
                *pos = ahead + 1;
                return (i + ahead - 1) / 2;
            }
        }
        i++;
    }
    *pos = i;
    return -1;
}

/* ---- False-peak removal (filter_false_peaks) ----
 *
 * The Python code works on float medians that are whole or half sample counts.
 * Here every interval-like quantity is kept doubled, so it stays an integer.
 */

static void sort_ascending(int32_t *v, uint32_t n)
{
    uint32_t i, j;
    for (i = 1; i < n; i++) {
        int32_t key = v[i];
        for (j = i; j > 0 && v[j - 1] > key; j--) v[j] = v[j - 1];
        v[j] = key;
    }
}

static int64_t median_doubled(const int32_t *sorted, uint32_t n)
{
    if (n & 1u) return 2 * (int64_t)sorted[n / 2];
    return (int64_t)sorted[n / 2 - 1] + sorted[n / 2];
}

static int64_t abs64(int64_t a) { return a < 0 ? -a : a; }

/* interval < (NUM / DEN) * median */
static int is_short(int32_t interval, int64_t median2)
{
    return 2 * (int64_t)RDETECT_SHORT_RR_DEN * interval < (int64_t)RDETECT_SHORT_RR_NUM * median2;
}

/* Doubled sum of |interval - expected| once peak j is removed; base is that sum with all peaks. */
static int64_t removal_score(const int32_t *d, uint32_t nd, uint32_t j, int64_t expected2, int64_t base)
{
    if (j == 0) return base - abs64(2 * (int64_t)d[0] - expected2);
    if (j == nd) return base - abs64(2 * (int64_t)d[nd - 1] - expected2);
    return base - abs64(2 * (int64_t)d[j - 1] - expected2)
                - abs64(2 * (int64_t)d[j] - expected2)
                + abs64(2 * ((int64_t)d[j - 1] + d[j]) - expected2);
}

/* v: peak sample indices in ascending order. Returns the new count. */
static uint32_t remove_false_peaks(int32_t *v, uint32_t m)
{
    while (m >= 3) {
        const uint32_t nd = m - 1;
        uint32_t i, n_sorted = 0, n_short = 0, visited;
        int64_t median2, expected2, base = 0, last_key = -1;
        int32_t removal = -1;

        for (i = 0; i < nd; i++) {
            rr[i] = v[i + 1] - v[i];
            if (rr[i] > 0) rr_sorted[n_sorted++] = rr[i];
        }
        if (n_sorted == 0) break;
        sort_ascending(rr_sorted, n_sorted);
        median2 = median_doubled(rr_sorted, n_sorted);

        n_sorted = 0;
        for (i = 0; i < nd; i++) {
            if (is_short(rr[i], median2)) n_short++;
            else rr_sorted[n_sorted++] = rr[i];
        }
        if (n_short == 0 || n_sorted == 0) break;
        sort_ascending(rr_sorted, n_sorted);
        expected2 = median_doubled(rr_sorted, n_sorted);        /* typical RR interval */

        for (i = 0; i < nd; i++) base += abs64(2 * (int64_t)rr[i] - expected2);

        /* Shortest interval first; equal intervals in index order (numpy's
         * argsort leaves the order of ties unspecified). */
        for (visited = 0; visited < n_short && removal < 0; visited++) {
            int64_t best_key = -1, s0, s1, diff;
            uint32_t best = 0;
            for (i = 0; i < nd; i++) {
                int64_t key = (int64_t)rr[i] * m + i;
                if (is_short(rr[i], median2) && key > last_key && (best_key < 0 || key < best_key)) {
                    best_key = key;
                    best = i;
                }
            }
            last_key = best_key;

            /* One of the two peaks around the short interval goes: the one
             * whose removal leaves the more regular rhythm. */
            s0 = removal_score(rr, nd, best, expected2, base);
            s1 = removal_score(rr, nd, best + 1, expected2, base);
            diff = abs64(s0 - s1);
            if (diff != 0 && 100000 * diff > s1) {              /* not np.isclose(s0, s1) */
                removal = (int32_t)(s1 < s0 ? best + 1 : best);
            }
        }
        if (removal < 0) break;

        memmove(&v[removal], &v[removal + 1], (m - 1 - (uint32_t)removal) * sizeof v[0]);
        m--;
    }
    return m;
}

/* ---- Detector ---- */

int rdetect_detect(const rdetect_sample_t *ecg, size_t n, uint32_t *peaks, size_t max_peaks)
{
    const real_t inv_window = (real_t)1 / (real_t)RDETECT_MOVING_AVERAGE_WINDOW;
    const int32_t half_window = RDETECT_MOVING_AVERAGE_WINDOW / 2;
    int32_t n_seg, k, pos, peak, len = (int32_t)n;
    uint32_t count = 0, i;
    real_t smoothed_sum = 0, threshold;

    if (!initialised) {
        int err = rdetect_init();
        if (err != RDETECT_OK) return err;
    }
    if (n > RDETECT_MAX_SAMPLES || n <= (size_t)MAX_PADLEN) return RDETECT_ERR_LENGTH;

    /* 1-2. Filtered ECG -> energy per segment; a shorter last segment counts too. */
    filtfilt(&main_filter, ecg, n, work);
    n_seg = (len + RDETECT_SEGMENT_SAMPLES - 1) / RDETECT_SEGMENT_SAMPLES;
    if (n_seg < RDETECT_MOVING_AVERAGE_WINDOW) return RDETECT_ERR_LENGTH;
    for (k = 0; k < n_seg; k++) {
        int32_t start = k * RDETECT_SEGMENT_SAMPLES;
        int32_t end = start + RDETECT_SEGMENT_SAMPLES < len ? start + RDETECT_SEGMENT_SAMPLES : len;
        real_t sum = 0;
        for (; start < end; start++) sum += work[start] * work[start];
        energy[k] = sum;
    }

    /* 3. Only the mean of the moving average is needed (edges see zeros, np.convolve "same"). */
    for (k = 0; k < n_seg; k++) {
        int32_t lo = k - half_window < 0 ? 0 : k - half_window;
        int32_t hi = k + half_window > n_seg - 1 ? n_seg - 1 : k + half_window;
        real_t sum = 0;
        for (; lo <= hi; lo++) sum += energy[lo] * inv_window;
        smoothed_sum += sum;
    }
    threshold = (real_t)RDETECT_ALPHA * (smoothed_sum / (real_t)n_seg);

    /* 4. Energy peaks above the threshold -> centre sample of their segment. */
    for (k = 0; k < n_seg; k++) {
        if (!(energy[k] > threshold && energy[k] > 0)) energy[k] = 0;
    }
    pos = 1;
    while ((peak = next_local_max(energy, n_seg, &pos)) >= 0) {
        int32_t twice = 2 * peak * RDETECT_SEGMENT_SAMPLES + RDETECT_SEGMENT_SAMPLES;
        int32_t centre = twice / 2;
        if ((twice & 1) && (centre & 1)) centre++;              /* x.5 rounds to even, as Python round() */
        if (centre > len - 1) centre = len - 1;
        if (count >= RDETECT_MAX_PEAKS) return RDETECT_ERR_CAPACITY;
        cand[count++] = centre;
    }

    /* 5. Strongest local maximum of the high-passed ECG around each centre. */
    filtfilt(&peak_hp_filter, ecg, n, work);
    for (i = 0; i < count; i++) {
        int32_t start = cand[i] - SEARCH_HALF_SAMPLES < 0 ? 0 : cand[i] - SEARCH_HALF_SAMPLES;
        int32_t end = cand[i] + SEARCH_HALF_SAMPLES + 1 > len ? len : cand[i] + SEARCH_HALF_SAMPLES + 1;
        int32_t best = -1;
        pos = 1;
        while ((peak = next_local_max(&work[start], end - start, &pos)) >= 0) {
            if (best < 0 || work[start + peak] > work[start + best]) best = peak;
        }
        if (best >= 0) cand[i] = start + best;
    }

    /* 6. Overlapping windows can return peaks out of order or twice. */
    sort_ascending(cand, count);
    count = remove_false_peaks(cand, count);

    if (count > max_peaks) return RDETECT_ERR_CAPACITY;
    for (i = 0; i < count; i++) peaks[i] = (uint32_t)cand[i];
    return (int)count;
}
