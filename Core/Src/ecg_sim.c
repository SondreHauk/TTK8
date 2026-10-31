/*
 * ecg_sim.c - synthetic ECG on DAC1 OUT1 (PA4), see ecg_sim.h.
 */
#include "ecg_sim.h"

#include <math.h>

/* One wave of the beat: centre relative to the R peak, width (sigma) and
 * height relative to R. Shapes after McSharry et al. (ECGSYN), simplified. */
typedef struct { float t_ms, sigma_ms, a; } wave_t;

static const wave_t waves[] = {
    { -200.0f, 25.0f,  0.15f },   /* P */
    {  -30.0f, 10.0f, -0.10f },   /* Q */
    {    0.0f, 12.0f,  1.00f },   /* R: ~30 ms at half height, most energy at 10-30 Hz */
    {   30.0f, 10.0f, -0.25f },   /* S */
    {  260.0f, 50.0f,  0.30f },   /* T */
};

/* Offsets from the baseline in DAC codes, one per ms of the beat. */
static int16_t beat[ECG_SIM_MAX_PERIOD];
static volatile uint32_t period;      /* ms per beat = entries used in beat[] */
static volatile uint32_t pos;
static uint32_t baseline_code;
static uint32_t cur_bpm;
static DAC_HandleTypeDef *dac;
static TIM_HandleTypeDef *tim;

#define REC_LEN 256u                  /* power of two */
static volatile uint32_t rec_decim, rec_count;
static volatile int32_t  rec_queue[REC_LEN];
static volatile uint32_t rec_head, rec_tail, rec_dropped;
static volatile uint32_t ticks;

static uint32_t mv_to_code(float mv)
{
    float c = mv * 4096.0f / (float)ECG_SIM_VREF_MV;
    if (c < 0.0f) c = 0.0f;
    if (c > 4095.0f) c = 4095.0f;
    return (uint32_t)(c + 0.5f);
}

/* Fill beat[0..n-1] with the R peak at 40 % of the period. Neighbouring beats
 * are added too, so the T wave of one beat runs into the P wave of the next
 * at high rates instead of being cut off. */
static void build_beat(uint32_t n)
{
    const float r_mv = (float)ECG_SIM_R_UV / 1000.0f;
    const float codes_per_mv = 4096.0f / (float)ECG_SIM_VREF_MV;
    const float t_r = 0.4f * (float)n;

    for (uint32_t i = 0; i < n; i++) {
        float mv = 0.0f;
        for (int k = -1; k <= 1; k++) {
            float t = (float)i - t_r + (float)k * (float)n;
            for (unsigned w = 0; w < sizeof waves / sizeof waves[0]; w++) {
                float d = (t - waves[w].t_ms) / waves[w].sigma_ms;
                if (d > -6.0f && d < 6.0f) mv += waves[w].a * r_mv * expf(-0.5f * d * d);
            }
        }
        beat[i] = (int16_t)lrintf(mv * codes_per_mv);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (tim == NULL || htim->Instance != tim->Instance) return;
    int32_t code = (int32_t)baseline_code + beat[pos];
    if (code < 0) code = 0;
    if (code > 4095) code = 4095;
    HAL_DAC_SetValue(dac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)code);
    if (++pos >= period) pos = 0;
    ticks++;

    if (rec_decim != 0u && ++rec_count >= rec_decim) {
        uint32_t next = (rec_head + 1u) & (REC_LEN - 1u);
        rec_count = 0;
        if (next == rec_tail) {
            rec_dropped++;
        } else {
            rec_queue[rec_head] = (int32_t)(((int64_t)(code - (int32_t)baseline_code)
                                             * (int64_t)ECG_SIM_VREF_MV * 1000) / 4096);
            rec_head = next;
        }
    }
}

void ecg_sim_record(uint32_t decim)
{
    __disable_irq();
    rec_decim = decim;
    rec_count = 0;
    rec_head = rec_tail = 0;
    rec_dropped = 0;
    __enable_irq();
}

int ecg_sim_pop(int32_t *uv)
{
    uint32_t tail = rec_tail;
    if (tail == rec_head) return 0;
    *uv = rec_queue[tail];
    rec_tail = (tail + 1u) & (REC_LEN - 1u);
    return 1;
}

uint32_t ecg_sim_dropped(void) { return rec_dropped; }
uint32_t ecg_sim_ticks(void)   { return ticks; }

int ecg_sim_set_bpm(uint32_t bpm)
{
    uint32_t n;

    if (bpm < 30u || bpm > 240u) return -1;
    n = (60u * ECG_SIM_RATE_HZ + bpm / 2u) / bpm;

    if (tim != NULL) HAL_NVIC_DisableIRQ(TIM6_IRQn);
    build_beat(n);
    period = n;
    pos = 0;
    cur_bpm = bpm;
    if (tim != NULL) HAL_NVIC_EnableIRQ(TIM6_IRQn);
    return 0;
}

uint32_t ecg_sim_bpm(void) { return cur_bpm; }

int ecg_sim_start(DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim, uint32_t bpm)
{
#if ECG_SIM_ENABLE
    dac = hdac;
    baseline_code = mv_to_code((float)ECG_SIM_BASELINE_MV);
    if (ecg_sim_set_bpm(bpm) != 0) return -1;

    if (HAL_DAC_SetValue(dac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, baseline_code) != HAL_OK) return -1;
    if (HAL_DAC_Start(dac, DAC_CHANNEL_1) != HAL_OK) return -1;

    tim = htim;
    if (HAL_TIM_Base_Start_IT(tim) != HAL_OK) return -1;
    return 0;
#else
    (void)hdac; (void)htim; (void)bpm;
    return 0;
#endif
}
