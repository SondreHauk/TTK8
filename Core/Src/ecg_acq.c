/*
 * ecg_acq.c - continuous ECG acquisition from the MAX30003, see ecg_acq.h.
 *
 * Signal path (MAX30003 test board on the Arduino headers):
 *   D9  PC6  TIM3_CH1  -> FCLK   32.768 kHz master clock (TIM3 PWM, 96 MHz / 2930)
 *   D10 PC9  GPIO      -> CSB
 *   D11 PA7  SPI1_MOSI -> SDI
 *   D12 PA6  SPI1_MISO <- SDO
 *   D13 PA5  SPI1_SCK  -> SCLK
 *   D3  PB3  EXTI3     <- INTB   FIFO threshold (EFIT) / FIFO overflow
 *   D2  PC8  EXTI8     <- INT2B  R event from the built-in R-to-R detector
 *
 * INTB is serviced in its interrupt: the FIFO is burst-read until the
 * end-of-file tag, which also releases INTB. Samples go into one of two block
 * buffers; a full block is handed to the main loop while the other fills.
 *
 * Fast-recovery samples (ETAG FAST) are time-valid but not voltage-valid; they
 * are replaced by the last valid sample so that sample indices stay true time.
 */
#include "ecg_acq.h"

static int32_t block_buf[2][ECG_ACQ_BLOCK_SAMPLES];

static volatile uint8_t  fill_buf;           /* buffer the interrupt is filling */
static volatile uint32_t fill_n;             /* samples in it */
static volatile uint32_t fill_first;         /* running index of its first sample */
static volatile int8_t   ready_buf = -1;     /* full block not yet collected by ecg_acq_poll() */
static volatile uint32_t ready_first;
static volatile int8_t   held_buf = -1;      /* block the main loop is working on */

static volatile uint32_t errors;
static volatile uint32_t rtor_ms;
static volatile uint8_t  rtor_new;
static volatile uint8_t  running;
static int32_t last_value;

/* ---- Bus ownership: the main loop masks both MAX30003 interrupts while it
        talks to the device, so a transaction is never cut in two. ---- */

static void lock(void)
{
    HAL_NVIC_DisableIRQ(MAX_INTB_EXTI_IRQn);
    HAL_NVIC_DisableIRQ(MAX_INT2B_EXTI_IRQn);
    __DSB();
    __ISB();
}

static void unlock(void)
{
    HAL_NVIC_EnableIRQ(MAX_INTB_EXTI_IRQn);
    HAL_NVIC_EnableIRQ(MAX_INT2B_EXTI_IRQn);
}

/* ---- Interrupt side ---- */

static void push_sample(int32_t v)
{
    block_buf[fill_buf][fill_n++] = v;
    if (fill_n < ECG_ACQ_BLOCK_SAMPLES) return;

    uint8_t other = fill_buf ^ 1u;
    if (ready_buf != (int8_t)other && held_buf != (int8_t)other) {
        ready_buf   = (int8_t)fill_buf;
        ready_first = fill_first;
        fill_buf    = other;
    } else {
        errors |= ECG_ACQ_ERR_OVERRUN;       /* main loop too slow: drop this block */
    }
    fill_first += ECG_ACQ_BLOCK_SAMPLES;
    fill_n = 0;
}

static void service_intb(void)
{
    uint32_t words[MAX30003_FIFO_DEPTH + 1u];
    int n = max30003_read_fifo(words, MAX30003_FIFO_DEPTH + 1u);

    if (n < 0) {
        errors |= ECG_ACQ_ERR_SPI;
        return;
    }
    for (int i = 0; i < n; i++) {
        switch (max30003_etag(words[i])) {
        case MAX30003_ETAG_VALID:
        case MAX30003_ETAG_VALID_EOF:
            last_value = max30003_sample(words[i]);
            push_sample(last_value);
            break;
        case MAX30003_ETAG_FAST:
        case MAX30003_ETAG_FAST_EOF:
            push_sample(last_value);
            break;
        case MAX30003_ETAG_OVERFLOW:
            /* EOVF keeps INTB low until the FIFO is reset (datasheet STATUS). */
            errors |= ECG_ACQ_ERR_OVERFLOW;
            if (max30003_fifo_reset() != MAX30003_OK) errors |= ECG_ACQ_ERR_SPI;
            break;
        default:                             /* EMPTY: nothing (more) to read */
            break;
        }
    }
}

static void service_int2b(void)
{
    uint32_t reg;
    /* Reading RTOR also clears RRINT (MNGR_INT.CLR_RRINT = 01). */
    if (max30003_read(MAX30003_REG_RTOR, &reg) != MAX30003_OK) {
        errors |= ECG_ACQ_ERR_SPI;
        return;
    }
    rtor_ms  = max30003_rtor_ms(reg);
    rtor_new = 1;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (!running) return;                    /* picked up by ecg_acq_poll() once running */
    if (GPIO_Pin == MAX_INTB_Pin) {
        service_intb();
    } else if (GPIO_Pin == MAX_INT2B_Pin) {
        service_int2b();
    }
}

/* ---- Main loop side ---- */

static int intb_low(void)  { return HAL_GPIO_ReadPin(MAX_INTB_GPIO_Port, MAX_INTB_Pin) == GPIO_PIN_RESET; }
static int int2b_low(void) { return HAL_GPIO_ReadPin(MAX_INT2B_GPIO_Port, MAX_INT2B_Pin) == GPIO_PIN_RESET; }

int ecg_acq_init(SPI_HandleTypeDef *hspi)
{
    int err;

    lock();
    running = 0;
    max30003_attach(hspi);
    err = max30003_init();
    fill_buf = 0;
    fill_n = 0;
    fill_first = 0;
    ready_buf = -1;
    held_buf = -1;
    errors = 0;
    rtor_new = 0;
    last_value = 0;
    running = (err == MAX30003_OK || err == MAX30003_ERR_PLL);
    unlock();
    return err;
}

int ecg_acq_poll(const int32_t **samples, uint32_t *first_index)
{
    int n = 0;

    if (!running) return 0;

    /* The interrupts are edge-triggered. If a line is already low (an edge
       came before acquisition started, or while the bus was locked), no new
       edge will come until it is serviced, so do that here. */
    if (intb_low() || int2b_low()) {
        lock();
        if (intb_low())  service_intb();
        if (int2b_low()) service_int2b();
        unlock();
    }

    __disable_irq();
    if (ready_buf >= 0 && held_buf < 0) {
        held_buf     = ready_buf;
        ready_buf    = -1;
        *samples     = block_buf[held_buf];
        *first_index = ready_first;
        n = (int)ECG_ACQ_BLOCK_SAMPLES;
    }
    __enable_irq();
    return n;
}

uint32_t ecg_acq_count(void)
{
    __disable_irq();
    uint32_t c = fill_first + fill_n;
    __enable_irq();
    return c;
}

void ecg_acq_release(void)
{
    held_buf = -1;
}

int ecg_acq_rtor(uint32_t *rr_ms)
{
    int got = 0;
    __disable_irq();
    if (rtor_new) {
        *rr_ms = rtor_ms;
        rtor_new = 0;
        got = 1;
    }
    __enable_irq();
    return got;
}

uint32_t ecg_acq_take_errors(void)
{
    __disable_irq();
    uint32_t e = errors;
    errors = 0;
    __enable_irq();
    return e;
}

void ecg_acq_restart(void)
{
    lock();
    if (max30003_synch() != MAX30003_OK) errors |= ECG_ACQ_ERR_SPI;
    fill_n = 0;
    fill_first = 0;
    ready_buf = -1;
    last_value = 0;
    rtor_new = 0;
    unlock();
}

void ecg_acq_stop(void)
{
    lock();
    running = 0;
    unlock();
}

void ecg_acq_start(void)
{
    lock();
    if (max30003_synch() != MAX30003_OK) errors |= ECG_ACQ_ERR_SPI;
    fill_n = 0;
    fill_first = 0;
    ready_buf = -1;
    held_buf = -1;
    last_value = 0;
    rtor_new = 0;
    running = 1;
    unlock();
}

void ecg_acq_sleep(void)
{
    /* Interrupts are masked between the checks and WFI so that an event
       arriving right here is not slept through: a pending interrupt still
       ends WFI, and runs once they are unmasked. */
    __disable_irq();
    if (ready_buf >= 0 || rtor_new || errors || intb_low() || int2b_low()) {
        __enable_irq();
        return;
    }
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    __enable_irq();
}
