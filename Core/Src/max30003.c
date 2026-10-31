/*
 * max30003.c - register-level driver for the MAX30003, see max30003.h.
 */
#include "max30003.h"

#define SPI_TIMEOUT_MS   5u
#define CMD_READ(reg)    ((uint8_t)(((reg) << 1) | 1u))
#define CMD_WRITE(reg)   ((uint8_t)((reg) << 1))

#define PLL_LOCK_TIMEOUT_MS 500u

static SPI_HandleTypeDef *spi;
static uint32_t info_reg;

static inline void cs_low(void)  { HAL_GPIO_WritePin(MAX_CSB_GPIO_Port, MAX_CSB_Pin, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(MAX_CSB_GPIO_Port, MAX_CSB_Pin, GPIO_PIN_SET); }

static int xfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    return HAL_SPI_TransmitReceive(spi, tx, rx, n, SPI_TIMEOUT_MS) == HAL_OK ? MAX30003_OK : MAX30003_ERR_SPI;
}

void max30003_attach(SPI_HandleTypeDef *hspi)
{
    spi = hspi;
    cs_high();
}

int max30003_write(uint8_t reg, uint32_t data)
{
    uint8_t tx[4] = { CMD_WRITE(reg), (uint8_t)(data >> 16), (uint8_t)(data >> 8), (uint8_t)data };
    uint8_t rx[4];
    int err;

    cs_low();
    err = xfer(tx, rx, sizeof tx);
    cs_high();
    return err;
}

int max30003_read(uint8_t reg, uint32_t *data)
{
    uint8_t tx[4] = { CMD_READ(reg), 0, 0, 0 };
    uint8_t rx[4] = { 0 };
    int err;

    cs_low();
    err = xfer(tx, rx, sizeof tx);
    cs_high();
    if (err == MAX30003_OK && data != NULL) {
        *data = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
    }
    return err;
}

int max30003_synch(void)      { return max30003_write(MAX30003_REG_SYNCH, 0); }
int max30003_fifo_reset(void) { return max30003_write(MAX30003_REG_FIFO_RST, 0); }
uint32_t max30003_info(void)  { return info_reg; }

int max30003_read_fifo(uint32_t *words, size_t max)
{
    static const uint8_t zeros[3] = { 0, 0, 0 };
    const uint8_t cmd = CMD_READ(MAX30003_REG_ECG_FIFO_BURST);
    uint8_t dummy, rx[3];
    size_t n = 0;
    int err;

    if (max == 0u) return 0;

    /* Burst mode: after the command byte, every further 24 SCLKs return the
       next FIFO word for as long as CSB stays low (datasheet Figure 9). The
       SPI may pause between words; SCLK idles low with KeepIOState on. */
    cs_low();
    err = xfer(&cmd, &dummy, 1);
    while (err == MAX30003_OK && n < max) {
        uint32_t w, tag;
        err = xfer(zeros, rx, 3);
        if (err != MAX30003_OK) break;
        w = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
        words[n++] = w;
        tag = max30003_etag(w);
        if (tag != MAX30003_ETAG_VALID && tag != MAX30003_ETAG_FAST) break;  /* EOF, empty or overflow */
    }
    cs_high();
    return err == MAX30003_OK ? (int)n : err;
}

/* ---- Register values built from max30003_config.h ---- */

static uint32_t cnfg_gen(void)
{
    uint32_t v = 0;
    v |= (uint32_t)(MAX30003_FMSTR & 0x3u) << 20;                 /* FMSTR */
    v |= 1uL << 19;                                                /* EN_ECG */
#if MAX30003_DCLOFF_ENABLE
    v |= 1uL << 12;                                                /* EN_DCLOFF = 01 */
    v |= (uint32_t)(MAX30003_DCLOFF_IMAG & 0x7u) << 8;
    v |= (uint32_t)(MAX30003_DCLOFF_VTH & 0x3u) << 6;
#endif
#if MAX30003_RBIAS_ENABLE
    v |= 1uL << 4;                                                 /* EN_RBIAS = 01 */
    v |= (uint32_t)(MAX30003_RBIAS_VALUE & 0x3u) << 2;            /* RBIASV */
    v |= (1uL << 1) | 1uL;                                         /* RBIASP, RBIASN */
#else
    v |= 1uL << 2;                                                 /* RBIASV reset value */
#endif
    return v;
}

static uint32_t cnfg_ecg(void)
{
    return ((uint32_t)(MAX30003_RATE & 0x3u) << 22)
         | ((uint32_t)(MAX30003_ECG_GAIN & 0x3u) << 16)
         | ((uint32_t)(MAX30003_ECG_DHPF & 0x1u) << 14)
         | ((uint32_t)(MAX30003_ECG_DLPF & 0x3u) << 12);
}

static uint32_t cnfg_emux(void)
{
    uint32_t v = (uint32_t)(MAX30003_ECG_INVERT & 0x1u) << 23;
#if MAX30003_USE_CAL_SIGNAL
    v |= (1uL << 21) | (1uL << 20);                                /* OPENP, OPENN: inputs isolated */
    v |= (2uL << 18) | (3uL << 16);                                /* ECGP <- VCALP, ECGN <- VCALN */
#endif
    return v;                                                      /* OPENP = OPENN = 0: inputs connected */
}

static uint32_t cnfg_cal(void)
{
    uint32_t v = (4uL << 12) | (1uL << 11);                        /* FCAL = FMSTR/2^15 (~1 Hz), 50 % duty */
#if MAX30003_USE_CAL_SIGNAL
    v |= (1uL << 22) | (1uL << 21) | (1uL << 20);                  /* EN_VCAL, bipolar, 0.5 mV */
#endif
    return v;
}

static uint32_t mngr_int(void)
{
    return ((uint32_t)(MAX30003_EFIT - 1u) << 19)                  /* EFIT */
         | (1uL << 4)                                              /* CLR_RRINT = 01: clear on RTOR read */
         | (1uL << 2);                                             /* CLR_SAMP = 1 (reset value) */
}

static uint32_t cnfg_rtor1(void)
{
    uint32_t v = (0x3uL << 20)                                     /* WNDW = 12 x 8 ms = 96 ms */
               | (0xFuL << 16)                                     /* GAIN: auto-scale */
               | (0x2uL << 12)                                     /* PAVG = 8 */
               | (0x3uL << 8);                                     /* PTSF = 4/16 */
#if MAX30003_RTOR_ENABLE
    v |= 1uL << 15;                                                /* EN_RTOR */
#endif
    return v;
}

int max30003_init(void)
{
    uint32_t status = 0;
    uint32_t t0;
    int err;

    if (spi == NULL) return MAX30003_ERR_SPI;
    cs_high();

    if ((err = max30003_write(MAX30003_REG_SW_RST, 0)) != MAX30003_OK) return err;
    HAL_Delay(2);

    /* INFO is not valid as the first command after a reset (datasheet INFO). */
    (void)max30003_read(MAX30003_REG_NO_OP, NULL);
    if ((err = max30003_read(MAX30003_REG_INFO, &info_reg)) != MAX30003_OK) return err;
    if (((info_reg >> 20) & 0xFu) != 0x5u) return MAX30003_ERR_ID;   /* D[23:20] = 0101 */

    if ((err = max30003_write(MAX30003_REG_CNFG_GEN,   cnfg_gen()))   != MAX30003_OK) return err;
    if ((err = max30003_write(MAX30003_REG_CNFG_CAL,   cnfg_cal()))   != MAX30003_OK) return err;
    if ((err = max30003_write(MAX30003_REG_CNFG_EMUX,  cnfg_emux()))  != MAX30003_OK) return err;
    if ((err = max30003_write(MAX30003_REG_CNFG_ECG,   cnfg_ecg()))   != MAX30003_OK) return err;
    if ((err = max30003_write(MAX30003_REG_CNFG_RTOR1, cnfg_rtor1())) != MAX30003_OK) return err;
    if ((err = max30003_write(MAX30003_REG_MNGR_INT,   mngr_int()))   != MAX30003_OK) return err;
    if ((err = max30003_write(MAX30003_REG_MNGR_DYN,   0x3FuL << 16)) != MAX30003_OK) return err; /* fast recovery off */

    /* INTB: FIFO threshold and overflow. INT2B: R events. Both push-pull. */
    if ((err = max30003_write(MAX30003_REG_EN_INT,
                              MAX30003_EINT | MAX30003_EOVF | MAX30003_INTB_TYPE_CMOS)) != MAX30003_OK) return err;
#if MAX30003_RTOR_ENABLE
    if ((err = max30003_write(MAX30003_REG_EN_INT2, MAX30003_RRINT | MAX30003_INTB_TYPE_CMOS)) != MAX30003_OK) return err;
#else
    if ((err = max30003_write(MAX30003_REG_EN_INT2, MAX30003_INTB_TYPE_CMOS)) != MAX30003_OK) return err;
#endif

    if ((err = max30003_synch()) != MAX30003_OK) return err;

    /* PLLINT is latched until STATUS is read, so lock is confirmed by a read
       that comes back clear. */
    t0 = HAL_GetTick();
    do {
        HAL_Delay(10);
        if ((err = max30003_read(MAX30003_REG_STATUS, &status)) != MAX30003_OK) return err;
        if ((status & MAX30003_PLLINT) == 0u) break;
    } while ((HAL_GetTick() - t0) < PLL_LOCK_TIMEOUT_MS);

    /* Discard whatever was recorded while the PLL settled. */
    if ((err = max30003_synch()) != MAX30003_OK) return err;
    return (status & MAX30003_PLLINT) ? MAX30003_ERR_PLL : MAX30003_OK;
}
