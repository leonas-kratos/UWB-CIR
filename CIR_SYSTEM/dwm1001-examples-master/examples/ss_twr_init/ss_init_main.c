// ============================================================================
// FIXED TDMA CODE - Synchronized 4-anchor ranging
// Collects ALL 4 anchors before printing
// Fixed time slots for predictable timing
// ============================================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#define APP_NAME "SS TWR INIT v2.0 - PURE TDMA"

#define TIME_SLOT_MS          2
#define RESPONSE_TIMEOUT_MS   5
#define CYCLE_DELAY_MS        3
#define MAX_RETRIES_PER_CYCLE 5

/* Device IDs */
#define MY_INITIATOR_DEVICE_ID 0x5678
#define ANCHOR_1 0x1001
#define ANCHOR_2 0x1002
#define ANCHOR_3 0x1003
#define ANCHOR_4 0x1004
#define ANCHOR_5 0x1005
#define ANCHOR_6 0x1006
#define ANCHOR_7 0x1007
#define ANCHOR_8 0x1008

#define NUM_ANCHORS 8

/* CIR storage */
#define CIR_SAMPLES_PER_ANCHOR 20

typedef struct {
    int16_t real;
    int16_t img;
} cir_sample_t;

typedef struct {
    float kurtosis;    // excess kurtosis: LOS cao, NLOS thap
    float skewness;    // do lech phan phoi CIR
    int   peak_count;  // so dinh cuc bo vuot nguong
} cir_multipath_t;

typedef struct {
    dwt_rxdiag_t    diagnostics;
    cir_sample_t    cir_samples[CIR_SAMPLES_PER_ANCHOR];
    uint8_t         valid;
    double          distance;
    cir_multipath_t multipath;
    /* Thêm mới */
    uint16_t        cir_pwr;
    uint16_t        fp_index;
    uint32_t        frame_len;
} anchor_data_t;

/* Global storage */
static anchor_data_t anchor_data[NUM_ANCHORS];

/* Global arrays */
static uint32 anchor_ids[NUM_ANCHORS] = {ANCHOR_1, ANCHOR_2, ANCHOR_3, ANCHOR_4, ANCHOR_5, ANCHOR_6, ANCHOR_7, ANCHOR_8};

/* Message frames */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'I', 'O', 'V', 'E', 0xE0, 0, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'I', 'O', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Message field definitions */
#define ALL_MSG_COMMON_LEN       10
#define ALL_MSG_SN_IDX            2
#define POLL_MSG_DEVICE_ID_IDX   10
#define POLL_MSG_DEVICE_ID_LEN    4
#define RESP_MSG_POLL_RX_TS_IDX  10
#define RESP_MSG_RESP_TX_TS_IDX  14
#define RESP_MSG_TS_LEN           4
#define RESP_MSG_DEVICE_ID_IDX   18
#define RESP_MSG_DEVICE_ID_LEN    4

static uint8 frame_seq_nb = 0;

/* RX buffer */
#define RX_BUF_LEN 24
static uint8  rx_buffer[RX_BUF_LEN];
static uint32 status_reg = 0;

/* Constants */
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT  299702547

/* Performance counters */
static volatile int tx_count          = 0;
static volatile int rx_count          = 0;
static volatile int measurement_cycle = 0;
static volatile int timeout_count     = 0;
static volatile int skip_count        = 0;
static uint8_t      ml_header_printed = 0;
static volatile int complete_cycles   = 0;
static volatile int incomplete_cycles = 0;

/* extern config from main.c */
extern dwt_config_t config;

// ============================================================================
// HELPER: power calculations
// ============================================================================

float approx_log10(uint32_t x)
{
    uint8_t log2 = 0;
    while (x >>= 1) ++log2;
    return log2 * 0.30103f;
}

uint32_t sqrt_uint32(uint32_t x)
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;

    while (bit > x)
        bit >>= 2;

    while (bit != 0)
    {
        if (x >= res + bit)
        {
            x -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* sqrt float dung Newton (khong dung math.h) */
static float sqrt_float(float x)
{
    if (x <= 0.0f) return 0.0f;
    float s = x * 0.5f;
    for (int i = 0; i < 16; i++) s = (s + x / s) * 0.5f;
    return s;
}

int8_t fPathPWR(dwt_rxdiag_t diag)
{
    uint32_t sum = diag.firstPathAmp1 * diag.firstPathAmp1 +
                   diag.firstPathAmp2 * diag.firstPathAmp2 +
                   diag.firstPathAmp3 * diag.firstPathAmp3;

    float log_sum = approx_log10(sum);
    float log_N2  = approx_log10(diag.rxPreamCount) * 2.0f;
    float power_dbm = 10.0f * (log_sum - log_N2) - 121.74f;

    return (int8_t)(power_dbm + 0.5f);
}

uint16_t readCIR_PWR_fast()
{
    uint32_t fqual;
    dwt_readfromdevice(0x12, 4, 4, (uint8_t*)&fqual);
    uint16_t cir_pwr = (fqual >> 16) & 0xFFFF;
    return cir_pwr;
}

int8_t RX_PWR(dwt_rxdiag_t diag)
{
    uint32_t C = readCIR_PWR_fast();
    uint32_t N = diag.rxPreamCount;

    if (C == 0 || N == 0) return 0;

    float logC  = approx_log10(C) + (17.0f * 0.30103f);
    float logN2 = approx_log10(N) * 2.0f;
    float power_dbm = 10.0f * (logC - logN2) - 121.74f;

    return (int8_t)(power_dbm + 0.5f);
}

#define CHUNK_SIZE  16

static void read_cir_samples_fast(cir_sample_t *samples, int fp_index)
{
    int start_idx = fp_index - 2;
    if (start_idx < 0) start_idx = 0;

    int max_idx = 1016 - CIR_SAMPLES_PER_ANCHOR;
    if (start_idx > max_idx) start_idx = max_idx;

    uint8_t buffer[CHUNK_SIZE * 4 + 1];
    int samples_read = 0;

    while (samples_read < CIR_SAMPLES_PER_ANCHOR)
    {
        int remaining  = CIR_SAMPLES_PER_ANCHOR - samples_read;
        int this_chunk = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

        int byte_offset = (start_idx + samples_read) * 4;

        dwt_readaccdata(buffer, this_chunk * 4 + 1, byte_offset);

        for (int i = 0; i < this_chunk; i++)
        {
            int offset = i * 4 + 1;
            samples[samples_read + i].real = (int16_t)(buffer[offset]     | (buffer[offset + 1] << 8));
            samples[samples_read + i].img  = (int16_t)(buffer[offset + 2] | (buffer[offset + 3] << 8));
        }

        samples_read += this_chunk;
    }
}

// ============================================================================
// MULTIPATH FEATURES: kurtosis, skewness, peak_count
// ============================================================================

static cir_multipath_t compute_multipath_features(cir_sample_t *samples, int n)
{
    cir_multipath_t mp = {0.0f, 0.0f, 0};

    float mag[CIR_SAMPLES_PER_ANCHOR];
    float mean = 0.0f;

    for (int i = 0; i < n; i++)
    {
        int32_t r  = samples[i].real;
        int32_t im = samples[i].img;
        mag[i] = (float)sqrt_uint32((uint32_t)(r * r + im * im));
        mean += mag[i];
    }
    mean /= (float)n;

    float m2 = 0.0f, m3 = 0.0f, m4 = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float d  = mag[i] - mean;
        float d2 = d * d;
        m2 += d2;
        m3 += d2 * d;
        m4 += d2 * d2;
    }
    m2 /= (float)n;
    m3 /= (float)n;
    m4 /= (float)n;

    float std = sqrt_float(m2);

    mp.kurtosis = (m2 > 0.0f) ? (m4 / (m2 * m2)) - 3.0f : 0.0f;
    mp.skewness = (std > 0.0f) ? (m3 / (std * std * std)) : 0.0f;

    float threshold = mean + 0.5f * std;
    mp.peak_count = 0;

    for (int i = 1; i < n - 1; i++)
    {
        if (mag[i] > mag[i - 1] && mag[i] > mag[i + 1] && mag[i] > threshold)
            mp.peak_count++;
    }

    return mp;
}

// ============================================================================
// CAPTURE diagnostics + CIR + multipath features
// ============================================================================

static void capture_anchor_diagnostics(uint8_t anchor_idx)
{
    anchor_data_t *data = &anchor_data[anchor_idx];

    dwt_readdiagnostics(&data->diagnostics);

    /* Đọc CIR_PWR ngay sau diagnostics, trước khi clear RX */
    data->cir_pwr  = readCIR_PWR_fast();
    data->fp_index = data->diagnostics.firstPath >> 6;

    read_cir_samples_fast(data->cir_samples, data->fp_index);
    data->multipath = compute_multipath_features(data->cir_samples, CIR_SAMPLES_PER_ANCHOR);

    data->valid = 1;
}

// ============================================================================
// PRINT: in theo format CSV chuẩn
// NLOS,RANGE,FP_IDX,FP_AMP1,FP_AMP2,FP_AMP3,STDEV_NOISE,CIR_PWR,
// MAX_NOISE,RXPACC,CH,FRAME_LEN,PREAM_LEN,BITRATE,PRFR,CIR0..CIR99
// ============================================================================

static void print_all_distances(void)
{
    static char line_buffer[25000];

    complete_cycles++;

    if (!ml_header_printed)
    {
        printf("NLOS,RANGE,FP_IDX,FP_AMP1,FP_AMP2,FP_AMP3,"
               "STDEV_NOISE,CIR_PWR,MAX_NOISE,RXPACC,"
               "CH,FRAME_LEN,PREAM_LEN,BITRATE,PRFR");

        for (int s = 0; s < CIR_SAMPLES_PER_ANCHOR; s++)
            printf(",CIR%d", s);

        printf("\r\n");
        ml_header_printed = 1;
    }

    for (int anchor_idx = 0; anchor_idx < NUM_ANCHORS; anchor_idx++)
    {
        anchor_data_t *data = &anchor_data[anchor_idx];
        int pos = 0;

        pos += sprintf(line_buffer + pos,
                       "0x%04X,%.0f",
                       anchor_ids[anchor_idx],
                       data->distance);

        /* FP_IDX, FP_AMP1, FP_AMP2, FP_AMP3, STDEV_NOISE, CIR_PWR, MAX_NOISE, RXPACC */
        pos += sprintf(line_buffer + pos,
                       ",%u,%u,%u,%u,%u,%u,%u",
                       data->fp_index,
                       data->diagnostics.firstPathAmp1,
                       data->diagnostics.firstPathAmp2,
                       data->diagnostics.firstPathAmp3,
                       data->diagnostics.stdNoise,
                       data->cir_pwr,
                       data->diagnostics.maxNoise);

        /* CH, FRAME_LEN, PREAM_LEN, BITRATE, PRFR — từ config tĩnh + frame_len */
        //pos += sprintf(line_buffer + pos,
        //               ",%u,%lu,%u,%u,%u",
        //               config.chan,
        //               data->frame_len,
        //               config.txPreambLength,
        //               config.dataRate,
        //               config.prf);

        /* CIR magnitude */
        for (int s = 0; s < CIR_SAMPLES_PER_ANCHOR; s++)
        {
            int32_t  r         = data->cir_samples[s].real;
            int32_t  im        = data->cir_samples[s].img;
            uint32_t magnitude = sqrt_uint32((uint32_t)(r * r + im * im));
            pos += sprintf(line_buffer + pos, ",%lu", magnitude);
        }

        line_buffer[pos++] = '\r';
        line_buffer[pos++] = '\n';
        line_buffer[pos]   = '\0';

        printf("%s", line_buffer);
    }
}

// ============================================================================
// HELPER: message utilities
// ============================================================================

static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < RESP_MSG_TS_LEN; i++)
        *ts += ts_field[i] << (i * 8);
}

static uint32 resp_msg_get_device_id(uint8 *device_id_field)
{
    int    i;
    uint32 device_id = 0;
    for (i = 0; i < RESP_MSG_DEVICE_ID_LEN; i++)
        device_id += device_id_field[i] << (i * 8);
    return device_id;
}

static void poll_msg_set_device_id(uint8 *device_id_field, const uint32 device_id)
{
    int i;
    for (i = 0; i < POLL_MSG_DEVICE_ID_LEN; i++)
        device_id_field[i] = (device_id >> (i * 8)) & 0xFF;
}

// ============================================================================
// RANGING: do khoang cach 1 anchor
// ============================================================================

static int ss_init_single_anchor(uint32 target_anchor_id, uint8_t anchor_idx)
{
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    poll_msg_set_device_id(&tx_poll_msg[POLL_MSG_DEVICE_ID_IDX], target_anchor_id);

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    tx_count++;

    TickType_t start_tick    = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
        if ((xTaskGetTickCount() - start_tick) > timeout_ticks)
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            dwt_rxreset();
            frame_seq_nb++;
            timeout_count++;
            return 0;
        }
        vTaskDelay(0);
    }

    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

        if (frame_len <= RX_BUF_LEN)
            dwt_readrxdata(rx_buffer, frame_len, 0);

        /* Lưu frame_len vào anchor_data */
        anchor_data[anchor_idx].frame_len = frame_len;

        rx_buffer[ALL_MSG_SN_IDX] = 0;

        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            capture_anchor_diagnostics(anchor_idx);
            rx_count++;

            uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32  rtd_init, rtd_resp;
            float  clockOffsetRatio;
            uint32 device_id;

            device_id = resp_msg_get_device_id(&rx_buffer[RESP_MSG_DEVICE_ID_IDX]);
            if (device_id != target_anchor_id)
                return 0;

            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            clockOffsetRatio = dwt_readcarrierintegrator() *
                               (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            double tof      = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS;
            double distance = tof * SPEED_OF_LIGHT * 1000;

            anchor_data[anchor_idx].distance = distance;
            return 1;
        }
    }
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
    }

    return 0;
}

// ============================================================================
// MAIN RANGING LOOP
// ============================================================================

int ss_init_run(void)
{
    for (int i = 0; i < NUM_ANCHORS; i++)
        anchor_data[i].valid = 0;

    for (int anchor_idx = 0; anchor_idx < NUM_ANCHORS; anchor_idx++)
    {
        TickType_t slot_start = xTaskGetTickCount();

        uint32 target_anchor_id = anchor_ids[anchor_idx];

        int success = ss_init_single_anchor(target_anchor_id, anchor_idx);
        if (!success)
            skip_count++;

        TickType_t elapsed    = xTaskGetTickCount() - slot_start;
        TickType_t slot_ticks = pdMS_TO_TICKS(TIME_SLOT_MS);

        if (elapsed < slot_ticks)
            vTaskDelay(slot_ticks - elapsed);
    }

    for (int i = 0; i < 1; i++)
    {
        print_all_distances();
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
    vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));

    return 1;
}

// ============================================================================
// TASK ENTRY POINT
// ============================================================================

void ss_initiator_task_function(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    printf("\r\n");
    printf("========================================\r\n");
    printf("PURE TDMA 4-Anchor Ranging System v2.0\r\n");
    printf("========================================\r\n");
    printf("Time slot per anchor: %d ms\r\n", TIME_SLOT_MS);
    printf("Response timeout:     %d ms\r\n", RESPONSE_TIMEOUT_MS);
    printf("Cycle delay:          %d ms\r\n", CYCLE_DELAY_MS);
    printf("Total cycle time: ~%d ms\r\n", NUM_ANCHORS * TIME_SLOT_MS + CYCLE_DELAY_MS);
    printf("Anchor IDs: 0x%04X, 0x%04X, 0x%04X, 0x%04X\r\n",
           ANCHOR_1, ANCHOR_2, ANCHOR_3, ANCHOR_4);
    printf("========================================\r\n\r\n");

    for (int i = 0; i < NUM_ANCHORS; i++)
        anchor_data[i].valid = 0;

    while (true)
        ss_init_run();
}