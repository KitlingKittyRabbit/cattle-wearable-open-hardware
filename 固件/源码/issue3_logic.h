#ifndef ISSUE3_LOGIC_H
#define ISSUE3_LOGIC_H

#include <stdint.h>

/* LOG2 container layout used by the current formal firmware.  V4 keeps the
 * existing little-endian header and raw-sample encodings, but stores the
 * post-read QMI/GZP decimated stream in a physically smaller frame.  Older
 * V1/V2/V3 records remain valid historical formats and are parsed by the
 * host tool with their own sizes. */
#define ISSUE3_FRAME_MAGIC_LOG2 0x4C4F4732UL
#define ISSUE3_FRAME_VERSION 4U
#define ISSUE3_FRAME_HEADER_SIZE 32U
#define ISSUE3_FRAME_QMI_CAPACITY 16U
#define ISSUE3_FRAME_GZP_CAPACITY 5U
#define ISSUE3_FRAME_ICP_CAPACITY 2U
#define ISSUE3_FRAME_MTS_BYTES 6U
#define ISSUE3_FRAME_QMI_OFFSET ISSUE3_FRAME_HEADER_SIZE
#define ISSUE3_FRAME_GZP_OFFSET \
    (ISSUE3_FRAME_QMI_OFFSET + ISSUE3_FRAME_QMI_CAPACITY * 16U)
#define ISSUE3_FRAME_ICP_OFFSET \
    (ISSUE3_FRAME_GZP_OFFSET + ISSUE3_FRAME_GZP_CAPACITY * 10U)
#define ISSUE3_FRAME_MTS_OFFSET \
    (ISSUE3_FRAME_ICP_OFFSET + ISSUE3_FRAME_ICP_CAPACITY * 10U)
#define ISSUE3_FRAME_DATA_SIZE \
    (ISSUE3_FRAME_MTS_OFFSET + ISSUE3_FRAME_MTS_BYTES)
#define ISSUE3_FRAME_CRC_OFFSET ISSUE3_FRAME_DATA_SIZE
#define ISSUE3_FRAME_SIZE (ISSUE3_FRAME_DATA_SIZE + 4U)

/* These values describe the legal native QMI stream, not a host-side
 * resampling grid.  The retention calculation is deliberately explicit:
 * one complete one-second frame, one FIFO capacity worth of age, one
 * millisecond timestamp-rounding guard, and a bounded SD metadata guard. */
#define ISSUE3_QMI_SAMPLE_PERIOD_US 35682ULL
#define ISSUE3_QMI_FIFO_MAX_SAMPLES 128ULL
#define ISSUE3_FRAME_PERIOD_MS 1000ULL
#define ISSUE3_TIMESTAMP_ROUNDING_MS 1ULL
/* SPI1 is clocked from the reset HSI16/PCLK.  SD identification deliberately
 * uses BR=5 (250 kHz, below the 400 kHz SPI-init ceiling); after CMD58 and the
 * first sector probe the shared peripheral switches to BR=2 (2 MHz).  A byte
 * is therefore 4 us on the wire.  The 8 us bound includes the two bounded
 * status polls and register/CS software overhead, rather than pretending the
 * old GPIO bit-banging delay is still present. */
#define ISSUE3_SPI_INPUT_CLOCK_HZ 16000000UL
#define ISSUE3_SD_INIT_SPI_HZ 250000UL
#define ISSUE3_SD_RUNTIME_SPI_HZ 2000000UL
#define ISSUE3_SD_SPI_BYTE_US 4ULL
#define ISSUE3_SD_SPI_XFER_MAX_US 8ULL
#define ISSUE3_SD_SPI_WAIT_LIMIT_LOOPS 4096UL
#define ISSUE3_SD_SPI_INIT_BR 5U
#define ISSUE3_SD_SPI_RUNTIME_BR 2U
#define ISSUE3_SD_SPI_MAX_HZ 400000UL
#define ISSUE3_SD_SPI_WAIT_MAX_US 256ULL
#define ISSUE3_SD_FIXED_XFERS 600ULL
#define ISSUE3_SD_READ_TOKEN_POLLS 3000ULL
#define ISSUE3_SD_WRITE_BUSY_POLLS 1000ULL
#define ISSUE3_SD_READ_MAX_MS \
    (((ISSUE3_SD_FIXED_XFERS + ISSUE3_SD_READ_TOKEN_POLLS) * \
      ISSUE3_SD_SPI_XFER_MAX_US + 999ULL) / 1000ULL)
#define ISSUE3_SD_WRITE_MAX_MS \
    (((ISSUE3_SD_FIXED_XFERS + ISSUE3_SD_WRITE_BUSY_POLLS) * \
      ISSUE3_SD_SPI_XFER_MAX_US + 999ULL) / 1000ULL)
#define ISSUE3_SD_METADATA_MAX_MS (ISSUE3_SD_READ_MAX_MS + ISSUE3_SD_WRITE_MAX_MS)
/* The wire-time bound is 42 ms, but a real card may hold an accepted write
 * internally while FAT/metadata work is being resumed.  Keep the established
 * 1.82 s retention guard for that card-internal busy interval; it is a safety
 * horizon, not a claimed SPI throughput. */
#define ISSUE3_SD_METADATA_GUARD_MS 1820ULL
#define ISSUE3_SD_STEP_BYTES 8U
/* Runtime SD transactions keep CS asserted, but return to the main loop after
 * at most this many data bytes or one response/token/busy poll.  At the
 * measured 8 us/byte bound a data step is <=64 us; I2C sensor service runs
 * between every step. */
#define ISSUE3_SD_STEP_MAX_US (ISSUE3_SD_STEP_BYTES * ISSUE3_SD_SPI_XFER_MAX_US)

static inline uint32_t issue3_spi_clock_hz(uint8_t br)
{
    return ISSUE3_SPI_INPUT_CLOCK_HZ / (1UL << (br + 1U));
}

/* The NF radio and SD card share the hardware-SPI1 pins.  A transaction keeps
 * its owner until its chip-select has been released; callers must acquire
 * this small state machine before touching either peripheral.  Keeping the
 * arbitration here makes the firmware and host regressions exercise the same
 * rule rather than relying on call-site ordering. */
enum {
    ISSUE3_SPI_OWNER_NONE = 0,
    ISSUE3_SPI_OWNER_SD = 1,
    ISSUE3_SPI_OWNER_NF = 2
};

typedef struct {
    uint8_t owner;
} Issue3SpiBus;

static inline void issue3_spi_bus_init(Issue3SpiBus *bus)
{
    if (bus) bus->owner = ISSUE3_SPI_OWNER_NONE;
}

static inline uint8_t issue3_spi_bus_try_acquire(Issue3SpiBus *bus,
                                                 uint8_t owner)
{
    if (!bus || owner == ISSUE3_SPI_OWNER_NONE) return 0U;
    if (bus->owner != ISSUE3_SPI_OWNER_NONE) return 0U;
    bus->owner = owner;
    return 1U;
}

static inline uint8_t issue3_spi_bus_owned(const Issue3SpiBus *bus,
                                           uint8_t owner)
{
    return bus && owner != ISSUE3_SPI_OWNER_NONE && bus->owner == owner;
}

static inline void issue3_spi_bus_release(Issue3SpiBus *bus, uint8_t owner)
{
    if (bus && bus->owner == owner) bus->owner = ISSUE3_SPI_OWNER_NONE;
}

/* Shared progress rule for firmware and host integration: a sector transfer
 * may advance only this bounded prefix before returning to sensor service. */
static inline uint16_t issue3_sd_step_chunk(uint16_t remaining_bytes)
{
    return remaining_bytes > ISSUE3_SD_STEP_BYTES ?
           (uint16_t)ISSUE3_SD_STEP_BYTES : remaining_bytes;
}

enum {
    ISSUE3_SD_OP_READ = 1,
    ISSUE3_SD_OP_WRITE = 2,
    ISSUE3_SD_PHASE_CMD = 0,
    ISSUE3_SD_PHASE_RESP,
    ISSUE3_SD_PHASE_TOKEN,
    ISSUE3_SD_PHASE_READ_DATA,
    ISSUE3_SD_PHASE_READ_CRC,
    ISSUE3_SD_PHASE_WRITE_TOKEN,
    ISSUE3_SD_PHASE_WRITE_DATA,
    ISSUE3_SD_PHASE_WRITE_CRC,
    ISSUE3_SD_PHASE_WRITE_RESP,
    ISSUE3_SD_PHASE_WRITE_BUSY
};

#define ISSUE3_SD_PHASE_COUNT (ISSUE3_SD_PHASE_WRITE_BUSY + 1U)
/* Diagnostics aggregate token/CRC sub-phases into the data/token stage; the
 * protocol state machine above is still unchanged and retains all ten states. */
#define ISSUE3_SD_DIAG_STAGE_COUNT 7U

/* Fixed-layout runtime diagnostics.  Counters and cumulative durations use
 * 32-bit fields; per-operation last/max durations use saturating 16-bit ms
 * values because the bounded SD stages are far below 65 seconds.  The layout
 * is easy to inspect over a halted ST-Link session and needs no printf/heap;
 * it is not part of the SD log or NF protocol. */
#define ISSUE3_RUNTIME_DIAG_MAGIC 0x49534433UL /* "ISD3" */
#define ISSUE3_RUNTIME_DIAG_VERSION 1UL

/* Exact reason for a rejected frame admission.  This is diagnostic metadata
 * only; the signed open result and all admission decisions remain unchanged. */
typedef enum {
    ISSUE3_FRAME_OPEN_REASON_NONE = 0,
    ISSUE3_FRAME_OPEN_REASON_TIME = 1,
    ISSUE3_FRAME_OPEN_REASON_OWNER = 2,
    ISSUE3_FRAME_OPEN_REASON_QUEUE = 3
} Issue3FrameOpenReason;

typedef struct {
    uint32_t entries;
    uint32_t total_ms;
    uint16_t last_ms;
    uint16_t max_ms;
} Issue3DiagStage;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t runtime_ms;
    uint32_t main_loop_count;
    uint32_t sd_step_count;
    uint32_t sd_step_max_gap_ms;
    uint32_t sector_complete_count;
    uint32_t sector_last_gap_ms;
    uint32_t sector_max_gap_ms;
    uint32_t frame_queue_max_depth;
    uint32_t gzp_queue_max_depth;
    uint32_t qmi_received;
    uint32_t gzp_received;
    uint32_t icp_received;
    uint32_t qmi_admitted;
    uint32_t gzp_admitted;
    uint32_t icp_admitted;
    uint32_t qmi_drop_queue;
    uint32_t qmi_drop_time;
    uint32_t qmi_drop_owner;
    uint32_t gzp_drop_queue;
    uint32_t gzp_drop_time;
    uint32_t gzp_drop_owner;
    uint32_t icp_drop_queue;
    uint32_t icp_drop_time;
    uint32_t icp_drop_owner;
    uint32_t complete_frame_count;
    uint32_t persisted_bytes;
    uint32_t sd_transactions_total;
    uint32_t sd_read_transactions;
    uint32_t sd_write_transactions;
    uint32_t spi_xfer_bytes;
    uint32_t read_token_polls;
    uint32_t read_token_max_polls;
    uint32_t write_busy_polls;
    uint32_t write_busy_max_polls;
    uint32_t spi_timeout_count;
    Issue3DiagStage sd_transaction;
    Issue3DiagStage sd_phase[ISSUE3_SD_DIAG_STAGE_COUNT];
    Issue3DiagStage fat_prefetch;
    Issue3DiagStage fat_link;
    Issue3DiagStage metadata;
} Issue3RuntimeDiagnostics;

enum {
    ISSUE3_DIAG_SENSOR_QMI = 0,
    ISSUE3_DIAG_SENSOR_GZP = 1,
    ISSUE3_DIAG_SENSOR_ICP = 2
};

/* Transient bookkeeping for the production diagnostic state machine.  This
 * is deliberately kept outside Issue3RuntimeDiagnostics: the latter is a
 * fixed ST-Link ABI, while this context is only the small amount of state
 * needed between resumable foreground calls.  The firmware and host tests
 * pass the same context through the shared transition helpers below. */
typedef struct {
    uint32_t sd_txn_start_ms;
    uint32_t sd_phase_start_ms;
    uint32_t sd_last_step_ms;
    uint32_t sd_last_sector_ms;
    uint32_t sd_token_polls_current;
    uint32_t sd_busy_polls_current;
    uint32_t fat_start_ms;
    uint32_t link_start_ms;
    uint32_t metadata_start_ms;
    uint8_t sd_phase;
    uint8_t sd_step_seen;
    uint8_t sd_sector_seen;
    uint8_t fat_active;
    uint8_t link_active;
    uint8_t metadata_active;
} Issue3DiagRuntimeState;

static inline void issue3_diag_sat_add(volatile uint32_t *value,
                                       uint32_t amount)
{
    if (!value || amount == 0U) return;
    if (*value > 0xFFFFFFFFUL - amount) *value = 0xFFFFFFFFUL;
    else *value += amount;
}

static inline void issue3_diag_sat_inc(volatile uint32_t *value)
{
    issue3_diag_sat_add(value, 1U);
}

static inline void issue3_diag_note_frame_drop(
    volatile Issue3RuntimeDiagnostics *diag, uint8_t sensor,
    Issue3FrameOpenReason reason)
{
    if (!diag) return;
    volatile uint32_t *queue_drop = 0;
    volatile uint32_t *time_drop = 0;
    volatile uint32_t *owner_drop = 0;
    if (sensor == ISSUE3_DIAG_SENSOR_QMI) {
        queue_drop = &diag->qmi_drop_queue;
        time_drop = &diag->qmi_drop_time;
        owner_drop = &diag->qmi_drop_owner;
    } else if (sensor == ISSUE3_DIAG_SENSOR_GZP) {
        queue_drop = &diag->gzp_drop_queue;
        time_drop = &diag->gzp_drop_time;
        owner_drop = &diag->gzp_drop_owner;
    } else if (sensor == ISSUE3_DIAG_SENSOR_ICP) {
        queue_drop = &diag->icp_drop_queue;
        time_drop = &diag->icp_drop_time;
        owner_drop = &diag->icp_drop_owner;
    } else {
        return;
    }
    if (reason == ISSUE3_FRAME_OPEN_REASON_TIME)
        issue3_diag_sat_inc(time_drop);
    else if (reason == ISSUE3_FRAME_OPEN_REASON_OWNER)
        issue3_diag_sat_inc(owner_drop);
    else
        issue3_diag_sat_inc(queue_drop);
}

static inline void issue3_diag_stage_enter(volatile Issue3DiagStage *stage)
{
    if (stage) issue3_diag_sat_inc(&stage->entries);
}

static inline void issue3_diag_stage_finish(volatile Issue3DiagStage *stage,
                                            uint32_t elapsed_ms)
{
    if (!stage) return;
    stage->last_ms = elapsed_ms > 0xFFFFU ? 0xFFFFU : (uint16_t)elapsed_ms;
    issue3_diag_sat_add(&stage->total_ms, elapsed_ms);
    if (elapsed_ms > stage->max_ms)
        stage->max_ms = elapsed_ms > 0xFFFFU ? 0xFFFFU : (uint16_t)elapsed_ms;
}

static inline uint8_t issue3_diag_sd_phase_index(uint8_t phase)
{
    if (phase == ISSUE3_SD_PHASE_CMD) return 0U;
    if (phase == ISSUE3_SD_PHASE_RESP) return 1U;
    if (phase == ISSUE3_SD_PHASE_TOKEN || phase == ISSUE3_SD_PHASE_WRITE_TOKEN)
        return 2U;
    if (phase == ISSUE3_SD_PHASE_READ_DATA || phase == ISSUE3_SD_PHASE_READ_CRC)
        return 3U;
    if (phase == ISSUE3_SD_PHASE_WRITE_DATA || phase == ISSUE3_SD_PHASE_WRITE_CRC)
        return 4U;
    if (phase == ISSUE3_SD_PHASE_WRITE_RESP) return 5U;
    if (phase == ISSUE3_SD_PHASE_WRITE_BUSY) return 6U;
    return ISSUE3_SD_DIAG_STAGE_COUNT;
}

static inline void issue3_diag_sd_phase_enter(
    volatile Issue3RuntimeDiagnostics *diag, uint8_t phase)
{
    uint8_t index = issue3_diag_sd_phase_index(phase);
    if (diag && index < ISSUE3_SD_DIAG_STAGE_COUNT)
        issue3_diag_stage_enter(&diag->sd_phase[index]);
}

static inline void issue3_diag_sd_phase_finish(
    volatile Issue3RuntimeDiagnostics *diag, uint8_t phase,
    uint32_t elapsed_ms)
{
    uint8_t index = issue3_diag_sd_phase_index(phase);
    if (diag && index < ISSUE3_SD_DIAG_STAGE_COUNT)
        issue3_diag_stage_finish(&diag->sd_phase[index], elapsed_ms);
}

/* Shared production/test transitions for the RAM-only diagnostics.  A
 * transaction begins with phase CMD and ends only after its CS has been
 * released.  sd_step_max_gap_ms is intentionally updated only after a first
 * step has occurred in the same active transaction; transaction-end resets
 * sd_step_seen so ordinary idle time between transactions is not reported as
 * an active SD scheduling gap. */
static inline void issue3_diag_runtime_sd_begin(
    volatile Issue3RuntimeDiagnostics *diag,
    volatile Issue3DiagRuntimeState *state, uint8_t op, uint32_t now_ms)
{
    if (!diag || !state) return;
    issue3_diag_stage_enter(&diag->sd_transaction);
    issue3_diag_sat_inc(&diag->sd_transactions_total);
    if (op == ISSUE3_SD_OP_READ)
        issue3_diag_sat_inc(&diag->sd_read_transactions);
    else
        issue3_diag_sat_inc(&diag->sd_write_transactions);
    state->sd_txn_start_ms = now_ms;
    state->sd_token_polls_current = 0U;
    state->sd_busy_polls_current = 0U;
    state->sd_step_seen = 0U;
    state->sd_phase = ISSUE3_SD_PHASE_CMD;
    state->sd_phase_start_ms = now_ms;
    issue3_diag_sd_phase_enter(diag, ISSUE3_SD_PHASE_CMD);
}

static inline void issue3_diag_runtime_sd_step(
    volatile Issue3RuntimeDiagnostics *diag,
    volatile Issue3DiagRuntimeState *state, uint32_t now_ms,
    uint8_t phase)
{
    if (!diag || !state || state->sd_phase >= ISSUE3_SD_PHASE_COUNT)
        return;
    issue3_diag_sat_inc(&diag->sd_step_count);
    if (state->sd_step_seen) {
        uint32_t gap = now_ms - state->sd_last_step_ms;
        if (gap > diag->sd_step_max_gap_ms)
            diag->sd_step_max_gap_ms = gap;
    }
    state->sd_last_step_ms = now_ms;
    state->sd_step_seen = 1U;
    if (phase == ISSUE3_SD_PHASE_TOKEN) {
        issue3_diag_sat_inc(&diag->read_token_polls);
        issue3_diag_sat_inc(&state->sd_token_polls_current);
    } else if (phase == ISSUE3_SD_PHASE_WRITE_BUSY) {
        issue3_diag_sat_inc(&diag->write_busy_polls);
        issue3_diag_sat_inc(&state->sd_busy_polls_current);
    }
}

static inline void issue3_diag_runtime_sd_phase_change(
    volatile Issue3RuntimeDiagnostics *diag,
    volatile Issue3DiagRuntimeState *state, uint8_t from_phase,
    uint8_t to_phase, uint32_t now_ms)
{
    if (!diag || !state || from_phase >= ISSUE3_SD_PHASE_COUNT ||
        to_phase >= ISSUE3_SD_PHASE_COUNT) return;
    issue3_diag_sd_phase_finish(diag, from_phase,
                                now_ms - state->sd_phase_start_ms);
    state->sd_phase = to_phase;
    state->sd_phase_start_ms = now_ms;
    issue3_diag_sd_phase_enter(diag, to_phase);
}

static inline void issue3_diag_runtime_sd_finish(
    volatile Issue3RuntimeDiagnostics *diag,
    volatile Issue3DiagRuntimeState *state, uint32_t now_ms)
{
    if (!diag || !state || state->sd_phase >= ISSUE3_SD_PHASE_COUNT)
        return;
    issue3_diag_sd_phase_finish(diag, state->sd_phase,
                                now_ms - state->sd_phase_start_ms);
    issue3_diag_stage_finish(&diag->sd_transaction,
                             now_ms - state->sd_txn_start_ms);
    if (state->sd_token_polls_current > diag->read_token_max_polls)
        diag->read_token_max_polls = state->sd_token_polls_current;
    if (state->sd_busy_polls_current > diag->write_busy_max_polls)
        diag->write_busy_max_polls = state->sd_busy_polls_current;
    /* 0xff is the explicit inactive marker; the next transaction starts a
     * fresh step sequence, so inter-transaction idle cannot affect the max. */
    state->sd_phase = 0xffU;
    state->sd_step_seen = 0U;
}

static inline void issue3_diag_runtime_operation_begin(
    volatile Issue3RuntimeDiagnostics *diag, volatile Issue3DiagStage *stage,
    uint8_t *active, uint32_t *start_ms, uint32_t now_ms)
{
    if (!diag || !stage || !active || !start_ms || *active) return;
    *active = 1U;
    *start_ms = now_ms;
    issue3_diag_stage_enter(stage);
}

static inline void issue3_diag_runtime_operation_finish(
    volatile Issue3RuntimeDiagnostics *diag, volatile Issue3DiagStage *stage,
    uint8_t *active, uint32_t start_ms, uint32_t now_ms)
{
    if (!diag || !stage || !active || !*active) return;
    issue3_diag_stage_finish(stage, now_ms - start_ms);
    *active = 0U;
}

static inline void issue3_diag_runtime_sector_complete(
    volatile Issue3RuntimeDiagnostics *diag,
    volatile Issue3DiagRuntimeState *state, uint32_t now_ms,
    uint32_t persisted_bytes, uint32_t complete_frames)
{
    if (!diag || !state) return;
    issue3_diag_sat_inc(&diag->sector_complete_count);
    if (state->sd_sector_seen)
        diag->sector_last_gap_ms = now_ms - state->sd_last_sector_ms;
    if (state->sd_sector_seen && diag->sector_last_gap_ms >
        diag->sector_max_gap_ms)
        diag->sector_max_gap_ms = diag->sector_last_gap_ms;
    state->sd_last_sector_ms = now_ms;
    state->sd_sector_seen = 1U;
    diag->persisted_bytes = persisted_bytes;
    diag->complete_frame_count = complete_frames;
}

typedef uint8_t (*Issue3SdXferFn)(void *ctx, uint8_t out);
typedef void (*Issue3SdCsFn)(void *ctx, uint8_t high);

/* This is the exact byte/response state machine used by main.c.  Keeping the
 * protocol phases in the shared C core lets the host integration drive the
 * same resumable transaction with a byte-level mock instead of inventing a
 * synchronous sector model. */
typedef struct {
    uint8_t active;
    uint8_t op;
    uint8_t phase;
    uint8_t result_ready;
    uint8_t result;
    uint16_t index;
    uint16_t polls;
    uint32_t lba;
    uint8_t *read_buf;
    const uint8_t *write_buf;
} Issue3SdTransfer;

static inline void issue3_sd_transfer_finish(Issue3SdTransfer *transfer,
                                             Issue3SdCsFn cs,
                                             Issue3SdXferFn xfer,
                                             void *ctx, uint8_t result)
{
    if (cs) cs(ctx, 1U);
    if (xfer) (void)xfer(ctx, 0xFFU);
    transfer->active = 0U;
    transfer->result = result;
    transfer->result_ready = 1U;
}

static inline int issue3_sd_transfer_start(Issue3SdTransfer *transfer,
                                           uint8_t op, uint32_t lba,
                                           uint8_t *read_buf,
                                           const uint8_t *write_buf,
                                           Issue3SdCsFn cs,
                                           Issue3SdXferFn xfer, void *ctx)
{
    if (!transfer || transfer->active || transfer->result_ready ||
        (op != ISSUE3_SD_OP_READ && op != ISSUE3_SD_OP_WRITE)) return -1;
    transfer->active = 1U;
    transfer->op = op;
    transfer->phase = ISSUE3_SD_PHASE_CMD;
    transfer->result_ready = 0U;
    transfer->result = 1U;
    transfer->index = 0U;
    transfer->polls = 0U;
    transfer->lba = lba;
    transfer->read_buf = read_buf;
    transfer->write_buf = write_buf;
    if (cs) cs(ctx, 1U);
    if (xfer) (void)xfer(ctx, 0xFFU);
    if (cs) cs(ctx, 0U);
    return 0;
}

static inline int issue3_sd_transfer_step(Issue3SdTransfer *transfer,
                                          Issue3SdCsFn cs,
                                          Issue3SdXferFn xfer, void *ctx)
{
    if (!transfer || !xfer) return -1;
    if (!transfer->active) return transfer->result_ready ? 0 : -1;

    if (transfer->phase == ISSUE3_SD_PHASE_CMD) {
        uint8_t byte;
        if (transfer->index == 0U) byte = 0xFFU;
        else if (transfer->index == 1U) byte = (uint8_t)(
            transfer->op == ISSUE3_SD_OP_READ ? 0x51U : 0x58U);
        else if (transfer->index == 2U) byte = (uint8_t)(transfer->lba >> 24);
        else if (transfer->index == 3U) byte = (uint8_t)(transfer->lba >> 16);
        else if (transfer->index == 4U) byte = (uint8_t)(transfer->lba >> 8);
        else if (transfer->index == 5U) byte = (uint8_t)transfer->lba;
        else byte = 0x01U;
        (void)xfer(ctx, byte);
        transfer->index++;
        if (transfer->index >= 7U) {
            transfer->phase = ISSUE3_SD_PHASE_RESP;
            transfer->index = 0U;
            transfer->polls = 0U;
        }
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_RESP) {
        uint8_t response = xfer(ctx, 0xFFU);
        transfer->polls++;
        if ((response & 0x80U) == 0U) {
            transfer->phase = transfer->op == ISSUE3_SD_OP_READ ?
                ISSUE3_SD_PHASE_TOKEN : ISSUE3_SD_PHASE_WRITE_TOKEN;
            transfer->polls = 0U;
        } else if (transfer->polls >= 10U) {
            issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 1U);
        }
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_TOKEN) {
        uint8_t token = xfer(ctx, 0xFFU);
        transfer->polls++;
        if (token == 0xFEU) {
            transfer->phase = ISSUE3_SD_PHASE_READ_DATA;
            transfer->index = 0U;
        } else if (transfer->polls >= ISSUE3_SD_READ_TOKEN_POLLS) {
            issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 1U);
        }
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_READ_DATA) {
        uint16_t chunk = issue3_sd_step_chunk(
            (uint16_t)(512U - transfer->index));
        for (uint16_t i = 0U; i < chunk; i++)
            transfer->read_buf[transfer->index + i] = xfer(ctx, 0xFFU);
        transfer->index = (uint16_t)(transfer->index + chunk);
        if (transfer->index >= 512U) {
            transfer->phase = ISSUE3_SD_PHASE_READ_CRC;
            transfer->index = 0U;
        }
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_READ_CRC) {
        (void)xfer(ctx, 0xFFU);
        transfer->index++;
        if (transfer->index >= 2U)
            issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 0U);
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_WRITE_TOKEN) {
        (void)xfer(ctx, 0xFEU);
        transfer->phase = ISSUE3_SD_PHASE_WRITE_DATA;
        transfer->index = 0U;
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_WRITE_DATA) {
        uint16_t chunk = issue3_sd_step_chunk(
            (uint16_t)(512U - transfer->index));
        for (uint16_t i = 0U; i < chunk; i++)
            (void)xfer(ctx, transfer->write_buf[transfer->index + i]);
        transfer->index = (uint16_t)(transfer->index + chunk);
        if (transfer->index >= 512U) {
            transfer->phase = ISSUE3_SD_PHASE_WRITE_CRC;
            transfer->index = 0U;
        }
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_WRITE_CRC) {
        (void)xfer(ctx, 0xFFU);
        transfer->index++;
        if (transfer->index >= 2U)
            transfer->phase = ISSUE3_SD_PHASE_WRITE_RESP;
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_WRITE_RESP) {
        uint8_t response = xfer(ctx, 0xFFU);
        if ((response & 0x1FU) != 0x05U) {
            issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 1U);
        } else {
            transfer->phase = ISSUE3_SD_PHASE_WRITE_BUSY;
            transfer->polls = 0U;
        }
        return 1;
    }
    if (transfer->phase == ISSUE3_SD_PHASE_WRITE_BUSY) {
        if (xfer(ctx, 0xFFU) == 0xFFU) {
            issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 0U);
        } else {
            transfer->polls++;
            if (transfer->polls >= ISSUE3_SD_WRITE_BUSY_POLLS)
                issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 1U);
        }
        return 1;
    }
    issue3_sd_transfer_finish(transfer, cs, xfer, ctx, 1U);
    return 1;
}

static inline int issue3_sd_transfer_take_result(Issue3SdTransfer *transfer)
{
    if (!transfer || !transfer->result_ready) return 1;
    int result = transfer->result == 0U ? 0 : -1;
    transfer->result_ready = 0U;
    return result;
}
#define ISSUE3_QMI_READ_GUARD_MS (ISSUE3_SD_METADATA_GUARD_MS + 100ULL)
#define ISSUE3_QMI_FIFO_MAX_DELAY_MS \
    ((ISSUE3_QMI_FIFO_MAX_SAMPLES * ISSUE3_QMI_SAMPLE_PERIOD_US + 999ULL) / 1000ULL)
#define ISSUE3_FRAME_RETENTION_MS \
    (ISSUE3_FRAME_PERIOD_MS + ISSUE3_QMI_FIFO_MAX_DELAY_MS + \
     ISSUE3_TIMESTAMP_ROUNDING_MS + ISSUE3_QMI_READ_GUARD_MS)

#define ISSUE3_GZP_PERIOD_MS 100U
/* 0xB4 is nominally about 19 ms.  Runtime SD data steps are bounded to
 * 64 us, so a current grid point remains eligible within this 10 ms service
 * margin; delays beyond it are real missed schedule points. */
#define ISSUE3_GZP_START_LATE_LIMIT_MS 10U
#define ISSUE3_FRAME_QUEUE_CAPACITY 8U
#define ISSUE3_SECTOR_BYTES 512U
#define ISSUE3_README_SECTOR_BYTES 512U

/* Read-side two-phase decimation shared by firmware and host tests.  The
 * phase is caller-owned and must live across FIFO batches, frame boundaries,
 * retries and scheduler turns.  A zero phase keeps the first successfully
 * read sample/result; the following one is intentionally omitted, then the
 * phase alternates forever.  This helper does not classify the omitted item
 * as a fault and does not touch any diagnostic counter. */
static inline uint8_t issue3_decimation_keep_next(uint8_t *phase)
{
    if (!phase) return 0U;
    uint8_t keep = (uint8_t)(*phase == 0U);
    *phase ^= 1U;
    return keep;
}

static inline int issue3_time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/* Runtime main-loop arbitration is shared with the host integration model.
 * A WFI is legal only when no bounded foreground step is ready.  The caller
 * supplies the earliest interrupt-backed deadline (normally the next TIM3
 * update, or an earlier conversion-complete compare event).  Keeping this
 * decision in the shared C core prevents the host test from silently running
 * a faster scheduler than the firmware. */
typedef struct {
    uint8_t sd_active;
    uint8_t sd_work_pending;
    uint8_t qmi_ready;
    uint8_t icp_ready;
    uint8_t gzp_ready;
    uint8_t gzp_result_ready;
    uint8_t mts_ready;
    uint8_t mts_result_ready;
    uint8_t nf_ready;
    uint8_t frame_ready;
    uint8_t wake_source_ready;
    uint32_t now_ms;
    uint32_t wake_deadline_ms;
    uint8_t wake_pending_mask;
} Issue3RuntimeWork;

static inline uint8_t issue3_runtime_has_immediate_work(
    const Issue3RuntimeWork *work)
{
    if (!work) return 0U;
    return (uint8_t)(work->sd_active || work->sd_work_pending ||
                     work->qmi_ready || work->icp_ready || work->gzp_ready ||
                     work->gzp_result_ready || work->mts_ready ||
                     work->mts_result_ready || work->nf_ready ||
                     work->frame_ready || work->wake_pending_mask != 0U);
}

static inline uint8_t issue3_runtime_should_wfi(
    const Issue3RuntimeWork *work)
{
    if (!work || !work->wake_source_ready ||
        issue3_runtime_has_immediate_work(work)) return 0U;
    return (uint8_t)!issue3_time_reached(work->now_ms,
                                         work->wake_deadline_ms);
}

/* Hardware wake sources that must remain visible while PRIMASK is set.  A
 * pending interrupt is deliberately not converted into a software-only flag:
 * Cortex-M0+ WFI returns for a pending interrupt even while it is masked, and
 * the ISR then runs immediately after interrupts are restored. */
enum {
    ISSUE3_WAKE_PENDING_TIM3_COMPARE = 1U << 0,
    ISSUE3_WAKE_PENDING_TIM3_UPDATE  = 1U << 1,
    ISSUE3_WAKE_PENDING_QMI_EXTI     = 1U << 2
};

typedef void (*Issue3RuntimeIrqFn)(void *ctx);
typedef void (*Issue3RuntimeRefreshFn)(void *ctx,
                                       Issue3RuntimeWork *work);
typedef void (*Issue3RuntimeArmFn)(void *ctx, uint32_t deadline_ms);
typedef void (*Issue3RuntimeWfiFn)(void *ctx);

typedef struct {
    Issue3RuntimeIrqFn disable_irq;
    Issue3RuntimeIrqFn enable_irq;
    Issue3RuntimeRefreshFn refresh;
    Issue3RuntimeArmFn arm_wake;
    /* Host regressions use this hook to inject an interrupt after the final
     * refresh/arm and immediately before WFI.  Firmware leaves it null. */
    Issue3RuntimeIrqFn before_wfi;
    Issue3RuntimeWfiFn wfi;
} Issue3RuntimeSleepOps;

/* The sole production sleep entrance.  Interrupts are masked before the
 * final refresh, so no ISR can run between the last work check and wake-source
 * programming.  An interrupt arriving in that interval remains pending and
 * wakes WFI under PRIMASK; restoring interrupts then dispatches its ISR. */
static inline uint8_t issue3_runtime_sleep_entry(
    Issue3RuntimeWork *work, const Issue3RuntimeSleepOps *ops, void *ctx)
{
    if (!work || !ops || !ops->disable_irq || !ops->enable_irq ||
        !ops->refresh || !ops->arm_wake || !ops->wfi)
        return 0U;
    ops->disable_irq(ctx);
    ops->refresh(ctx, work);
    if (!issue3_runtime_should_wfi(work)) {
        ops->enable_irq(ctx);
        return 0U;
    }
    ops->arm_wake(ctx, work->wake_deadline_ms);
    if (ops->before_wfi) ops->before_wfi(ctx);
    ops->wfi(ctx);
    ops->enable_irq(ctx);
    return 1U;
}

typedef uint8_t (*Issue3AtomicWaitReadyFn)(void *ctx);
typedef uint8_t (*Issue3AtomicWaitPendingFn)(void *ctx);
typedef void (*Issue3AtomicWaitArmFn)(void *ctx);
typedef void (*Issue3AtomicWaitStartFn)(void *ctx);

typedef struct {
    Issue3RuntimeIrqFn disable_irq;
    Issue3RuntimeIrqFn enable_irq;
    Issue3AtomicWaitReadyFn ready;
    Issue3AtomicWaitPendingFn pending;
    Issue3AtomicWaitArmFn arm;
    /* Host regressions inject CC1 in this exact masked window. */
    Issue3RuntimeIrqFn before_wfi;
    Issue3RuntimeWfiFn wfi;
    /* Start a new timer interval while interrupts remain masked. */
    Issue3AtomicWaitStartFn start;
} Issue3AtomicWaitOps;

/* Establish a new short interval before entering the normal atomic wait
 * loop.  The start callback owns stale-pending cleanup and timer programming;
 * keeping it under the same PRIMASK region prevents an old compare event from
 * being mistaken for completion of the new interval. */
static inline uint8_t issue3_atomic_wait_begin(
    const Issue3AtomicWaitOps *ops, void *ctx)
{
    if (!ops || !ops->disable_irq || !ops->enable_irq || !ops->start)
        return 0U;
    ops->disable_irq(ctx);
    ops->start(ctx);
    ops->enable_irq(ctx);
    return 1U;
}

#define ISSUE3_SHORT_TIMER_PERIOD_MS 100U

/* Return elapsed TIM3 ticks in the 0..99 modulo-100 timebase.  The short
 * timer setup is bounded to far less than one full period, so this also
 * distinguishes a target that was passed while CCR1 was being programmed. */
static inline uint32_t issue3_short_timer_elapsed(uint32_t start,
                                                  uint32_t now)
{
    return (now >= start) ? (now - start) :
           (ISSUE3_SHORT_TIMER_PERIOD_MS - start + now);
}

/* Compute a compare target only while it is still in the future.  Production
 * setup retries from a fresh CNT sample when configuration crossed the
 * requested 1..20 ms interval; host tests exercise the same decision. */
static inline uint8_t issue3_short_timer_target(uint32_t start,
                                                uint32_t now,
                                                uint16_t delay_ms,
                                                uint32_t *target)
{
    if (!target || delay_ms == 0U || delay_ms >= ISSUE3_SHORT_TIMER_PERIOD_MS)
        return 0U;
    if (issue3_short_timer_elapsed(start, now) >= delay_ms) return 0U;
    *target = (start + delay_ms) % ISSUE3_SHORT_TIMER_PERIOD_MS;
    return 1U;
}

/* One bounded step of a short hardware-timer wait.  Both the completion flag
 * and the peripheral pending bit are read only after interrupts are masked;
 * an event arriving after that read stays pending and wakes WFI under
 * PRIMASK.  The caller may repeat this step until ready() returns true. */
static inline uint8_t issue3_atomic_wait_step(
    const Issue3AtomicWaitOps *ops, void *ctx)
{
    if (!ops || !ops->disable_irq || !ops->enable_irq || !ops->ready ||
        !ops->pending || !ops->arm || !ops->wfi)
        return 0U;
    ops->disable_irq(ctx);
    if (ops->ready(ctx)) {
        ops->enable_irq(ctx);
        return 0U;
    }
    if (!ops->pending(ctx)) ops->arm(ctx);
    if (ops->before_wfi) ops->before_wfi(ctx);
    ops->wfi(ctx);
    ops->enable_irq(ctx);
    return 1U;
}

static inline int issue3_frame_commit_due(uint32_t frame_start_ms,
                                          uint32_t now_ms)
{
    return issue3_time_reached(now_ms,
                               frame_start_ms + (uint32_t)ISSUE3_FRAME_RETENTION_MS);
}

typedef void (*Issue3FrameCommitFn)(void *ctx, uint8_t slot_index,
                                    uint32_t frame_start_ms);

enum {
    ISSUE3_FRAME_OPEN_EXISTING = 0,
    ISSUE3_FRAME_OPEN_NEW = 1,
    ISSUE3_FRAME_OPEN_LATE = -1,
    ISSUE3_FRAME_OPEN_BLOCKED = -2,
};

typedef struct {
    uint32_t start_ms[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t count;
    uint8_t pending_valid;
    uint8_t pending_slot;
} Issue3FrameQueue;

static inline void issue3_frame_queue_init(Issue3FrameQueue *queue)
{
    queue->head = 0;
    queue->count = 0;
    queue->pending_valid = 0;
    queue->pending_slot = 0;
}

static inline uint8_t issue3_frame_queue_index(const Issue3FrameQueue *queue,
                                               uint8_t logical_index)
{
    return (uint8_t)((queue->head + logical_index) %
                     ISSUE3_FRAME_QUEUE_CAPACITY);
}

/* Frame sequence numbers are allocated when a new chronological frame is
 * opened, not when an older frame happens to reach the SD writer.  This keeps
 * the identity stable across queue wraparound, delayed commits, and a commit
 * callback that is interrupted by a pending SD write. */
static inline uint32_t issue3_frame_sequence_take(volatile uint32_t *next_sequence)
{
    uint32_t sequence = *next_sequence;
    *next_sequence = sequence + 1U;
    return sequence;
}

/* A committed frame may remain in its physical slot while SD writes its
 * bytes.  The queue count has already advanced, so this explicit ownership
 * bit prevents the ring allocator from wrapping onto that slot. */
static inline void issue3_frame_queue_claim_pending(Issue3FrameQueue *queue,
                                                     uint8_t slot_index)
{
    queue->pending_valid = 1;
    queue->pending_slot = slot_index;
}

static inline void issue3_frame_queue_release_pending(Issue3FrameQueue *queue,
                                                       uint8_t slot_index)
{
    if (queue->pending_valid && queue->pending_slot == slot_index)
        queue->pending_valid = 0;
}

static inline int issue3_frame_queue_commit_oldest(Issue3FrameQueue *queue,
                                                    Issue3FrameCommitFn commit,
                                                    void *ctx)
{
    if (queue->count == 0) return 0;
    uint8_t index = queue->head;
    uint32_t start_ms = queue->start_ms[index];
    if (commit) commit(ctx, index, start_ms);
    queue->head = (uint8_t)((queue->head + 1U) % ISSUE3_FRAME_QUEUE_CAPACITY);
    queue->count--;
    return 1;
}

static inline uint8_t issue3_frame_queue_commit_ready(Issue3FrameQueue *queue,
                                                       uint32_t now_ms,
                                                       Issue3FrameCommitFn commit,
                                                       void *ctx)
{
    uint8_t committed = 0;
    if (queue->pending_valid) return 0;
    /* One callback per service turn keeps frame finalization bounded; a
     * caller can invoke this again on the next turn for another due frame. */
    if (queue->count != 0 && issue3_frame_commit_due(
            queue->start_ms[queue->head], now_ms)) {
        (void)issue3_frame_queue_commit_oldest(queue, commit, ctx);
        committed++;
    }
    return committed;
}

/* Check whether a delayed FIFO batch can be admitted without requiring the
 * queue's implicit in-batch commit path.  The caller supplies the oldest and
 * newest sample timestamps; every second in that span must already exist or
 * fit in a free slot. */
static inline uint8_t issue3_frame_queue_span_fits(
    const Issue3FrameQueue *queue, uint32_t first_sample_ms,
    uint32_t last_sample_ms)
{
    uint32_t start = first_sample_ms - (first_sample_ms % 1000U);
    uint32_t end = last_sample_ms - (last_sample_ms % 1000U);
    uint8_t reserved = queue->pending_valid ? 1U : 0U;
    if (queue->count == 0) {
        uint32_t span = (end - start) / 1000U + 1U;
        return span + reserved <= ISSUE3_FRAME_QUEUE_CAPACITY;
    }

    uint32_t oldest = queue->start_ms[queue->head];
    uint32_t newest = queue->start_ms[
        issue3_frame_queue_index(queue, queue->count - 1U)];
    /* The ring can only append after its current tail.  A delayed batch that
     * reaches behind head would make frame_queue_open() return LATE after the
     * FIFO had already been consumed, so reject it before entering read mode.
     */
    if (start < oldest || end < oldest) return 0;

    if (start <= newest) {
        uint32_t check_end = end < newest ? end : newest;
        for (uint32_t second = start;; second += 1000U) {
            uint8_t found = 0;
            for (uint8_t i = 0; i < queue->count; i++) {
                uint8_t index = issue3_frame_queue_index(queue, i);
                if (queue->start_ms[index] == second) {
                    /* A timestamp match is not sufficient while the SD
                     * writer owns that physical slot.  Batch callers must
                     * refuse to enter FIFO read mode in this case; otherwise
                     * frame_add_* will consume samples that cannot be
                     * appended without overwriting bytes being written. */
                    if (queue->pending_valid &&
                        index == queue->pending_slot)
                        return 0;
                    found = 1;
                    break;
                }
            }
            if (!found) return 0;
            if (second == check_end) break;
        }
    }

    if (end > newest) {
        uint32_t append_start = newest + 1000U;
        uint32_t missing = (end - append_start) / 1000U + 1U;
        /* issue3_frame_queue_open() fills every missing second between the
         * current tail and a delayed timestamp.  Reserve all such slots plus
         * the SD-owned pending slot, not only the batch's terminal second.
         */
        if ((uint32_t)queue->count + reserved + missing >
            ISSUE3_FRAME_QUEUE_CAPACITY) return 0;
    }
    return 1;
}

/* A span that begins before the retained queue head can never be appended.
 * Keep this predicate separate from the general capacity check so the
 * firmware can distinguish a temporarily full queue from an already-late
 * FIFO batch and perform the documented FIFO reset/re-anchor recovery. */
static inline uint8_t issue3_frame_queue_span_before_head(
    const Issue3FrameQueue *queue, uint32_t first_sample_ms,
    uint32_t last_sample_ms)
{
    if (queue->count == 0) return 0;
    uint32_t start = first_sample_ms - (first_sample_ms % 1000U);
    uint32_t end = last_sample_ms - (last_sample_ms % 1000U);
    uint32_t oldest = queue->start_ms[queue->head];
    return (start < oldest || end < oldest) ? 1U : 0U;
}

/* A producer may safely abandon a sample only when its logical second is
 * already older than the retained head, or when the physical slot that holds
 * that second is still owned by the SD writer.  A future second that merely
 * cannot fit yet is not stale: callers must leave it queued and retry after
 * the pending transaction releases its slot.  Keeping this distinction in
 * the shared core prevents GZP/ICP from treating temporary back-pressure as
 * an instruction to overwrite a frame or silently lose a future sample. */
static inline uint8_t issue3_frame_queue_timestamp_unrecoverable(
    const Issue3FrameQueue *queue, uint32_t timestamp_ms)
{
    if (!queue) return 0U;
    uint32_t start = timestamp_ms - (timestamp_ms % 1000U);
    if (queue->pending_valid && queue->pending_slot <
        ISSUE3_FRAME_QUEUE_CAPACITY &&
        queue->start_ms[queue->pending_slot] == start)
        return 1U;
    if (queue->count == 0U) return 0U;
    uint32_t oldest = queue->start_ms[queue->head];
    if (start < oldest) return 1U;
    return 0U;
}

/* Return whether the physical slot that an admission would actually need for
 * this timestamp is the one currently owned by the SD writer.  Merely having
 * some pending slot is not enough: existing safe frames and unrelated queue
 * capacity stalls are deliberately classified separately. */
static inline uint8_t issue3_frame_queue_target_hits_pending(
    const Issue3FrameQueue *queue, uint32_t timestamp_ms)
{
    if (!queue || !queue->pending_valid ||
        queue->pending_slot >= ISSUE3_FRAME_QUEUE_CAPACITY) return 0U;
    uint32_t start = timestamp_ms - (timestamp_ms % 1000U);
    for (uint8_t i = 0U; i < queue->count; i++) {
        uint8_t index = issue3_frame_queue_index(queue, i);
        if (queue->start_ms[index] == start)
            return (uint8_t)(index == queue->pending_slot);
    }
    if (queue->count == 0U)
        return (uint8_t)(queue->head == queue->pending_slot);
    uint32_t oldest = queue->start_ms[queue->head];
    uint8_t tail = issue3_frame_queue_index(queue, queue->count - 1U);
    uint32_t newest = queue->start_ms[tail];
    if (start <= newest || start < oldest) return 0U;
    uint32_t missing = (start - newest) / 1000U;
    uint32_t limit = missing > ISSUE3_FRAME_QUEUE_CAPACITY ?
                     ISSUE3_FRAME_QUEUE_CAPACITY : missing;
    for (uint32_t k = 0U; k < limit; k++) {
        uint8_t index = (uint8_t)((queue->head + queue->count + k) %
                                  ISSUE3_FRAME_QUEUE_CAPACITY);
        if (index == queue->pending_slot) return 1U;
    }
    return 0U;
}

/* Classify one timestamp without changing queue state.  Producers use this
 * when an already-unrecoverable point is explicitly discarded, so OWNER is
 * reported only when this point's logical second maps to the SD-owned slot;
 * an unrelated pending transaction is never enough. */
static inline Issue3FrameOpenReason issue3_frame_queue_timestamp_reason(
    const Issue3FrameQueue *queue, uint32_t timestamp_ms)
{
    if (!queue) return ISSUE3_FRAME_OPEN_REASON_QUEUE;
    uint32_t start = timestamp_ms - (timestamp_ms % 1000U);
    for (uint8_t i = 0U; i < queue->count; i++) {
        uint8_t index = issue3_frame_queue_index(queue, i);
        if (queue->start_ms[index] == start)
            return (queue->pending_valid && index == queue->pending_slot) ?
                   ISSUE3_FRAME_OPEN_REASON_OWNER :
                   ISSUE3_FRAME_OPEN_REASON_NONE;
    }
    if (queue->count == 0U)
        return issue3_frame_queue_target_hits_pending(queue, timestamp_ms) ?
               ISSUE3_FRAME_OPEN_REASON_OWNER : ISSUE3_FRAME_OPEN_REASON_QUEUE;
    if (start < queue->start_ms[queue->head])
        return ISSUE3_FRAME_OPEN_REASON_TIME;
    uint8_t tail = issue3_frame_queue_index(queue, queue->count - 1U);
    if (start <= queue->start_ms[tail])
        return ISSUE3_FRAME_OPEN_REASON_TIME;
    return issue3_frame_queue_target_hits_pending(queue, timestamp_ms) ?
           ISSUE3_FRAME_OPEN_REASON_OWNER : ISSUE3_FRAME_OPEN_REASON_QUEUE;
}

static inline void issue3_frame_queue_reason_set(
    Issue3FrameOpenReason *reason, Issue3FrameOpenReason value)
{
    if (reason) *reason = value;
}

/* Derive the complete time span of one FIFO batch before entering CTRL9 read
 * mode.  A valid clock uses its next native sample timestamp; after a loss,
 * the caller supplies the current monotonic time and the batch is anchored
 * backwards by the fixed native period.  Keeping this calculation shared
 * prevents the firmware and host integration test from drifting apart. */
static inline uint8_t issue3_qmi_batch_span_fits(
    const Issue3FrameQueue *queue, uint8_t clock_valid, uint64_t clock_us,
    uint64_t now_us, uint16_t samples, uint32_t *first_ms,
    uint32_t *last_ms)
{
    if (samples == 0) return 1;
    uint64_t age_us = (uint64_t)(samples - 1U) *
                      ISSUE3_QMI_SAMPLE_PERIOD_US;
    /* Startup/re-anchor timestamps may be earlier than one whole FIFO
     * horizon.  Saturate instead of allowing uint64 wrap to manufacture a
     * timestamp near 2^64 and, after conversion, 0xFFFFxxxx milliseconds. */
    uint64_t first_us = clock_valid ? clock_us :
        (now_us >= age_us ? now_us - age_us : 0ULL);
    uint64_t last_us = first_us +
        ((uint64_t)(samples - 1U) * ISSUE3_QMI_SAMPLE_PERIOD_US);
    *first_ms = (uint32_t)(first_us / 1000ULL);
    *last_ms = (uint32_t)(last_us / 1000ULL);
    return issue3_frame_queue_span_fits(queue, *first_ms, *last_ms);
}

/* ICP Mode 3 produces one pressure/temperature tuple every 500 ms.  The
 * service uses this preflight before reading FIFO_DATA so a full frame queue
 * never consumes a batch that cannot be represented. */
static inline uint8_t issue3_icp_batch_span_fits(
    const Issue3FrameQueue *queue, uint32_t now_ms, uint8_t samples,
    uint32_t *first_ms, uint32_t *last_ms)
{
    if (samples == 0) return 1;
    uint32_t age_ms = (uint32_t)(samples - 1U) * 500U;
    *first_ms = now_ms >= age_ms ? now_ms - age_ms : 0U;
    *last_ms = now_ms;
    return issue3_frame_queue_span_fits(queue, *first_ms, *last_ms);
}

/* Plan one bounded ICP FIFO service turn without consuming FIFO_DATA first.
 * The returned discard count is an explicitly unrecoverable old prefix
 * (before the retained head or in the SD-owned physical slot).  After that
 * prefix, admit only the longest contiguous suffix whose complete span fits
 * the current queue.  A future sample blocked solely by capacity/pending
 * ownership remains in hardware for a later turn instead of being mistaken
 * for an old loss. */
static inline void issue3_icp_batch_prefix_plan(
    const Issue3FrameQueue *queue, uint32_t first_sample_ms,
    uint8_t available, uint8_t max_take, uint8_t *discard_prefix,
    uint8_t *admit_count)
{
    uint8_t limit = available < max_take ? available : max_take;
    uint8_t discard = 0U;
    uint8_t admit = 0U;
    while (discard < limit &&
           issue3_frame_queue_timestamp_unrecoverable(
               queue, first_sample_ms + (uint32_t)discard * 500U))
        discard++;

    if (discard < limit) {
        uint32_t admit_first = first_sample_ms +
                               (uint32_t)discard * 500U;
        while ((uint8_t)(discard + admit) < limit) {
            uint32_t candidate = admit_first + (uint32_t)admit * 500U;
            if (issue3_frame_queue_timestamp_unrecoverable(queue,
                                                            candidate) ||
                !issue3_frame_queue_span_fits(queue, admit_first,
                                              candidate))
                break;
            admit++;
        }
    }
    if (discard_prefix) *discard_prefix = discard;
    if (admit_count) *admit_count = admit;
}

static inline int issue3_frame_queue_open_ex(
    Issue3FrameQueue *queue, uint32_t timestamp_ms, uint32_t now_ms,
    Issue3FrameCommitFn commit, void *ctx, uint8_t *slot_index,
    uint8_t *is_new, Issue3FrameOpenReason *reason)
{
    uint32_t start_ms = timestamp_ms - (timestamp_ms % 1000U);
    issue3_frame_queue_reason_set(reason, ISSUE3_FRAME_OPEN_REASON_NONE);
    *is_new = 0;
    for (uint8_t i = 0; i < queue->count; i++) {
        uint8_t index = issue3_frame_queue_index(queue, i);
        if (queue->start_ms[index] == start_ms) {
            /* A committed slot can still be physically owned by the SD
             * writer.  It is not safe to append another sensor sample to
             * that slot, even when its timestamp is already present. */
            if (queue->pending_valid && index == queue->pending_slot) {
                issue3_frame_queue_reason_set(
                    reason, ISSUE3_FRAME_OPEN_REASON_OWNER);
                return ISSUE3_FRAME_OPEN_BLOCKED;
            }
            *slot_index = index;
            return ISSUE3_FRAME_OPEN_EXISTING;
        }
    }
    if (queue->count == 0) {
        if (queue->pending_valid && queue->head == queue->pending_slot) {
            issue3_frame_queue_reason_set(
                reason, ISSUE3_FRAME_OPEN_REASON_OWNER);
            return ISSUE3_FRAME_OPEN_BLOCKED;
        }
        /* Validate the complete append before changing count/start_ms.  This
         * keeps a failed delayed admission from leaving a partially appended
         * (and therefore uninitialised) second in the ring. */
        if (!issue3_frame_queue_span_fits(queue, start_ms, start_ms)) {
            issue3_frame_queue_reason_set(
                reason, issue3_frame_queue_target_hits_pending(queue, start_ms) ?
                ISSUE3_FRAME_OPEN_REASON_OWNER : ISSUE3_FRAME_OPEN_REASON_QUEUE);
            return ISSUE3_FRAME_OPEN_BLOCKED;
        }
        queue->start_ms[queue->head] = start_ms;
        queue->count = 1;
        *slot_index = queue->head;
        *is_new = 1;
        return ISSUE3_FRAME_OPEN_NEW;
    }
    /* The search above found no existing frame.  A target older than the
     * retained head or a hole in the middle of the chronological ring is a
     * genuinely late sample, not a capacity stall. */
    {
        uint32_t oldest = queue->start_ms[queue->head];
        uint8_t newest_index = issue3_frame_queue_index(queue,
                                                        queue->count - 1U);
        uint32_t newest = queue->start_ms[newest_index];
        if (start_ms <= newest && start_ms >= oldest) {
            issue3_frame_queue_reason_set(
                reason, ISSUE3_FRAME_OPEN_REASON_TIME);
            return ISSUE3_FRAME_OPEN_LATE;
        }
        if (start_ms < oldest) {
            issue3_frame_queue_reason_set(
                reason, ISSUE3_FRAME_OPEN_REASON_TIME);
            return ISSUE3_FRAME_OPEN_LATE;
        }
        /* Preflight all missing seconds before mutating the ring.  A false
         * result here means no safe physical slot is available (including a
         * slot still owned by SD), so callers can retry after the pending
         * transaction releases it. */
        if (!issue3_frame_queue_span_fits(queue, start_ms, start_ms)) {
            issue3_frame_queue_reason_set(
                reason, issue3_frame_queue_target_hits_pending(queue, start_ms) ?
                ISSUE3_FRAME_OPEN_REASON_OWNER : ISSUE3_FRAME_OPEN_REASON_QUEUE);
            return ISSUE3_FRAME_OPEN_BLOCKED;
        }
    }
    uint8_t committed_for_this_sample = 0;
    while (queue->count != 0) {
        uint8_t last_index = issue3_frame_queue_index(queue, queue->count - 1U);
        uint32_t last_start = queue->start_ms[last_index];
        if (last_start >= start_ms) break;
        if (queue->count == ISSUE3_FRAME_QUEUE_CAPACITY) {
            /* A committed slot remains owned by SD until all bytes are
             * persisted; never commit another slot while that ownership is
             * outstanding, even if the ring count has fallen below capacity.
             */
            if (queue->pending_valid) {
                issue3_frame_queue_reason_set(
                    reason, queue->pending_slot == queue->head ?
                    ISSUE3_FRAME_OPEN_REASON_OWNER :
                    ISSUE3_FRAME_OPEN_REASON_QUEUE);
                return ISSUE3_FRAME_OPEN_BLOCKED;
            }
            if (committed_for_this_sample) {
                issue3_frame_queue_reason_set(
                    reason, ISSUE3_FRAME_OPEN_REASON_QUEUE);
                return ISSUE3_FRAME_OPEN_BLOCKED;
            }
            if (!issue3_frame_commit_due(queue->start_ms[queue->head], now_ms)) {
                issue3_frame_queue_reason_set(
                    reason, ISSUE3_FRAME_OPEN_REASON_QUEUE);
                return ISSUE3_FRAME_OPEN_BLOCKED;
            }
            (void)issue3_frame_queue_commit_oldest(queue, commit, ctx);
            committed_for_this_sample = 1;
        }
        uint8_t index = issue3_frame_queue_index(queue, queue->count);
        if (queue->pending_valid && index == queue->pending_slot) {
            issue3_frame_queue_reason_set(
                reason, ISSUE3_FRAME_OPEN_REASON_OWNER);
            return ISSUE3_FRAME_OPEN_BLOCKED;
        }
        queue->start_ms[index] = last_start + 1000U;
        queue->count++;
    }
    for (uint8_t i = 0; i < queue->count; i++) {
        uint8_t index = issue3_frame_queue_index(queue, i);
        if (queue->start_ms[index] == start_ms) {
            *slot_index = index;
            *is_new = 1;
            return ISSUE3_FRAME_OPEN_NEW;
        }
    }
    issue3_frame_queue_reason_set(reason, ISSUE3_FRAME_OPEN_REASON_TIME);
    return ISSUE3_FRAME_OPEN_LATE;
}

static inline int issue3_frame_queue_open(Issue3FrameQueue *queue,
                                          uint32_t timestamp_ms,
                                          uint32_t now_ms,
                                          Issue3FrameCommitFn commit,
                                          void *ctx, uint8_t *slot_index,
                                          uint8_t *is_new)
{
    return issue3_frame_queue_open_ex(queue, timestamp_ms, now_ms, commit,
                                      ctx, slot_index, is_new, 0);
}

/* Runtime callers also know whether an SD/FAT/metadata transaction is
 * pending outside the frame queue.  Such work must not turn an already-open
 * non-pending frame into BLOCKED merely because the ring is full; its samples
 * still belong to that frame and can be appended safely.  A genuinely new
 * second at a full queue remains blocked until the pending work releases a
 * slot.  The physical pending-slot check above is still authoritative. */
static inline int issue3_frame_queue_open_runtime_ex(
    Issue3FrameQueue *queue, uint32_t timestamp_ms, uint32_t now_ms,
    uint8_t sd_work_pending, Issue3FrameCommitFn commit, void *ctx,
    uint8_t *slot_index, uint8_t *is_new, Issue3FrameOpenReason *reason)
{
    issue3_frame_queue_reason_set(reason, ISSUE3_FRAME_OPEN_REASON_NONE);
    if (queue->count == ISSUE3_FRAME_QUEUE_CAPACITY && sd_work_pending) {
        uint32_t start_ms = timestamp_ms - (timestamp_ms % 1000U);
        uint8_t existing_safe = 0U;
        for (uint8_t i = 0; i < queue->count; i++) {
            uint8_t index = issue3_frame_queue_index(queue, i);
            if (queue->start_ms[index] == start_ms) {
                if (queue->pending_valid && index == queue->pending_slot) {
                    issue3_frame_queue_reason_set(
                        reason, ISSUE3_FRAME_OPEN_REASON_OWNER);
                    return ISSUE3_FRAME_OPEN_BLOCKED;
                }
                existing_safe = 1U;
                break;
            }
        }
        if (!existing_safe) {
            if (queue->count != 0U) {
                uint32_t oldest = queue->start_ms[queue->head];
                uint8_t tail = issue3_frame_queue_index(
                    queue, queue->count - 1U);
                uint32_t newest = queue->start_ms[tail];
                if (start_ms < oldest || start_ms <= newest) {
                    issue3_frame_queue_reason_set(
                        reason, ISSUE3_FRAME_OPEN_REASON_TIME);
                } else {
                    issue3_frame_queue_reason_set(
                        reason, issue3_frame_queue_target_hits_pending(
                            queue, start_ms) ?
                        ISSUE3_FRAME_OPEN_REASON_OWNER :
                        ISSUE3_FRAME_OPEN_REASON_QUEUE);
                }
            } else {
                issue3_frame_queue_reason_set(
                    reason, issue3_frame_queue_target_hits_pending(
                        queue, start_ms) ? ISSUE3_FRAME_OPEN_REASON_OWNER :
                        ISSUE3_FRAME_OPEN_REASON_QUEUE);
            }
            return ISSUE3_FRAME_OPEN_BLOCKED;
        }
    }
    return issue3_frame_queue_open_ex(queue, timestamp_ms, now_ms, commit,
                                      ctx, slot_index, is_new, reason);
}

static inline int issue3_frame_queue_open_runtime(
    Issue3FrameQueue *queue, uint32_t timestamp_ms, uint32_t now_ms,
    uint8_t sd_work_pending, Issue3FrameCommitFn commit, void *ctx,
    uint8_t *slot_index, uint8_t *is_new)
{
    return issue3_frame_queue_open_runtime_ex(
        queue, timestamp_ms, now_ms, sd_work_pending, commit, ctx,
        slot_index, is_new, 0);
}

/* NF heartbeat scheduling is shared with the firmware so the startup packet
 * and the periodic absolute grid cannot drift from host regression tests.
 * sequence is advanced only by issue3_heartbeat_complete(), after all three
 * physical copies of the in-flight logical heartbeat finish. */
typedef struct {
    uint32_t period_ms;
    uint32_t next_ms;
    uint32_t sequence;
    uint8_t startup_sent;
    uint8_t in_flight;
} Issue3HeartbeatSchedule;

static inline void issue3_heartbeat_schedule_init(
    Issue3HeartbeatSchedule *schedule, uint32_t period_ms)
{
    schedule->period_ms = period_ms;
    schedule->next_ms = period_ms;
    schedule->sequence = 0;
    schedule->startup_sent = 0;
    schedule->in_flight = 0;
}

static inline uint8_t issue3_heartbeat_startup(
    Issue3HeartbeatSchedule *schedule, uint32_t *sequence)
{
    if (schedule->startup_sent || schedule->in_flight) return 0;
    schedule->startup_sent = 1;
    schedule->in_flight = 1;
    *sequence = schedule->sequence;
    return 1;
}

static inline void issue3_heartbeat_complete(
    Issue3HeartbeatSchedule *schedule)
{
    if (!schedule->in_flight) return;
    schedule->in_flight = 0;
    schedule->sequence++;
}

/* One logical heartbeat is broadcast three times without ACK.  The first
 * copy is immediate; the other copies are deliberately spread through the
 * approved 5..15 s and 15..30 s windows.  The hash is deterministic so two
 * devices normally occupy different air-time slots without a runtime random
 * source. */
#define ISSUE3_NF_BROADCAST_COPIES 3U
#define ISSUE3_NF_COPY2_MIN_MS 5000U
#define ISSUE3_NF_COPY2_MAX_MS 14999U
#define ISSUE3_NF_COPY3_MIN_MS 15000U
#define ISSUE3_NF_COPY3_MAX_MS 29999U

static inline void issue3_heartbeat_build_payload(
    uint8_t *msg, uint32_t device_id, uint8_t version, uint8_t message_type,
    uint32_t sequence, uint32_t uptime_ms, uint8_t qmi_status,
    uint8_t icp_status, uint8_t gzp_status, uint8_t mts4_status,
    uint8_t state_flags)
{
    for (uint8_t i = 0; i < 32U; i++) msg[i] = 0U;
    msg[0] = (uint8_t)(device_id >> 16);
    msg[1] = (uint8_t)(device_id >> 8);
    msg[2] = (uint8_t)device_id;
    msg[3] = version;
    msg[4] = message_type;
    msg[5] = (uint8_t)(sequence >> 24);
    msg[6] = (uint8_t)(sequence >> 16);
    msg[7] = (uint8_t)(sequence >> 8);
    msg[8] = (uint8_t)sequence;
    msg[9] = (uint8_t)(uptime_ms >> 24);
    msg[10] = (uint8_t)(uptime_ms >> 16);
    msg[11] = (uint8_t)(uptime_ms >> 8);
    msg[12] = (uint8_t)uptime_ms;
    msg[13] = qmi_status;
    msg[14] = icp_status;
    msg[15] = gzp_status;
    msg[16] = mts4_status;
    msg[17] = state_flags;
}

typedef struct {
    uint32_t sequence;
    uint32_t start_ms;
    uint32_t offset_ms[ISSUE3_NF_BROADCAST_COPIES];
    uint8_t active;
    uint8_t next_copy;
} Issue3NfBroadcast;

static inline uint32_t issue3_nf_copy_offset_ms(
    uint32_t device_id, uint32_t sequence, uint8_t copy_index)
{
    if (copy_index == 0U) return 0U;
    uint32_t h = device_id ^ (sequence * 0x9E3779B9U) ^
                 ((uint32_t)(copy_index + 1U) * 0x85EBCA6BU);
    h ^= h >> 16;
    h *= 0x7FEB352DU;
    h ^= h >> 15;
    if (copy_index == 1U)
        return ISSUE3_NF_COPY2_MIN_MS + (h %
               (ISSUE3_NF_COPY2_MAX_MS - ISSUE3_NF_COPY2_MIN_MS + 1U));
    if (copy_index == 2U)
        return ISSUE3_NF_COPY3_MIN_MS + (h %
               (ISSUE3_NF_COPY3_MAX_MS - ISSUE3_NF_COPY3_MIN_MS + 1U));
    return 0U;
}

static inline uint8_t issue3_nf_broadcast_begin(
    Issue3NfBroadcast *broadcast, uint32_t device_id,
    uint32_t sequence, uint32_t start_ms)
{
    if (broadcast->active) return 0U;
    broadcast->sequence = sequence;
    broadcast->start_ms = start_ms;
    for (uint8_t i = 0; i < ISSUE3_NF_BROADCAST_COPIES; i++)
        broadcast->offset_ms[i] = issue3_nf_copy_offset_ms(device_id,
                                                            sequence, i);
    broadcast->active = 1U;
    broadcast->next_copy = 0U;
    return 1U;
}

static inline uint8_t issue3_nf_broadcast_due(
    const Issue3NfBroadcast *broadcast, uint32_t now_ms,
    uint32_t *sequence, uint8_t *copy_index)
{
    if (!broadcast->active || broadcast->next_copy >= ISSUE3_NF_BROADCAST_COPIES)
        return 0U;
    if (!issue3_time_reached(now_ms,
            broadcast->start_ms + broadcast->offset_ms[broadcast->next_copy]))
        return 0U;
    *sequence = broadcast->sequence;
    *copy_index = broadcast->next_copy;
    return 1U;
}

static inline void issue3_nf_broadcast_complete(Issue3NfBroadcast *broadcast)
{
    if (!broadcast->active) return;
    broadcast->next_copy++;
    if (broadcast->next_copy >= ISSUE3_NF_BROADCAST_COPIES)
        broadcast->active = 0U;
}

/* Emit at most one packet for the current time.  If the loop was asleep or
 * blocked past one or more old grid points, those old points are skipped and
 * the next future grid remains the absolute schedule; no 30-minute drift is
 * introduced by TX time. */
static inline uint8_t issue3_heartbeat_periodic_due(
    Issue3HeartbeatSchedule *schedule, uint32_t now_ms, uint32_t *sequence)
{
    if (schedule->period_ms == 0 || schedule->in_flight ||
        !issue3_time_reached(now_ms, schedule->next_ms)) return 0;
    do {
        schedule->next_ms += schedule->period_ms;
    } while (issue3_time_reached(now_ms, schedule->next_ms));
    schedule->in_flight = 1;
    *sequence = schedule->sequence;
    return 1;
}

typedef struct {
    uint32_t next_ms;
    uint32_t plan_ms;
} Issue3GzpSchedule;

enum {
    ISSUE3_GZP_NOT_DUE = 0,
    ISSUE3_GZP_START = 1,
    ISSUE3_GZP_SKIP = 2,
    ISSUE3_GZP_START_FAILED = -1,
};

/* Decide at most one current sampling turn.  A point that is only slightly
 * late is still measured on its original grid point.  Once the service is
 * late beyond that tolerance, every elapsed grid point is a genuinely missed
 * historical conversion; start one conversion now and stamp its result with
 * this actual start time rather than fabricating an old plan timestamp.  The
 * next deadline always remains on the absolute 100 ms grid and is strictly
 * in the future. */
static inline int issue3_gzp_plan(Issue3GzpSchedule *schedule, uint32_t now_ms,
                                  uint32_t *plan_ms, uint32_t *missed)
{
    *missed = 0;
    if (!issue3_time_reached(now_ms, schedule->next_ms))
        return ISSUE3_GZP_NOT_DUE;

    uint32_t late_ms = now_ms - schedule->next_ms;
    uint32_t older_points = late_ms / ISSUE3_GZP_PERIOD_MS;
    uint32_t candidate_ms = schedule->next_ms +
                            older_points * ISSUE3_GZP_PERIOD_MS;
    uint32_t candidate_late_ms = now_ms - candidate_ms;
    if (candidate_late_ms <= ISSUE3_GZP_START_LATE_LIMIT_MS) {
        *plan_ms = candidate_ms;
        schedule->plan_ms = *plan_ms;
        schedule->next_ms = candidate_ms + ISSUE3_GZP_PERIOD_MS;
        *missed = older_points;
        return ISSUE3_GZP_START;
    }

    uint32_t skipped = older_points + 1U;
    schedule->next_ms = candidate_ms + ISSUE3_GZP_PERIOD_MS;
    *plan_ms = now_ms;
    schedule->plan_ms = *plan_ms;
    *missed = skipped;
    return ISSUE3_GZP_START;
}

/* While a conversion is active, move the absolute grid past every elapsed
 * point without launching historical conversions. */
static inline uint32_t issue3_gzp_skip_pending(Issue3GzpSchedule *schedule,
                                               uint32_t now_ms)
{
    uint32_t missed = 0;
    while (issue3_time_reached(now_ms, schedule->next_ms)) {
        schedule->next_ms += ISSUE3_GZP_PERIOD_MS;
        missed++;
    }
    return missed;
}

typedef int (*Issue3GzpStartFn)(void *ctx);
typedef void (*Issue3GzpMissedFn)(void *ctx, uint32_t count);

/* Shared with the firmware's non-busy main-loop branch.  Older grid points
 * returned alongside START are recorded before the conversion is attempted;
 * a failed start adds exactly one for the current point. */
static inline int issue3_gzp_dispatch(Issue3GzpSchedule *schedule,
                                      uint32_t now_ms,
                                      Issue3GzpStartFn start, void *start_ctx,
                                      Issue3GzpMissedFn note, void *note_ctx,
                                      uint32_t *plan_ms)
{
    uint32_t missed = 0;
    int decision = issue3_gzp_plan(schedule, now_ms, plan_ms, &missed);
    if (decision == ISSUE3_GZP_SKIP) {
        if (note) note(note_ctx, missed);
        return decision;
    }
    if (decision != ISSUE3_GZP_START) return decision;
    if (missed != 0 && note) note(note_ctx, missed);
    if (start && start(start_ctx) == 0) return ISSUE3_GZP_START;
    if (note) note(note_ctx, 1U);
    return ISSUE3_GZP_START_FAILED;
}

/* A completed GZP conversion may wait behind an SD-owned frame slot.  Keep a
 * small bounded result queue so the next 100 ms conversion can still start on
 * its absolute grid instead of turning a single 120 ms sector transaction
 * into a schedule-wide loss.  The payload values travel with the timestamp
 * assigned at conversion start; callers only remove a result after it has
 * entered a frame. */
#define ISSUE3_GZP_RESULT_QUEUE_CAPACITY 8U

typedef struct {
    uint32_t timestamp_ms;
    uint32_t pressure_raw;
    int16_t temperature_raw;
} Issue3GzpResult;

typedef struct {
    Issue3GzpResult items[ISSUE3_GZP_RESULT_QUEUE_CAPACITY];
    uint8_t head;
    uint8_t count;
} Issue3GzpResultQueue;

static inline void issue3_gzp_result_queue_init(Issue3GzpResultQueue *queue)
{
    if (!queue) return;
    queue->head = 0U;
    queue->count = 0U;
}

static inline uint8_t issue3_gzp_result_queue_push(
    Issue3GzpResultQueue *queue, uint32_t timestamp_ms,
    uint32_t pressure_raw, int16_t temperature_raw)
{
    if (!queue || queue->count >= ISSUE3_GZP_RESULT_QUEUE_CAPACITY)
        return 0U;
    uint8_t tail = (uint8_t)((queue->head + queue->count) %
                             ISSUE3_GZP_RESULT_QUEUE_CAPACITY);
    queue->items[tail].timestamp_ms = timestamp_ms;
    queue->items[tail].pressure_raw = pressure_raw;
    queue->items[tail].temperature_raw = temperature_raw;
    queue->count++;
    return 1U;
}

static inline const Issue3GzpResult *issue3_gzp_result_queue_peek(
    const Issue3GzpResultQueue *queue)
{
    if (!queue || queue->count == 0U) return 0;
    return &queue->items[queue->head];
}

static inline void issue3_gzp_result_queue_pop(Issue3GzpResultQueue *queue)
{
    if (!queue || queue->count == 0U) return;
    queue->head = (uint8_t)((queue->head + 1U) %
                            ISSUE3_GZP_RESULT_QUEUE_CAPACITY);
    queue->count--;
}

typedef struct {
    uint8_t valid;
    uint8_t gap_pending;
    uint8_t last_sample_valid;
    uint16_t reanchor_count;
    uint64_t clock_us;
    uint64_t last_sample_us;
} Issue3QmiClock;

/* The QMI watermark normally fills four native samples in about 143 ms at
 * 28.025 Hz.  A 500 ms interval with no successfully read raw sample is
 * therefore a conservative indication that the stream has stopped, while
 * still leaving room for ordinary I2C/scheduler jitter.  Recovery retries are
 * deliberately much slower than the 100 ms TIM3 poll so a failed reset cannot
 * turn the foreground loop into a reconfiguration loop. */
#define ISSUE3_QMI_EMPTY_FIFO_WATCHDOG_MS 500U
#define ISSUE3_QMI_RECOVERY_RETRY_MS 1000U
/* A batch may be read a little later than its nominal FIFO watermark.  Keep
 * that ordinary scheduling jitter on the existing clock, but correct a clock
 * that has fallen more than one watchdog-sized window behind the batch's
 * current-time sampling window.  This is separate from the no-sample
 * watchdog: successful FIFO reads still receive time-axis correction. */
#define ISSUE3_QMI_CLOCK_LAG_TOLERANCE_US \
    ((uint64_t)ISSUE3_QMI_EMPTY_FIFO_WATCHDOG_MS * 1000ULL)

/* QMI8658A Rev-D FIFO registers and the formal stream configuration.  Keep
 * these values in the shared logic header so the production recovery path
 * and its register-level host mock cannot silently drift apart. */
#define ISSUE3_QMI_REG_CTRL1 0x02U
#define ISSUE3_QMI_REG_CTRL2 0x03U
#define ISSUE3_QMI_REG_CTRL3 0x04U
#define ISSUE3_QMI_REG_CTRL6 0x06U
#define ISSUE3_QMI_REG_CTRL7 0x08U
#define ISSUE3_QMI_REG_CTRL8 0x09U
#define ISSUE3_QMI_REG_FIFO_WTM 0x13U
#define ISSUE3_QMI_REG_FIFO_CTRL 0x14U
#define ISSUE3_QMI_REG_FIFO_COUNT_LSB 0x15U
#define ISSUE3_QMI_REG_FIFO_STATUS 0x16U
#define ISSUE3_QMI_CMD_RST_FIFO 0x04U
#define ISSUE3_QMI_CTRL1_CONFIG 0x4CU
#define ISSUE3_QMI_CTRL2_CONFIG 0x28U
#define ISSUE3_QMI_CTRL3_CONFIG 0x58U
#define ISSUE3_QMI_CTRL6_CONFIG 0x00U
#define ISSUE3_QMI_CTRL7_CONFIG 0x03U
#define ISSUE3_QMI_CTRL8_CONFIG 0x80U
#define ISSUE3_QMI_FIFO_WTM_CONFIG 4U
#define ISSUE3_QMI_FIFO_CTRL_CONFIG 0x0EU
#define ISSUE3_QMI_FIFO_CTRL_RD_MODE 0x80U
#define ISSUE3_QMI_FIFO_STATUS_COUNT_MSB_MASK 0x03U
#define ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY 0x10U
#define ISSUE3_QMI_FIFO_STATUS_OVERFLOW 0x20U
#define ISSUE3_QMI_FIFO_STATUS_WTM 0x40U
#define ISSUE3_QMI_FIFO_STATUS_FULL 0x80U
#define ISSUE3_QMI_FIFO_STATUS_RESERVED_MASK 0x0CU
#define ISSUE3_QMI_STATUS_EMPTY_RECOVERY (1UL << 10)
#define ISSUE3_QMI_STATUS_RECOVERY_OK (1UL << 11)
#define ISSUE3_QMI_STATUS_RECOVERY_FAIL (1UL << 12)

typedef struct {
    uint8_t last_sample_valid;
    uint8_t service_seen;
    uint8_t recovery_active;
    uint8_t empty_streak_active; /* compatibility/debug: no-sample window */
    uint32_t last_sample_ms;
    uint32_t last_service_ms;
    uint32_t retry_after_ms;
} Issue3QmiRecovery;

static inline void issue3_qmi_recovery_init(Issue3QmiRecovery *state,
                                            uint32_t now_ms)
{
    if (!state) return;
    state->last_sample_valid = 0U;
    state->service_seen = 0U;
    state->recovery_active = 0U;
    state->empty_streak_active = 0U;
    state->last_sample_ms = now_ms;
    state->last_service_ms = now_ms;
    state->retry_after_ms = now_ms;
}

/* These two compatibility helpers remain available to older host tests, but
 * the production watchdog no longer treats a particular FIFO status as the
 * source of time.  The source of truth is last_sample_ms, updated only after
 * a complete raw sample is read. */
static inline void issue3_qmi_recovery_observe_empty(
    Issue3QmiRecovery *state, uint32_t now_ms)
{
    if (!state) return;
    state->service_seen = 1U;
    state->last_service_ms = now_ms;
    if (!state->empty_streak_active) {
        state->empty_streak_active = 1U;
    }
}

static inline void issue3_qmi_recovery_clear_empty(Issue3QmiRecovery *state)
{
    if (state) state->empty_streak_active = 0U;
}

static inline uint8_t issue3_qmi_recovery_due(
    const Issue3QmiRecovery *state, uint32_t now_ms)
{
    if (!state) return 0U;
    if (state->recovery_active)
        return (uint8_t)((int32_t)(now_ms - state->retry_after_ms) >= 0);
    return (uint8_t)(state->service_seen &&
        (uint32_t)(now_ms - state->last_sample_ms) >=
        ISSUE3_QMI_EMPTY_FIFO_WATCHDOG_MS);
}

/* Arm one recovery attempt and its bounded retry deadline.  A later
 * successful raw sample clears recovery_active; until then this state blocks
 * 100 ms polling from repeatedly resetting the sensor. */
static inline void issue3_qmi_recovery_trigger(Issue3QmiRecovery *state,
                                               uint32_t now_ms)
{
    if (!state) return;
    state->recovery_active = 1U;
    state->empty_streak_active = 0U;
    state->retry_after_ms = now_ms + ISSUE3_QMI_RECOVERY_RETRY_MS;
}

/* Called once per 100 ms QMI service turn, including turns where the status or
 * FIFO_DATA transaction fails.  It arms one recovery attempt after 500 ms
 * without a successful complete raw sample.  Before the first sample,
 * last_sample_ms is the runtime-origin timestamp, so startup also has a
 * bounded 500 ms recovery deadline. */
static inline uint8_t issue3_qmi_recovery_service_tick(
    Issue3QmiRecovery *state, uint32_t now_ms)
{
    if (!state) return 0U;
    state->service_seen = 1U;
    state->last_service_ms = now_ms;
    if (issue3_qmi_recovery_due(state, now_ms)) {
        issue3_qmi_recovery_trigger(state, now_ms);
        return 1U;
    }
    if ((uint32_t)(now_ms - state->last_sample_ms) >=
        ISSUE3_QMI_EMPTY_FIFO_WATCHDOG_MS)
        state->empty_streak_active = 1U;
    return 0U;
}

/* A raw FIFO sample is the only event that proves the stream has recovered.
 * It also provides the running timestamp used by the heartbeat/debugger. */
static inline void issue3_qmi_recovery_note_sample(Issue3QmiRecovery *state,
                                                   uint32_t now_ms)
{
    if (!state) return;
    state->last_sample_valid = 1U;
    state->last_sample_ms = now_ms;
    state->service_seen = 1U;
    state->last_service_ms = now_ms;
    state->empty_streak_active = 0U;
    state->recovery_active = 0U;
}

typedef enum {
    ISSUE3_QMI_FIFO_INCONSISTENT = -1,
    ISSUE3_QMI_FIFO_EMPTY = 0,
    ISSUE3_QMI_FIFO_NONEMPTY = 1,
    ISSUE3_QMI_FIFO_OVERFLOW = 2
} Issue3QmiFifoObservation;

/* The count LSB and FIFO_STATUS are separate I2C reads.  Treat their
 * relationship as a protocol observation rather than inferring emptiness
 * from the count alone.  In particular, NOT_EMPTY must agree with the
 * reconstructed ten-bit word count, and WTM/FULL may not be asserted for an
 * empty FIFO.  Reserved status bits are also rejected so a transient or
 * shifted register read cannot feed the watchdog as a false empty state. */
static inline uint16_t issue3_qmi_fifo_words(uint8_t count_lsb,
                                             uint8_t fifo_status)
{
    return (uint16_t)(((uint16_t)(fifo_status &
                                  ISSUE3_QMI_FIFO_STATUS_COUNT_MSB_MASK)
                       << 8) | count_lsb);
}

static inline Issue3QmiFifoObservation issue3_qmi_fifo_classify(
    uint8_t count_lsb, uint8_t fifo_status)
{
    uint16_t words;
    uint8_t not_empty;

    if (fifo_status & ISSUE3_QMI_FIFO_STATUS_RESERVED_MASK)
        return ISSUE3_QMI_FIFO_INCONSISTENT;
    words = issue3_qmi_fifo_words(count_lsb, fifo_status);
    /* Overflow is an explicit hardware loss condition, not a valid empty
     * observation.  Let the existing bounded overflow reset path handle it. */
    if (fifo_status & ISSUE3_QMI_FIFO_STATUS_OVERFLOW)
        return ISSUE3_QMI_FIFO_OVERFLOW;
    not_empty = (uint8_t)((fifo_status &
                           ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY) != 0U);
    if ((uint8_t)(words != 0U) != not_empty)
        return ISSUE3_QMI_FIFO_INCONSISTENT;
    if (words == 0U && (fifo_status &
                        (ISSUE3_QMI_FIFO_STATUS_WTM |
                         ISSUE3_QMI_FIFO_STATUS_FULL)))
        return ISSUE3_QMI_FIFO_INCONSISTENT;
    return words == 0U ? ISSUE3_QMI_FIFO_EMPTY : ISSUE3_QMI_FIFO_NONEMPTY;
}

/* Shared FIFO observation/timeout decision.  The caller performs the
 * register transaction and then invokes this exact helper in both firmware
 * and host tests.  It preserves a due decision made before the register
 * reads, so an I2C/status contradiction cannot erase the no-sample timeout. */
static inline Issue3QmiFifoObservation issue3_qmi_recovery_observe_fifo(
    Issue3QmiRecovery *state, uint8_t count_lsb, uint8_t fifo_status,
    uint32_t now_ms, uint8_t *recovery_due)
{
    Issue3QmiFifoObservation observation = issue3_qmi_fifo_classify(
        count_lsb, fifo_status);
    uint8_t due = recovery_due ? *recovery_due : 0U;
    due |= issue3_qmi_recovery_service_tick(state, now_ms);
    if (recovery_due) *recovery_due = due;
    return observation;
}

typedef int (*Issue3QmiWriteRegFn)(void *ctx, uint8_t reg, uint8_t value);
typedef int (*Issue3QmiReadRegFn)(void *ctx, uint8_t reg, uint8_t *value);
typedef int (*Issue3QmiCtrl9Fn)(void *ctx, uint8_t command);

typedef struct {
    void *ctx;
    Issue3QmiWriteRegFn write_reg;
    Issue3QmiReadRegFn read_reg;
    Issue3QmiCtrl9Fn ctrl9;
} Issue3QmiRecoveryIo;

static inline int issue3_qmi_recovery_write_verify(
    const Issue3QmiRecoveryIo *io, uint8_t reg, uint8_t expected,
    uint8_t mask)
{
    uint8_t actual = 0U;
    if (io->write_reg(io->ctx, reg, expected) < 0 ||
        io->read_reg(io->ctx, reg, &actual) < 0)
        return -1;
    return ((actual & mask) == (expected & mask)) ? 0 : -1;
}

/* Execute the formal production recovery sequence.  It intentionally owns
 * both the command order and the post-reset checks: tests provide only a
 * controlled register/I2C mock through Issue3QmiRecoveryIo. */
static inline int issue3_qmi_recovery_execute(
    const Issue3QmiRecoveryIo *io)
{
    uint8_t fifo_ctrl = 0U, count_lsb = 0U, fifo_status = 0U;
    if (!io || !io->write_reg || !io->read_reg || !io->ctrl9)
        return -1;
    /* Exit FIFO_RD_MODE before issuing the documented FIFO reset command. */
    if (io->write_reg(io->ctx, ISSUE3_QMI_REG_FIFO_CTRL,
                      ISSUE3_QMI_FIFO_CTRL_CONFIG) < 0 ||
        io->ctrl9(io->ctx, ISSUE3_QMI_CMD_RST_FIFO) < 0)
        return -1;
    if (issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_CTRL1, ISSUE3_QMI_CTRL1_CONFIG, 0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_CTRL2, ISSUE3_QMI_CTRL2_CONFIG, 0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_CTRL3, ISSUE3_QMI_CTRL3_CONFIG, 0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_CTRL6, ISSUE3_QMI_CTRL6_CONFIG, 0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_CTRL7, ISSUE3_QMI_CTRL7_CONFIG, 0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_CTRL8, ISSUE3_QMI_CTRL8_CONFIG, 0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_FIFO_WTM, ISSUE3_QMI_FIFO_WTM_CONFIG,
            0xFFU) < 0 ||
        issue3_qmi_recovery_write_verify(
            io, ISSUE3_QMI_REG_FIFO_CTRL, ISSUE3_QMI_FIFO_CTRL_CONFIG,
            0x0FU) < 0)
        return -1;
    if (io->read_reg(io->ctx, ISSUE3_QMI_REG_FIFO_CTRL, &fifo_ctrl) < 0 ||
        (fifo_ctrl & ISSUE3_QMI_FIFO_CTRL_RD_MODE) != 0U ||
        (fifo_ctrl & 0x0FU) != ISSUE3_QMI_FIFO_CTRL_CONFIG)
        return -1;
    /* Count LSB and status are deliberately read independently, matching
     * the production service and allowing the strict consistency checks. */
    if (io->read_reg(io->ctx, ISSUE3_QMI_REG_FIFO_COUNT_LSB, &count_lsb) < 0 ||
        io->read_reg(io->ctx, ISSUE3_QMI_REG_FIFO_STATUS, &fifo_status) < 0)
        return -1;
    return issue3_qmi_fifo_classify(count_lsb, fifo_status) ==
           ISSUE3_QMI_FIFO_EMPTY ? 0 : -1;
}

/* Persistent frame status is event-accumulating.  A later successful retry
 * must never erase a failure observed earlier in the same frame (and vice
 * versa). */
static inline uint32_t issue3_qmi_recovery_status_or(
    uint32_t frame_status, uint8_t recovery_ok, uint8_t recovery_fail)
{
    if (recovery_ok) frame_status |= ISSUE3_QMI_STATUS_RECOVERY_OK;
    if (recovery_fail) frame_status |= ISSUE3_QMI_STATUS_RECOVERY_FAIL;
    return frame_status;
}

static inline void issue3_qmi_clock_invalidate(Issue3QmiClock *clock)
{
    /* Count one diagnostic for each newly observed discontinuity, including
     * a failure before the first valid anchor.  Repeated cleanup calls while
     * the gap is already pending must not inflate the counter. */
    if ((clock->valid || !clock->gap_pending) &&
        clock->reanchor_count != 0xFFFFU)
        clock->reanchor_count++;
    clock->valid = 0;
    clock->gap_pending = 1;
}

static inline void issue3_qmi_clock_anchor(Issue3QmiClock *clock,
                                           uint64_t now_us, uint16_t samples)
{
    if (samples == 0) return;
    uint64_t age_us = (uint64_t)(samples - 1U) *
                      ISSUE3_QMI_SAMPLE_PERIOD_US;
    clock->clock_us = now_us >= age_us ? now_us - age_us : 0ULL;
    clock->valid = 1;
}

/* Prepare the next successfully readable FIFO batch against the current
 * monotonic time.  clock_us is the timestamp of the next native sample, so a
 * batch of N samples read at now_us has a reasonable earliest timestamp of
 * now_us - (N - 1) * nominal_period.  Only a clearly stale clock is advanced;
 * small 100 ms scheduling jitter remains on the free-running clock and does
 * not create a re-anchor on every batch.  Advancing is always forward, so
 * output sample times stay monotonic; the batch anchor guarantees its final
 * sample is no later than the read time. */
static inline uint8_t issue3_qmi_clock_prepare_batch(
    Issue3QmiClock *clock, uint64_t batch_now_us, uint16_t samples)
{
    if (!clock || samples == 0U) return 0U;
    uint64_t age_us = (uint64_t)(samples - 1U) *
                      ISSUE3_QMI_SAMPLE_PERIOD_US;
    uint64_t batch_anchor_us = batch_now_us >= age_us ?
                               batch_now_us - age_us : 0ULL;
    if (!clock->valid) {
        issue3_qmi_clock_anchor(clock, batch_now_us, samples);
        return 0U;
    }
    if (batch_anchor_us > clock->clock_us &&
        batch_anchor_us - clock->clock_us >
            ISSUE3_QMI_CLOCK_LAG_TOLERANCE_US) {
        issue3_qmi_clock_invalidate(clock);
        issue3_qmi_clock_anchor(clock, batch_now_us, samples);
        return 1U;
    }
    /* A variable FIFO batch can otherwise make the free-running next time
     * briefly extend past this read's current time.  Move only that small
     * scheduling artifact to the batch window, without marking a data gap;
     * issue3_qmi_clock_sample_ms() still enforces monotonicity against the
     * last emitted raw timestamp. */
    if (clock->clock_us + age_us > batch_now_us)
        clock->clock_us = batch_anchor_us;
    return 0U;
}

static inline uint32_t issue3_qmi_clock_sample_ms(Issue3QmiClock *clock)
{
    uint64_t sample_us = clock->clock_us;
    if (clock->last_sample_valid && sample_us <= clock->last_sample_us)
        sample_us = clock->last_sample_us + 1ULL;
    clock->last_sample_valid = 1U;
    clock->last_sample_us = sample_us;
    clock->clock_us = sample_us + ISSUE3_QMI_SAMPLE_PERIOD_US;
    uint32_t sample_ms = (uint32_t)(sample_us / 1000ULL);
    return sample_ms;
}

/* Clear a pending gap only after an entire FIFO batch has been read and its
 * read mode cleanup has succeeded.  Partial batches must leave the gap
 * visible so the next complete batch is explicitly re-anchored. */
static inline void issue3_qmi_clock_complete_batch(Issue3QmiClock *clock)
{
    if (clock->valid) clock->gap_pending = 0;
}

typedef int (*Issue3FatReadFn)(void *ctx, uint32_t lba, uint8_t *buf);

enum {
    ISSUE3_FAT_FOUND = 1,
    ISSUE3_FAT_BUDGET = 0,
    ISSUE3_FAT_FULL = -1,
    ISSUE3_FAT_IO = -2,
};

typedef struct {
    uint32_t cursor;
    uint32_t scanned_clusters;
    uint32_t cached_lba;
    uint8_t complete;
    uint8_t cache_valid;
} Issue3FatScan;

/* The scan cache is backed by a caller-owned scratch sector.  That sector is
 * also used by the firmware for root-directory and other metadata I/O, so a
 * caller must invalidate this bit whenever it lends the buffer to another
 * operation.  Re-reading one FAT sector is preferable to interpreting
 * directory/text bytes as FAT entries. */
static inline void issue3_fat_scan_cache_invalidate(Issue3FatScan *scan)
{
    if (!scan) return;
    scan->cache_valid = 0U;
    scan->cached_lba = 0U;
}

typedef int (*Issue3ClusterAllocFn)(void *ctx, uint32_t *cluster);

/* Startup allocation may run before formal acquisition begins, so it can
 * finish all resumable FAT budgets.  Runtime callers use one step instead. */
static inline int issue3_fat_alloc_until_terminal(Issue3ClusterAllocFn allocate,
                                                  void *ctx,
                                                  uint32_t *cluster)
{
    int result;
    do {
        result = allocate(ctx, cluster);
    } while (result == ISSUE3_FAT_BUDGET);
    return result;
}

enum {
    ISSUE3_LOG_PREFETCH_IDLE = 0,
    ISSUE3_LOG_PREFETCH_READY = 1,
    ISSUE3_LOG_PREFETCH_BUDGET = 2,
    ISSUE3_LOG_PREFETCH_FULL = -1,
    ISSUE3_LOG_PREFETCH_IO = -2,
    ISSUE3_LOG_NEEDS_SUCCESSOR = -3,
};

typedef struct {
    uint8_t active;
    uint8_t next_ready;
    uint8_t allocation_pending;
    uint8_t sd_full;
    uint32_t current_cluster;
    uint32_t next_cluster;
    uint16_t sectors_per_cluster;
    uint16_t sector_index;
} Issue3LogPrefetch;

typedef int (*Issue3SectorWriteFn)(void *ctx, uint32_t lba,
                                   const uint8_t *sector);

typedef struct {
    uint16_t offset;
    uint32_t total_written;
} Issue3SectorBuffer;

/* README content and FAT 8.3 naming are shared with the host regression so
 * the cross-sector boundary is tested against the exact bytes emitted by the
 * firmware, rather than a second copy of the text or padding rules. */
static inline const char *issue3_readme_text(void)
{
    return
"PCB32 LOG FORMAT V4\r\n"
"Frame size: 368 bytes, little-endian; CRC32 at offset 364\r\n"
"Header: magic,u16 version,u16 header_size,u32 frame_seq,u32 frame_start_ms,\r\n"
"  u8 qmi_count,u8 gzp_count,u8 icp_count,u8 mts_valid,u32 status,\r\n"
"  u16 qmi_fifo_dropped,u16 qmi_fifo_overflows,u16 qmi_reanchors,u16 gzp_missed\r\n"
"QMI: 16 slots x (u32 sample_ms + 6 x int16 raw); raw FIFO is ~28.025 Hz, saved stream is 1/2\r\n"
"GZP: 5 slots x (u32 sample_ms + int24 pressure + int16 temp); raw grid is 10 Hz, saved stream is 1/2\r\n"
"ICP: 2 slots x (u32 sample_ms + 20-bit pressure + 20-bit temp)\r\n"
"MTS4: (u32 sample_ms + int16 temp_x256)\r\n"
"QMI/GZP are read at their native rates; after successful reads every other raw sample/result is saved without interpolation or feature extraction.\r\n";
}

static inline uint32_t issue3_readme_length(void)
{
    const char *text = issue3_readme_text();
    uint32_t length = 0U;
    while (text[length] != '\0') length++;
    return length;
}

static inline uint32_t issue3_readme_sector_count(void)
{
    uint32_t length = issue3_readme_length();
    return (length + ISSUE3_README_SECTOR_BYTES - 1U) /
           ISSUE3_README_SECTOR_BYTES;
}

static inline void issue3_readme_name(uint8_t *name11)
{
    const uint8_t name[11] = {'R','E','A','D','M','E',' ',' ','T','X','T'};
    for (uint8_t i = 0U; i < 11U; i++) name11[i] = name[i];
}

static inline uint16_t issue3_readme_copy_sector(uint8_t *destination,
                                                 uint32_t sector)
{
    uint32_t length = issue3_readme_length();
    uint32_t offset = sector * ISSUE3_README_SECTOR_BYTES;
    if (offset >= length) return 0U;
    uint32_t left = length - offset;
    uint16_t count = (uint16_t)(left > ISSUE3_README_SECTOR_BYTES ?
                                ISSUE3_README_SECTOR_BYTES : left);
    const char *text = issue3_readme_text();
    for (uint16_t i = 0U; i < ISSUE3_README_SECTOR_BYTES; i++)
        destination[i] = i < count ? (uint8_t)text[offset + i] : 0U;
    return count;
}

enum {
    ISSUE3_README_OK = 0,
    ISSUE3_README_FULL = -1,
    ISSUE3_README_IO = -2,
    ISSUE3_README_DIRECTORY_FULL = -3,
    ISSUE3_README_INVALID = -4,
    ISSUE3_README_ROLLBACK_IO = -5,
    ISSUE3_README_COMMITTED_CLEANUP_IO = -6,
};

typedef int (*Issue3ReadmeRootReadFn)(void *ctx, uint8_t *sector);
typedef int (*Issue3ReadmeRootWriteFn)(void *ctx, const uint8_t *sector);
typedef int (*Issue3ReadmeFatSetFn)(void *ctx, uint32_t cluster,
                                    uint32_t value);
typedef int (*Issue3ReadmeFreeChainFn)(void *ctx, uint32_t first_cluster);
typedef int (*Issue3ReadmeDataWriteFn)(void *ctx, uint32_t cluster,
                                       uint32_t sector_in_cluster,
                                       const uint8_t *sector);

typedef struct {
    Issue3ReadmeRootReadFn read_root;
    Issue3ReadmeRootWriteFn write_root;
    Issue3ClusterAllocFn allocate;
    Issue3ReadmeFatSetFn set_fat;
    Issue3ReadmeFreeChainFn free_chain;
    Issue3ReadmeDataWriteFn write_data;
    void *ctx;
    uint32_t first_cluster;
    uint32_t cluster_count;
    uint16_t sectors_per_cluster;
} Issue3ReadmeStorage;

/* Transactionally replace or create README.TXT.  The directory is committed
 * only after every new data sector and FAT link is durable.  Startup FAT
 * BUDGET results are resumed here until FOUND/FULL/IO, so occupied leading
 * FAT sectors cannot be mistaken for a full card.  On a pre-commit failure
 * every newly allocated cluster is released; on a directory-write failure
 * the exact old 32-byte entry is restored before the new chain is released.
 * The old reachable chain is reclaimed only after the new entry commits. */
static inline int issue3_readme_write_transaction(
    const Issue3ReadmeStorage *storage, uint8_t *root_sector,
    uint8_t *data_sector)
{
    if (!storage || !root_sector || !data_sector || !storage->read_root ||
        !storage->write_root || !storage->allocate || !storage->set_fat ||
        !storage->free_chain || !storage->write_data ||
        storage->sectors_per_cluster == 0U || storage->cluster_count == 0U)
        return ISSUE3_README_INVALID;

    uint32_t sector_count = issue3_readme_sector_count();
    uint32_t cluster_needed =
        (sector_count + storage->sectors_per_cluster - 1U) /
        storage->sectors_per_cluster;
    if (sector_count == 0U || cluster_needed == 0U || cluster_needed > 4U)
        return ISSUE3_README_INVALID;

    if (storage->read_root(storage->ctx, root_sector) != 0)
        return ISSUE3_README_IO;

    uint8_t name11[11];
    issue3_readme_name(name11);
    uint16_t entry_offset = 0xFFFFU;
    uint8_t old_entry[32];
    for (uint16_t off = 0U; off < ISSUE3_README_SECTOR_BYTES; off += 32U) {
        uint8_t match = 1U;
        for (uint8_t i = 0U; i < 11U; i++)
            if (root_sector[off + i] != name11[i]) match = 0U;
        if (root_sector[off] != 0x00U && root_sector[off] != 0xE5U && match) {
            entry_offset = off;
            break;
        }
        if ((root_sector[off] == 0x00U || root_sector[off] == 0xE5U) &&
            entry_offset == 0xFFFFU)
            entry_offset = off;
    }
    if (entry_offset == 0xFFFFU) return ISSUE3_README_DIRECTORY_FULL;
    for (uint8_t i = 0U; i < 32U; i++)
        old_entry[i] = root_sector[entry_offset + i];

    uint8_t entry_new = old_entry[0] == 0x00U || old_entry[0] == 0xE5U;
    uint32_t old_cluster = ((uint32_t)old_entry[20] << 16) |
                           ((uint32_t)old_entry[21] << 24) |
                           (uint32_t)old_entry[26] |
                           ((uint32_t)old_entry[27] << 8);
    if (entry_new) old_cluster = 0U;

    uint32_t chain[4];
    for (uint8_t i = 0U; i < 4U; i++) chain[i] = 0U;
    uint32_t allocated = 0U;
    int failure = ISSUE3_README_IO;
    for (; allocated < cluster_needed; allocated++) {
        int result = issue3_fat_alloc_until_terminal(
            storage->allocate, storage->ctx, &chain[allocated]);
        if (result != ISSUE3_FAT_FOUND) {
            failure = result == ISSUE3_FAT_FULL ? ISSUE3_README_FULL :
                      ISSUE3_README_IO;
            goto release_new_chain;
        }
    }

    for (uint32_t i = 0U; i + 1U < allocated; i++) {
        if (storage->set_fat(storage->ctx, chain[i], chain[i + 1U]) != 0)
            goto release_new_chain;
    }

    for (uint32_t sector = 0U; sector < sector_count; sector++) {
        (void)issue3_readme_copy_sector(data_sector, sector);
        uint32_t chain_index = sector / storage->sectors_per_cluster;
        uint32_t in_cluster = sector % storage->sectors_per_cluster;
        if (storage->write_data(storage->ctx, chain[chain_index], in_cluster,
                                data_sector) != 0)
            goto release_new_chain;
    }

    if (storage->read_root(storage->ctx, root_sector) != 0)
        goto release_new_chain;
    if (entry_new) {
        for (uint8_t i = 0U; i < 32U; i++)
            root_sector[entry_offset + i] = 0U;
        for (uint8_t i = 0U; i < 11U; i++)
            root_sector[entry_offset + i] = name11[i];
        root_sector[entry_offset + 11U] = 0x20U;
    }
    root_sector[entry_offset + 20U] = (uint8_t)(chain[0] >> 16);
    root_sector[entry_offset + 21U] = (uint8_t)(chain[0] >> 24);
    root_sector[entry_offset + 26U] = (uint8_t)chain[0];
    root_sector[entry_offset + 27U] = (uint8_t)(chain[0] >> 8);
    uint32_t length = issue3_readme_length();
    root_sector[entry_offset + 28U] = (uint8_t)length;
    root_sector[entry_offset + 29U] = (uint8_t)(length >> 8);
    root_sector[entry_offset + 30U] = (uint8_t)(length >> 16);
    root_sector[entry_offset + 31U] = (uint8_t)(length >> 24);
    if (storage->write_root(storage->ctx, root_sector) != 0) {
        /* The failed write may have changed any subset of the entry.  Keep
         * the new chain allocated until the exact old entry is durable. */
        if (storage->read_root(storage->ctx, root_sector) != 0)
            return ISSUE3_README_ROLLBACK_IO;
        for (uint8_t i = 0U; i < 32U; i++)
            root_sector[entry_offset + i] = old_entry[i];
        if (storage->write_root(storage->ctx, root_sector) != 0)
            return ISSUE3_README_ROLLBACK_IO;
        failure = ISSUE3_README_IO;
        goto release_new_chain;
    }

    if (old_cluster >= storage->first_cluster &&
        old_cluster < storage->first_cluster + storage->cluster_count &&
        old_cluster != chain[0] &&
        storage->free_chain(storage->ctx, old_cluster) != 0)
        return ISSUE3_README_COMMITTED_CLEANUP_IO;
    return ISSUE3_README_OK;

release_new_chain:
    for (uint32_t i = 0U; i < allocated; i++)
        if (chain[i] != 0U &&
            storage->set_fat(storage->ctx, chain[i], 0U) != 0)
            return ISSUE3_README_ROLLBACK_IO;
    return failure;
}

/* Append one bounded chunk to the shared 512-byte sector path.  At most one
 * physical sector write occurs per call; callers schedule another call for
 * the next chunk, which is what keeps the firmware's SD path interruptible. */
static inline int issue3_sector_buffer_append(Issue3SectorBuffer *state,
                                              uint8_t *sector,
                                              const uint8_t *data,
                                              uint16_t length,
                                              uint32_t lba,
                                              Issue3SectorWriteFn write_sector,
                                              void *ctx)
{
    if (length > (uint16_t)(ISSUE3_SECTOR_BYTES - state->offset))
        return -2;
    for (uint16_t i = 0; i < length; i++)
        sector[state->offset + i] = data[i];
    state->offset = (uint16_t)(state->offset + length);
    if (state->offset != ISSUE3_SECTOR_BYTES) return 0;
    if (!write_sector || write_sector(ctx, lba, sector) != 0) return -1;
    state->total_written += ISSUE3_SECTOR_BYTES;
    state->offset = 0;
    return 1;
}

/* The firmware calls this small step after servicing QMI.  It never scans
 * more than the allocator's configured FAT budget and preserves BUDGET as a
 * resumable state instead of stopping the log. */
static inline int issue3_log_prefetch_step(Issue3LogPrefetch *state,
                                           Issue3ClusterAllocFn allocate,
                                           void *ctx)
{
    if (!state->active || state->next_ready) return ISSUE3_LOG_PREFETCH_IDLE;
    uint32_t cluster = 0;
    int result = allocate(ctx, &cluster);
    if (result == ISSUE3_FAT_FOUND) {
        state->next_cluster = cluster;
        state->next_ready = 1;
        state->allocation_pending = 0;
        return ISSUE3_LOG_PREFETCH_READY;
    }
    if (result == ISSUE3_FAT_BUDGET) {
        state->allocation_pending = 1;
        return ISSUE3_LOG_PREFETCH_BUDGET;
    }
    if (result == ISSUE3_FAT_FULL) {
        state->sd_full = 1;
        return ISSUE3_LOG_PREFETCH_FULL;
    }
    state->allocation_pending = 1;
    return ISSUE3_LOG_PREFETCH_IO;
}

static inline int issue3_log_prefetch_advance_sector(Issue3LogPrefetch *state)
{
    state->sector_index++;
    if (state->sector_index < state->sectors_per_cluster)
        return ISSUE3_LOG_PREFETCH_IDLE;
    if (!state->next_ready) {
        state->allocation_pending = 1;
        return ISSUE3_LOG_NEEDS_SUCCESSOR;
    }
    state->current_cluster = state->next_cluster;
    state->next_cluster = 0;
    state->next_ready = 0;
    state->allocation_pending = 0;
    state->sector_index = 0;
    return ISSUE3_LOG_PREFETCH_READY;
}

/* Complete a cluster transition that reached the boundary before the
 * background allocator found a successor.  The FAT link write is performed
 * by the caller; this shared step only advances the resumable log state. */
static inline int issue3_log_prefetch_link_ready(Issue3LogPrefetch *state)
{
    if (!state->next_ready || state->sector_index < state->sectors_per_cluster)
        return 0;
    state->current_cluster = state->next_cluster;
    state->next_cluster = 0;
    state->next_ready = 0;
    state->allocation_pending = 0;
    state->sector_index = 0;
    return 1;
}

/* Scan at most sector_budget FAT sectors and retain the cursor across calls.
 * A full result is only possible after every data-cluster entry was checked;
 * a read failure is never reported as full. */
static inline int issue3_fat_scan_step(Issue3FatScan *scan,
                                       uint32_t first_cluster,
                                       uint32_t cluster_count,
                                       uint32_t fat0_lba,
                                       uint32_t sector_budget,
                                       Issue3FatReadFn read_sector,
                                       void *ctx, uint8_t *buf,
                                       uint32_t *found_cluster)
{
    if (cluster_count == 0) return ISSUE3_FAT_FULL;
    if (scan->complete) return ISSUE3_FAT_FULL;
    if (scan->cursor < first_cluster ||
        scan->cursor >= first_cluster + cluster_count)
        scan->cursor = first_cluster;

    for (uint32_t s = 0; s < sector_budget; s++) {
        uint32_t lba = fat0_lba + (scan->cursor * 4U) / 512U;
        uint32_t sector_first = ((scan->cursor * 4U) / 512U) * 128U;
        uint32_t begin = scan->cursor > sector_first ? scan->cursor : sector_first;
        uint32_t end = sector_first + 128U;
        uint32_t limit = first_cluster + cluster_count;
        if (end > limit) end = limit;
        if (begin >= end) {
            scan->cursor = first_cluster;
            scan->scanned_clusters = 0;
            continue;
        }
        /* Adjacent allocations commonly land in the same FAT sector.  Keep
         * that sector across bounded service calls so a free run does not
         * spend one full worst-case SD read per cluster.  The cursor still
         * advances and every call retains the explicit sector budget. */
        if (!scan->cache_valid || scan->cached_lba != lba) {
            if (read_sector(ctx, lba, buf) != 0) {
                scan->cache_valid = 0;
                return ISSUE3_FAT_IO;
            }
            scan->cached_lba = lba;
            scan->cache_valid = 1;
        }
        for (uint32_t c = begin; c < end; c++) {
            uint16_t off = (uint16_t)((c * 4U) & 511U);
            uint32_t value = (uint32_t)buf[off] |
                ((uint32_t)buf[off + 1U] << 8) |
                ((uint32_t)buf[off + 2U] << 16) |
                ((uint32_t)buf[off + 3U] << 24);
            if ((value & 0x0FFFFFFFU) == 0) {
                *found_cluster = c;
                scan->cursor = (c + 1U < limit) ? c + 1U : first_cluster;
                scan->scanned_clusters = 0;
                return ISSUE3_FAT_FOUND;
            }
        }
        scan->scanned_clusters += end - begin;
        scan->cursor = end < limit ? end : first_cluster;
        if (scan->scanned_clusters >= cluster_count) {
            scan->complete = 1;
            return ISSUE3_FAT_FULL;
        }
    }
    return ISSUE3_FAT_BUDGET;
}

#endif
