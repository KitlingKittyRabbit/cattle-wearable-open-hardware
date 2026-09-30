#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "issue3_logic.h"

typedef struct {
    uint8_t op;
    uint32_t xfers;
    uint32_t cs_low;
    uint32_t cs_high;
} MockCard;

static void mock_cs(void *ctx, uint8_t high)
{
    MockCard *card = (MockCard *)ctx;
    if (high) card->cs_high++;
    else card->cs_low++;
}

static uint8_t mock_xfer(void *ctx, uint8_t out)
{
    MockCard *card = (MockCard *)ctx;
    uint32_t index = card->xfers++;
    (void)out;

    /* The first byte is the start guard, followed by seven command bytes and
     * one response poll.  The remaining stream mirrors the production
     * protocol phases closely enough to exercise every bounded transition. */
    if (index <= 8U) return index == 8U ? 0x00U : 0xFFU;
    if (card->op == ISSUE3_SD_OP_READ) {
        if (index == 9U) return 0xFEU;
        if (index >= 10U && index < 522U)
            return (uint8_t)(index - 10U);
        return 0xFFU;
    }
    if (index == 524U) return 0x05U;
    return 0xFFU;
}

static void drive_transfer(volatile Issue3RuntimeDiagnostics *diag,
                           volatile Issue3DiagRuntimeState *state,
                           uint8_t op, uint32_t *now_ms)
{
    uint8_t read_buf[512] = {0};
    uint8_t write_buf[512];
    for (uint32_t i = 0; i < sizeof(write_buf); i++)
        write_buf[i] = (uint8_t)i;

    MockCard card = {op, 0U, 0U, 0U};
    Issue3SdTransfer transfer = {0};
    assert(issue3_sd_transfer_start(&transfer, op, 17U, read_buf,
                                    write_buf, mock_cs, mock_xfer, &card) == 0);
    issue3_diag_runtime_sd_begin(diag, state, op, *now_ms);

    while (transfer.active) {
        uint8_t before = transfer.phase;
        issue3_diag_runtime_sd_step(diag, state, *now_ms, before);
        assert(issue3_sd_transfer_step(&transfer, mock_cs, mock_xfer,
                                       &card) == 1);
        issue3_diag_sat_add(&diag->spi_xfer_bytes, 1U);
        (*now_ms)++;
        if (!transfer.active) {
            issue3_diag_runtime_sd_finish(diag, state, *now_ms);
        } else if (transfer.phase != before) {
            issue3_diag_runtime_sd_phase_change(diag, state, before,
                                                transfer.phase, *now_ms);
        }
    }

    assert(transfer.result_ready);
    assert(issue3_sd_transfer_take_result(&transfer) == 0);
    assert(card.cs_low == 1U && card.cs_high == 2U);
    assert(card.xfers > 520U);
    if (op == ISSUE3_SD_OP_READ) {
        assert(read_buf[0] == 0U);
        assert(read_buf[511] == 0xFFU);
    }
}

static void test_diagnostics_do_not_change_transfer(void)
{
    uint8_t plain_read[512] = {0};
    uint8_t diag_read[512] = {0};
    uint8_t write_buf[512];
    for (uint32_t i = 0U; i < sizeof(write_buf); i++)
        write_buf[i] = (uint8_t)i;
    MockCard plain_card = {ISSUE3_SD_OP_READ, 0U, 0U, 0U};
    MockCard diag_card = {ISSUE3_SD_OP_READ, 0U, 0U, 0U};
    Issue3SdTransfer plain = {0};
    Issue3SdTransfer instrumented = {0};
    volatile Issue3RuntimeDiagnostics diag = {0};
    Issue3DiagRuntimeState state = {0};
    uint32_t now_ms = 0U;
    assert(issue3_sd_transfer_start(&plain, ISSUE3_SD_OP_READ, 17U,
                                    plain_read, write_buf, mock_cs,
                                    mock_xfer, &plain_card) == 0);
    assert(issue3_sd_transfer_start(&instrumented, ISSUE3_SD_OP_READ, 17U,
                                    diag_read, write_buf, mock_cs,
                                    mock_xfer, &diag_card) == 0);
    issue3_diag_runtime_sd_begin(&diag, &state, ISSUE3_SD_OP_READ, now_ms);
    while (plain.active && instrumented.active) {
        uint8_t before = instrumented.phase;
        issue3_diag_runtime_sd_step(&diag, &state, now_ms, before);
        int plain_result = issue3_sd_transfer_step(
            &plain, mock_cs, mock_xfer, &plain_card);
        int diag_result = issue3_sd_transfer_step(
            &instrumented, mock_cs, mock_xfer, &diag_card);
        assert(plain_result == diag_result);
        now_ms++;
        if (!instrumented.active)
            issue3_diag_runtime_sd_finish(&diag, &state, now_ms);
        else if (instrumented.phase != before)
            issue3_diag_runtime_sd_phase_change(
                &diag, &state, before, instrumented.phase, now_ms);
        assert(plain.active == instrumented.active);
        assert(plain.phase == instrumented.phase &&
               plain.index == instrumented.index &&
               plain.polls == instrumented.polls &&
               plain.result_ready == instrumented.result_ready &&
               plain.result == instrumented.result);
    }
    assert(!plain.active && !instrumented.active);
    int plain_result = issue3_sd_transfer_take_result(&plain);
    int diag_result = issue3_sd_transfer_take_result(&instrumented);
    assert(plain_result == diag_result);
    assert(memcmp(plain_read, diag_read, sizeof(plain_read)) == 0);
    assert(plain_card.xfers == diag_card.xfers &&
           plain_card.cs_low == diag_card.cs_low &&
           plain_card.cs_high == diag_card.cs_high);
}

static void test_sd_step_gap_scope(void)
{
    volatile Issue3RuntimeDiagnostics diag = {0};
    Issue3DiagRuntimeState state = {0};
    diag.magic = ISSUE3_RUNTIME_DIAG_MAGIC;
    diag.version = ISSUE3_RUNTIME_DIAG_VERSION;
    issue3_diag_runtime_sd_begin(&diag, &state, ISSUE3_SD_OP_READ, 10U);
    issue3_diag_runtime_sd_step(&diag, &state, 10U, ISSUE3_SD_PHASE_CMD);
    issue3_diag_runtime_sd_phase_change(
        &diag, &state, ISSUE3_SD_PHASE_CMD, ISSUE3_SD_PHASE_RESP, 17U);
    issue3_diag_runtime_sd_step(&diag, &state, 17U, ISSUE3_SD_PHASE_RESP);
    issue3_diag_runtime_sd_finish(&diag, &state, 18U);

    /* The one-second idle interval is between transactions.  It must not
     * replace the seven-millisecond gap observed within the first one. */
    issue3_diag_runtime_sd_begin(&diag, &state, ISSUE3_SD_OP_WRITE, 1000U);
    issue3_diag_runtime_sd_step(&diag, &state, 1000U, ISSUE3_SD_PHASE_CMD);
    issue3_diag_runtime_sd_phase_change(
        &diag, &state, ISSUE3_SD_PHASE_CMD, ISSUE3_SD_PHASE_RESP, 1002U);
    issue3_diag_runtime_sd_step(&diag, &state, 1002U, ISSUE3_SD_PHASE_RESP);
    issue3_diag_runtime_sd_finish(&diag, &state, 1003U);
    assert(diag.sd_step_max_gap_ms == 7U);
}

static void test_stage_saturation(void)
{
    volatile Issue3RuntimeDiagnostics diag = {0};
    Issue3DiagRuntimeState state = {0};
    issue3_diag_runtime_operation_begin(
        &diag, &diag.fat_prefetch, &state.fat_active, &state.fat_start_ms, 0U);
    issue3_diag_runtime_operation_finish(
        &diag, &diag.fat_prefetch, &state.fat_active, state.fat_start_ms, 7U);
    issue3_diag_runtime_operation_begin(
        &diag, &diag.fat_prefetch, &state.fat_active, &state.fat_start_ms, 8U);
    issue3_diag_runtime_operation_finish(
        &diag, &diag.fat_prefetch, &state.fat_active, state.fat_start_ms,
        70008U);
    assert(diag.fat_prefetch.entries == 2U);
    assert(diag.fat_prefetch.last_ms == 0xFFFFU);
    assert(diag.fat_prefetch.max_ms == 0xFFFFU);
    assert(diag.fat_prefetch.total_ms == 70007U);
}

static void test_frame_open_reason_is_exact(
    volatile Issue3RuntimeDiagnostics *diag)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.count = ISSUE3_FRAME_QUEUE_CAPACITY;
    for (uint8_t i = 0U; i < queue.count; i++)
        queue.start_ms[i] = (uint32_t)i * 1000U;
    uint8_t slot = 0U, is_new = 0U;
    Issue3FrameOpenReason reason = ISSUE3_FRAME_OPEN_REASON_NONE;

    /* Existing safe frame: an unrelated pending slot is not OWNER. */
    issue3_frame_queue_claim_pending(&queue, 6U);
    assert(issue3_frame_queue_open_runtime_ex(
               &queue, 3000U, 10000U, 1U, 0, 0, &slot, &is_new,
               &reason) == ISSUE3_FRAME_OPEN_EXISTING);
    assert(reason == ISSUE3_FRAME_OPEN_REASON_NONE && slot == 3U);

    /* The exact physical slot is pending: OWNER, not a blanket pending test. */
    issue3_frame_queue_claim_pending(&queue, 3U);
    assert(issue3_frame_queue_timestamp_reason(&queue, 3000U) ==
           ISSUE3_FRAME_OPEN_REASON_OWNER);
    assert(issue3_frame_queue_open_runtime_ex(
               &queue, 3000U, 10000U, 1U, 0, 0, &slot, &is_new,
               &reason) == ISSUE3_FRAME_OPEN_BLOCKED);
    assert(reason == ISSUE3_FRAME_OPEN_REASON_OWNER);

    /* A future append whose needed slot is not the pending slot is QUEUE. */
    issue3_frame_queue_claim_pending(&queue, 6U);
    assert(issue3_frame_queue_timestamp_reason(&queue, 3000U) ==
           ISSUE3_FRAME_OPEN_REASON_NONE);
    assert(issue3_frame_queue_open_runtime_ex(
               &queue, 8000U, 10000U, 1U, 0, 0, &slot, &is_new,
               &reason) == ISSUE3_FRAME_OPEN_BLOCKED);
    assert(reason == ISSUE3_FRAME_OPEN_REASON_QUEUE);

    /* A timestamp before the retained head is TIME even while SD work is
     * pending; the reason comes from the frame-open decision itself. */
    queue.head = 0U;
    for (uint8_t i = 0U; i < queue.count; i++)
        queue.start_ms[i] = 3000U + (uint32_t)i * 1000U;
    issue3_frame_queue_claim_pending(&queue, 6U);
    assert(issue3_frame_queue_open_runtime_ex(
               &queue, 2000U, 10000U, 1U, 0, 0, &slot, &is_new,
               &reason) == ISSUE3_FRAME_OPEN_BLOCKED);
    assert(reason == ISSUE3_FRAME_OPEN_REASON_TIME);

    issue3_diag_note_frame_drop(diag, ISSUE3_DIAG_SENSOR_QMI,
                                ISSUE3_FRAME_OPEN_REASON_TIME);
    issue3_diag_note_frame_drop(diag, ISSUE3_DIAG_SENSOR_QMI,
                                ISSUE3_FRAME_OPEN_REASON_OWNER);
    issue3_diag_note_frame_drop(diag, ISSUE3_DIAG_SENSOR_QMI,
                                ISSUE3_FRAME_OPEN_REASON_QUEUE);
    assert(diag->qmi_drop_time == 1U && diag->qmi_drop_owner == 1U &&
           diag->qmi_drop_queue == 1U);
}

int main(void)
{
    volatile Issue3RuntimeDiagnostics diag = {0};
    Issue3DiagRuntimeState state = {0};
    diag.magic = ISSUE3_RUNTIME_DIAG_MAGIC;
    diag.version = ISSUE3_RUNTIME_DIAG_VERSION;
    assert(sizeof(Issue3RuntimeDiagnostics) == 280U);

    test_stage_saturation();
    test_diagnostics_do_not_change_transfer();
    test_sd_step_gap_scope();
    uint32_t now_ms = 0U;
    drive_transfer(&diag, &state, ISSUE3_SD_OP_READ, &now_ms);
    /* A long idle gap belongs between transactions, not to the active-step
     * metric.  The shared begin/step/finish path resets that identity. */
    now_ms += 1000U;
    drive_transfer(&diag, &state, ISSUE3_SD_OP_WRITE, &now_ms);

    assert(diag.sd_transactions_total == 2U);
    assert(diag.sd_read_transactions == 1U);
    assert(diag.sd_write_transactions == 1U);
    assert(diag.sd_transaction.entries == 2U);
    assert(diag.sd_transaction.total_ms > 0U);
    assert(diag.sd_phase[0].entries == 2U); /* command */
    assert(diag.sd_phase[1].entries == 2U); /* response */
    assert(diag.sd_phase[2].entries == 2U); /* token */
    assert(diag.sd_phase[3].entries == 2U); /* read data/CRC */
    assert(diag.sd_phase[4].entries == 2U); /* write data/CRC */
    assert(diag.sd_phase[5].entries == 1U); /* write response */
    assert(diag.sd_phase[6].entries == 1U); /* write busy */
    assert(diag.read_token_polls == 1U);
    assert(diag.write_busy_polls == 1U);
    assert(diag.sd_step_max_gap_ms <= 1U);

    issue3_diag_runtime_operation_begin(
        &diag, &diag.fat_prefetch, &state.fat_active, &state.fat_start_ms, 10U);
    issue3_diag_runtime_operation_finish(
        &diag, &diag.fat_prefetch, &state.fat_active, state.fat_start_ms, 13U);
    issue3_diag_runtime_operation_begin(
        &diag, &diag.fat_link, &state.link_active, &state.link_start_ms, 20U);
    issue3_diag_runtime_operation_finish(
        &diag, &diag.fat_link, &state.link_active, state.link_start_ms, 24U);
    issue3_diag_runtime_operation_begin(
        &diag, &diag.metadata, &state.metadata_active,
        &state.metadata_start_ms, 30U);
    issue3_diag_runtime_operation_finish(
        &diag, &diag.metadata, &state.metadata_active,
        state.metadata_start_ms, 35U);
    assert(diag.fat_prefetch.entries == 1U && diag.fat_prefetch.last_ms == 3U);
    assert(diag.fat_link.entries == 1U && diag.fat_link.last_ms == 4U);
    assert(diag.metadata.entries == 1U && diag.metadata.last_ms == 5U);

    issue3_diag_runtime_sector_complete(&diag, &state, 100U, 512U, 0U);
    issue3_diag_runtime_sector_complete(&diag, &state, 107U, 1024U, 1U);
    assert(diag.sector_complete_count == 2U);
    assert(diag.sector_last_gap_ms == 7U && diag.sector_max_gap_ms == 7U);
    assert(diag.persisted_bytes == 1024U && diag.complete_frame_count == 1U);

    /* Diagnostic writes are counters only; the transfer result and protocol
     * byte stream above remain the same successful read/write transactions. */
    assert(issue3_diag_sd_phase_index(ISSUE3_SD_PHASE_WRITE_BUSY) == 6U);
    assert(issue3_diag_sd_phase_index(ISSUE3_SD_PHASE_READ_CRC) == 3U);
    test_frame_open_reason_is_exact(&diag);
    return 0;
}
