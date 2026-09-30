#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../固件/源码/issue3_logic.h"

typedef struct {
    uint8_t fat[16][512];
    uint32_t reads;
    uint32_t fail_lba;
} FatMock;

static int fat_read(void *ctx, uint32_t lba, uint8_t *buf)
{
    FatMock *mock = (FatMock *)ctx;
    mock->reads++;
    if (lba == mock->fail_lba) return -1;
    if (lba >= 16) return -1;
    memcpy(buf, mock->fat[lba], 512);
    return 0;
}

static void fat_set(FatMock *mock, uint32_t cluster, uint32_t value)
{
    uint32_t lba = (cluster * 4U) / 512U;
    uint16_t off = (uint16_t)((cluster * 4U) & 511U);
    mock->fat[lba][off] = (uint8_t)value;
    mock->fat[lba][off + 1U] = (uint8_t)(value >> 8);
    mock->fat[lba][off + 2U] = (uint8_t)(value >> 16);
    mock->fat[lba][off + 3U] = (uint8_t)(value >> 24);
}

static void test_frame_boundary(void)
{
    assert(ISSUE3_SPI_INPUT_CLOCK_HZ == 16000000UL);
    assert(ISSUE3_SD_INIT_SPI_HZ <= ISSUE3_SD_SPI_MAX_HZ);
    assert(ISSUE3_SD_RUNTIME_SPI_HZ == 2000000UL);
    assert(issue3_spi_clock_hz(ISSUE3_SD_SPI_INIT_BR) ==
           ISSUE3_SD_INIT_SPI_HZ);
    assert(issue3_spi_clock_hz(ISSUE3_SD_SPI_RUNTIME_BR) ==
           ISSUE3_SD_RUNTIME_SPI_HZ);
    assert(ISSUE3_SD_SPI_BYTE_US == 4ULL);
    assert(ISSUE3_SD_SPI_XFER_MAX_US == 8ULL);
    assert(ISSUE3_SD_STEP_MAX_US == 64ULL);
    assert(issue3_sd_step_chunk(512U) == 8U);
    assert(issue3_sd_step_chunk(3U) == 3U);
    assert(ISSUE3_QMI_FIFO_MAX_DELAY_MS == 4568ULL);
    assert(ISSUE3_SD_READ_MAX_MS == 29ULL);
    assert(ISSUE3_SD_WRITE_MAX_MS == 13ULL);
    assert(ISSUE3_SD_METADATA_MAX_MS == 42ULL);
    assert(ISSUE3_QMI_READ_GUARD_MS == 1920ULL);
    assert(ISSUE3_QMI_READ_GUARD_MS <
           (ISSUE3_QMI_FIFO_MAX_DELAY_MS + ISSUE3_FRAME_PERIOD_MS));
    assert(ISSUE3_SD_METADATA_GUARD_MS == 1820ULL);
    assert(ISSUE3_FRAME_RETENTION_MS == 7489ULL);
    assert(!issue3_frame_commit_due(0, 5000));
    assert(!issue3_frame_commit_due(0, 5530));
    assert(!issue3_frame_commit_due(0, 5819));
    assert(issue3_frame_commit_due(0, 7489));
    assert(!issue3_frame_commit_due(UINT32_MAX - 1000U, UINT32_MAX - 500U));
    assert(issue3_frame_commit_due(UINT32_MAX - 1000U, 8000U));

    /* An old end-of-second sample is accepted before the derived boundary,
     * then committed once, and cannot be committed twice. */
    uint32_t starts[6] = {0, 1000, 2000, 3000, 4000, 5000};
    uint8_t count = 6;
    assert(!issue3_frame_commit_due(starts[0], 5530));
    assert(issue3_frame_commit_due(starts[0], 7489));
    memmove(starts, starts + 1, (count - 1U) * sizeof(starts[0]));
    count--;
    assert(!issue3_frame_commit_due(starts[0], 7489));
    assert(count == 5); /* the old slot was committed exactly once */
}

static void test_runtime_wfi_decision(void)
{
    Issue3RuntimeWork work = {0};
    work.wake_source_ready = 1U;
    work.now_ms = 40U;
    work.wake_deadline_ms = 100U;
    assert(issue3_runtime_should_wfi(&work));

    /* An active SD transaction, a ready FIFO, or an immediately due
     * conversion must keep the foreground running; the old unconditional
     * WFI would have slept in each of these states. */
    work.sd_active = 1U;
    assert(!issue3_runtime_should_wfi(&work));
    work.sd_active = 0U;
    work.qmi_ready = 1U;
    assert(!issue3_runtime_should_wfi(&work));
    work.qmi_ready = 0U;
    work.gzp_ready = 1U;
    assert(!issue3_runtime_should_wfi(&work));
    work.gzp_ready = 0U;
    work.now_ms = 100U;
    assert(!issue3_runtime_should_wfi(&work));
    puts("运行期工作判定：SD/FIFO/短期限不休眠，空闲才使用可靠唤醒：通过");
}

typedef struct {
    Issue3RuntimeWork state;
    uint8_t irq_masked;
    uint8_t pending;
    uint8_t inject_pending;
    uint8_t woke_by_pending;
    uint32_t armed_deadline;
    uint32_t now_ms;
    uint32_t wfi_calls;
} SleepRaceMock;

static void sleep_mock_disable(void *ctx)
{
    SleepRaceMock *mock = (SleepRaceMock *)ctx;
    assert(!mock->irq_masked);
    mock->irq_masked = 1U;
}

static void sleep_mock_enable(void *ctx)
{
    SleepRaceMock *mock = (SleepRaceMock *)ctx;
    assert(mock->irq_masked);
    mock->irq_masked = 0U;
}

static void sleep_mock_refresh(void *ctx, Issue3RuntimeWork *work)
{
    SleepRaceMock *mock = (SleepRaceMock *)ctx;
    assert(mock->irq_masked);
    *work = mock->state;
    work->now_ms = mock->now_ms;
    work->wake_pending_mask = mock->pending;
}

static void sleep_mock_arm(void *ctx, uint32_t deadline_ms)
{
    SleepRaceMock *mock = (SleepRaceMock *)ctx;
    assert(mock->irq_masked);
    mock->armed_deadline = deadline_ms;
}

static void sleep_mock_inject_before_wfi(void *ctx)
{
    SleepRaceMock *mock = (SleepRaceMock *)ctx;
    assert(mock->irq_masked);
    /* This is the adversarial window: the event arrives after the final
     * refresh/arm but before the WFI instruction itself. */
    mock->pending |= mock->inject_pending;
}

static void sleep_mock_wfi(void *ctx)
{
    SleepRaceMock *mock = (SleepRaceMock *)ctx;
    assert(mock->irq_masked);
    mock->wfi_calls++;
    if (mock->pending != 0U) {
        /* A pending masked interrupt wakes WFI without advancing to the next
         * 100 ms timer grid.  The real ISR runs after cpsie i. */
        mock->woke_by_pending = 1U;
    } else {
        mock->now_ms = mock->armed_deadline;
    }
}

static void test_runtime_wfi_entry_races(void)
{
    const Issue3RuntimeSleepOps ops = {
        sleep_mock_disable,
        sleep_mock_enable,
        sleep_mock_refresh,
        sleep_mock_arm,
        sleep_mock_inject_before_wfi,
        sleep_mock_wfi
    };
    const uint8_t races[] = {
        ISSUE3_WAKE_PENDING_TIM3_COMPARE,
        ISSUE3_WAKE_PENDING_TIM3_UPDATE,
        ISSUE3_WAKE_PENDING_QMI_EXTI
    };
    for (size_t i = 0; i < sizeof(races); i++) {
        SleepRaceMock mock = {0};
        mock.state.wake_source_ready = 1U;
        mock.now_ms = 40U;
        mock.state.wake_deadline_ms = 100U;
        mock.inject_pending = races[i];
        Issue3RuntimeWork work = {0};
        assert(issue3_runtime_sleep_entry(&work, &ops, &mock) == 1U);
        assert(mock.wfi_calls == 1U);
        assert(mock.woke_by_pending);
        assert(mock.now_ms == 40U); /* no extra 100 ms sleep */
        assert(!mock.irq_masked);
    }

    /* A flag already pending at the final refresh is handled as immediate
     * work and never enters WFI. */
    SleepRaceMock pending = {0};
    pending.state.wake_source_ready = 1U;
    pending.now_ms = 40U;
    pending.state.wake_deadline_ms = 100U;
    pending.pending = ISSUE3_WAKE_PENDING_TIM3_UPDATE;
    Issue3RuntimeWork work = {0};
    assert(issue3_runtime_sleep_entry(&work, &ops, &pending) == 0U);
    assert(pending.wfi_calls == 0U);
    assert(!pending.irq_masked);

    puts("运行期原子 WFI 入口：TIM3 比较/更新及 QMI 竞态均无丢唤醒：通过");
}

typedef struct {
    uint8_t irq_masked;
    uint8_t done;
    uint8_t pending;
    uint8_t inject_pending;
    uint8_t woke_by_pending;
    uint32_t now_ms;
    uint32_t deadline_ms;
    uint32_t wfi_calls;
    uint32_t arm_calls;
    uint32_t start_calls;
    uint8_t old_pending_cleared;
    uint8_t armed;
} ShortWaitMock;

static void short_mock_disable(void *ctx)
{
    ShortWaitMock *mock = (ShortWaitMock *)ctx;
    assert(!mock->irq_masked);
    mock->irq_masked = 1U;
}

static void short_mock_enable(void *ctx)
{
    ShortWaitMock *mock = (ShortWaitMock *)ctx;
    assert(mock->irq_masked);
    mock->irq_masked = 0U;
    if (mock->pending) {
        /* Model TIM3_IRQHandler running immediately after cpsie i. */
        mock->pending = 0U;
        mock->done = 1U;
    }
}

static uint8_t short_mock_ready(void *ctx)
{
    return ((ShortWaitMock *)ctx)->done;
}

static uint8_t short_mock_pending(void *ctx)
{
    return ((ShortWaitMock *)ctx)->pending;
}

static void short_mock_arm(void *ctx)
{
    ShortWaitMock *mock = (ShortWaitMock *)ctx;
    assert(mock->irq_masked);
    if (!mock->armed) mock->arm_calls++;
}

static void short_mock_start(void *ctx)
{
    ShortWaitMock *mock = (ShortWaitMock *)ctx;
    assert(mock->irq_masked);
    mock->start_calls++;
    mock->old_pending_cleared = mock->pending;
    mock->pending = 0U;
    mock->done = 0U;
    mock->armed = 1U;
}

static void short_mock_inject_before_wfi(void *ctx)
{
    ShortWaitMock *mock = (ShortWaitMock *)ctx;
    assert(mock->irq_masked);
    mock->pending |= mock->inject_pending;
}

static void short_mock_wfi(void *ctx)
{
    ShortWaitMock *mock = (ShortWaitMock *)ctx;
    assert(mock->irq_masked);
    mock->wfi_calls++;
    if (mock->pending) {
        mock->woke_by_pending = 1U;
    } else {
        mock->now_ms = mock->deadline_ms;
        mock->pending = ISSUE3_WAKE_PENDING_TIM3_COMPARE;
    }
}

static void test_short_wait_race(void)
{
    const Issue3AtomicWaitOps ops = {
        short_mock_disable,
        short_mock_enable,
        short_mock_ready,
        short_mock_pending,
        short_mock_arm,
        short_mock_inject_before_wfi,
        short_mock_wfi,
        0
    };
    ShortWaitMock mock = {
        .inject_pending = ISSUE3_WAKE_PENDING_TIM3_COMPARE,
        .now_ms = 40U,
        .deadline_ms = 100U
    };
    while (issue3_atomic_wait_step(&ops, &mock)) {}
    assert(mock.done);
    assert(mock.wfi_calls == 1U);
    assert(mock.woke_by_pending);
    assert(mock.now_ms == 40U); /* CC1 arrived before WFI: no 100 ms oversleep */
    assert(mock.arm_calls == 1U);
    assert(!mock.irq_masked);

    const Issue3AtomicWaitOps started_ops = {
        short_mock_disable,
        short_mock_enable,
        short_mock_ready,
        short_mock_pending,
        short_mock_arm,
        0,
        short_mock_wfi,
        short_mock_start
    };
    ShortWaitMock already_pending = {
        .pending = ISSUE3_WAKE_PENDING_TIM3_COMPARE,
        .now_ms = 40U,
        .deadline_ms = 42U
    };
    assert(issue3_atomic_wait_begin(&started_ops, &already_pending));
    assert(already_pending.start_calls == 1U);
    assert(already_pending.old_pending_cleared);
    assert(!already_pending.done && !already_pending.pending);
    while (issue3_atomic_wait_step(&started_ops, &already_pending)) {}
    assert(already_pending.done && already_pending.wfi_calls == 1U);
    assert(!already_pending.woke_by_pending);
    assert(already_pending.arm_calls == 0U);
    assert(already_pending.now_ms == 42U);

    uint32_t target = 0U;
    assert(issue3_short_timer_target(98U, 99U, 5U, &target));
    assert(target == 3U);
    assert(issue3_short_timer_elapsed(98U, 99U) == 1U);
    assert(!issue3_short_timer_target(98U, 3U, 5U, &target));
    assert(!issue3_short_timer_target(50U, 52U, 2U, &target));
    assert(issue3_short_timer_target(99U, 99U, 1U, &target));
    assert(target == 0U);
    assert(issue3_short_timer_target(99U, 0U, 2U, &target));
    assert(target == 1U);

    puts("short_sleep_ms 原子启动/CC1竞态、回绕和配置前进：通过");
}

typedef struct {
    uint32_t calls;
    uint8_t busy_forever;
    uint8_t cs_high;
} SdTransferMock;

static void sd_mock_cs(void *ctx, uint8_t high)
{
    SdTransferMock *mock = (SdTransferMock *)ctx;
    mock->cs_high = high;
}

static uint8_t sd_mock_xfer(void *ctx, uint8_t out)
{
    SdTransferMock *mock = (SdTransferMock *)ctx;
    uint32_t n = mock->calls++;
    /* start() performs one idle byte before the seven command bytes. */
    if (n <= 7U) return 0xFFU;
    if (n == 8U) return 0x00U;             /* R1 */
    if (out == 0xFEU) return 0xFFU;        /* write data token */
    if (n == 9U) return 0xFEU;             /* read data token */
    if (n >= 525U) return mock->busy_forever ? 0x00U : 0xFFU;
    if (n >= 524U) return 0x05U;           /* data response */
    return 0xA5U;
}

static void test_sd_transfer_shared_state_machine(void)
{
    uint8_t data[512] = {0};
    SdTransferMock mock = {0};
    Issue3SdTransfer transfer = {0};
    assert(issue3_sd_transfer_start(&transfer, ISSUE3_SD_OP_READ, 7U,
                                    data, 0, sd_mock_cs, sd_mock_xfer,
                                    &mock) == 0);
    uint32_t guard = 0U;
    while (transfer.active) {
        uint32_t before = mock.calls;
        (void)issue3_sd_transfer_step(&transfer, sd_mock_cs, sd_mock_xfer,
                                      &mock);
        if (transfer.phase == ISSUE3_SD_PHASE_READ_DATA)
            assert(mock.calls - before <= ISSUE3_SD_STEP_BYTES);
        assert(++guard < 1000U);
    }
    assert(issue3_sd_transfer_take_result(&transfer) == 0);
    for (uint16_t i = 0U; i < sizeof(data); i++) assert(data[i] == 0xA5U);

    uint8_t write_data[512];
    memset(write_data, 0x3C, sizeof(write_data));
    mock = (SdTransferMock){0};
    transfer = (Issue3SdTransfer){0};
    assert(issue3_sd_transfer_start(&transfer, ISSUE3_SD_OP_WRITE, 9U, 0,
                                    write_data, sd_mock_cs, sd_mock_xfer,
                                    &mock) == 0);
    guard = 0U;
    while (transfer.active) {
        (void)issue3_sd_transfer_step(&transfer, sd_mock_cs, sd_mock_xfer,
                                      &mock);
        assert(++guard < 1000U);
    }
    assert(issue3_sd_transfer_take_result(&transfer) == 0);

    mock = (SdTransferMock){.busy_forever = 1U};
    transfer = (Issue3SdTransfer){0};
    assert(issue3_sd_transfer_start(&transfer, ISSUE3_SD_OP_WRITE, 10U, 0,
                                    write_data, sd_mock_cs, sd_mock_xfer,
                                    &mock) == 0);
    guard = 0U;
    while (transfer.active) {
        (void)issue3_sd_transfer_step(&transfer, sd_mock_cs, sd_mock_xfer,
                                      &mock);
        assert(++guard < 2000U);
    }
    assert(issue3_sd_transfer_take_result(&transfer) != 0);
    puts("共享 SD 分步读写/token/busy 状态机回归：通过");
}

static void test_frame_sequence_allocation(void)
{
    volatile uint32_t next = 0U;
    assert(issue3_frame_sequence_take(&next) == 0U);
    assert(issue3_frame_sequence_take(&next) == 1U);
    assert(issue3_frame_sequence_take(&next) == 2U);
    assert(next == 3U);
}

static void test_qmi_reanchor(void)
{
    Issue3QmiClock first_loss = {0};
    issue3_qmi_clock_invalidate(&first_loss);
    assert(first_loss.reanchor_count == 1);
    assert(first_loss.gap_pending && !first_loss.valid);
    issue3_qmi_clock_anchor(&first_loss, 2000000ULL, 2);
    issue3_qmi_clock_complete_batch(&first_loss);
    assert(!first_loss.gap_pending);

    Issue3QmiClock clock = {0};
    issue3_qmi_clock_anchor(&clock, 1000000ULL, 4);
    uint32_t a = issue3_qmi_clock_sample_ms(&clock);
    uint32_t b = issue3_qmi_clock_sample_ms(&clock);
    assert(b - a == 35U || b - a == 36U);
    assert(clock.reanchor_count == 0);

    issue3_qmi_clock_invalidate(&clock);
    assert(!clock.valid && clock.gap_pending && clock.reanchor_count == 1);
    issue3_qmi_clock_anchor(&clock, 3000000ULL, 2);
    assert(issue3_qmi_clock_sample_ms(&clock) == 2964U);
    assert(issue3_qmi_clock_sample_ms(&clock) == 3000U);
    issue3_qmi_clock_invalidate(&clock);
    issue3_qmi_clock_invalidate(&clock); /* one loss -> one diagnostic */
    assert(clock.reanchor_count == 2);
}

static void qmi_clock_test_commit(void *ctx, uint8_t slot, uint32_t start_ms)
{
    (void)ctx;
    (void)slot;
    (void)start_ms;
}

static void test_qmi_clock_batch_lag_correction(void)
{
    Issue3QmiClock corrected = {0};
    Issue3QmiClock legacy = {0};
    Issue3QmiClock jitter = {0};
    Issue3FrameQueue corrected_queue = {0};
    Issue3FrameQueue legacy_queue = {0};
    issue3_frame_queue_init(&corrected_queue);
    issue3_frame_queue_init(&legacy_queue);

    uint32_t corrected_late = 0U;
    uint32_t legacy_late = 0U;
    uint32_t corrected_admitted = 0U;
    uint32_t raw_samples = 0U;
    uint32_t last_corrected_ms = 0U;
    uint8_t last_corrected_valid = 0U;
    uint32_t correction_count = 0U;

    /* LOG0010/second-device shape: every FIFO transaction succeeds, but only
     * 587 raw samples arrive during 39.5 s.  The old free-running clock then
     * ends near 20.96 s even though the service itself keeps running. */
    for (uint32_t turn = 0U, now_ms = 100U; turn < 395U;
         turn++, now_ms += 100U) {
        (void)issue3_frame_queue_commit_ready(
            &corrected_queue, now_ms, qmi_clock_test_commit, 0);
        (void)issue3_frame_queue_commit_ready(
            &legacy_queue, now_ms, qmi_clock_test_commit, 0);
        /* GZP/ICP and the one-second frame axis keep the current frame open;
         * this makes old QMI timestamps hit the real retained-window TIME
         * result instead of letting an empty queue hide the failure. */
        uint8_t slot = 0U, is_new = 0U;
        Issue3FrameOpenReason reason = ISSUE3_FRAME_OPEN_REASON_NONE;
        (void)issue3_frame_queue_open_runtime_ex(
            &corrected_queue, now_ms, now_ms, 0U,
            qmi_clock_test_commit, 0, &slot, &is_new, &reason);
        (void)issue3_frame_queue_open_runtime_ex(
            &legacy_queue, now_ms, now_ms, 0U,
            qmi_clock_test_commit, 0, &slot, &is_new, &reason);

        uint16_t samples = turn < 192U ? 2U : 1U;
        uint64_t batch_now_us = (uint64_t)now_ms * 1000ULL;
        if (issue3_qmi_clock_prepare_batch(
                &corrected, batch_now_us, samples))
            correction_count++;
        if (!legacy.valid)
            issue3_qmi_clock_anchor(&legacy, batch_now_us, samples);

        for (uint16_t i = 0U; i < samples; i++) {
            uint32_t corrected_ms = issue3_qmi_clock_sample_ms(&corrected);
            uint32_t legacy_ms = issue3_qmi_clock_sample_ms(&legacy);
            assert(!last_corrected_valid || corrected_ms >= last_corrected_ms);
            assert(corrected_ms <= now_ms);
            last_corrected_ms = corrected_ms;
            last_corrected_valid = 1U;

            int corrected_result = issue3_frame_queue_open_runtime_ex(
                &corrected_queue, corrected_ms, now_ms, 0U,
                qmi_clock_test_commit, 0, &slot, &is_new, &reason);
            if (corrected_result >= 0) corrected_admitted++;
            else if (reason == ISSUE3_FRAME_OPEN_REASON_TIME)
                corrected_late++;

            int legacy_result = issue3_frame_queue_open_runtime_ex(
                &legacy_queue, legacy_ms, now_ms, 0U,
                qmi_clock_test_commit, 0, &slot, &is_new, &reason);
            if (legacy_result < 0 &&
                reason == ISSUE3_FRAME_OPEN_REASON_TIME)
                legacy_late++;
            raw_samples++;
        }
    }

    assert(raw_samples == 587U);
    assert(legacy.clock_us < 22000000ULL);
    assert(39500000ULL - legacy.clock_us > 15000000ULL);
    assert(legacy_late > 0U);
    assert(corrected_admitted == raw_samples);
    assert(corrected_late == 0U);
    assert(correction_count > 0U);
    assert(corrected.reanchor_count == correction_count);
    assert(corrected.clock_us > 39000000ULL);

    /* A normal 100 ms service cadence with a plausible 2/3-sample FIFO
     * batch remains on the existing clock and does not re-anchor per turn. */
    for (uint32_t turn = 0U, now_ms = 1000U; turn < 100U;
         turn++, now_ms += 100U) {
        uint16_t samples = (turn % 4U) == 3U ? 2U : 3U;
        uint64_t now_us = (uint64_t)now_ms * 1000ULL;
        assert(!issue3_qmi_clock_prepare_batch(&jitter, now_us, samples));
        for (uint16_t i = 0U; i < samples; i++)
            assert(issue3_qmi_clock_sample_ms(&jitter) <= now_ms);
    }
    assert(jitter.reanchor_count == 0U);

    /* An explicit FIFO gap still leaves the existing re-anchor diagnostic;
     * this correction must not silently hide a real discontinuity. */
    issue3_qmi_clock_invalidate(&jitter);
    assert(jitter.reanchor_count == 1U && jitter.gap_pending);
    (void)issue3_qmi_clock_prepare_batch(&jitter, 12000000ULL, 4U);
    assert(jitter.gap_pending && jitter.reanchor_count == 1U);
    puts("QMI 40秒成功读取但软件时钟落后校正、TIME admission与正常抖动回归：通过");
}

static void test_qmi_empty_fifo_watchdog(void)
{
    Issue3QmiRecovery startup;
    issue3_qmi_recovery_init(&startup, 0U);
    assert(!issue3_qmi_recovery_service_tick(&startup, 499U));
    assert(issue3_qmi_recovery_service_tick(&startup, 500U));

    Issue3QmiRecovery recovery;
    issue3_qmi_recovery_init(&recovery, 0U);

    /* The watchdog is keyed to the last successful raw sample, not to a
     * particular FIFO status.  Startup has the same bounded deadline. */
    assert(!issue3_qmi_recovery_service_tick(&recovery, 100U));
    assert(!issue3_qmi_recovery_service_tick(&recovery, 400U));
    assert(!issue3_qmi_recovery_service_tick(&recovery, 499U));
    issue3_qmi_recovery_note_sample(&recovery, 500U);
    assert(!issue3_qmi_recovery_service_tick(&recovery, 900U));
    assert(issue3_qmi_recovery_service_tick(&recovery, 1000U));

    /* One watchdog event arms one recovery and suppresses 100 ms retries.
     * A failed/no-data recovery is retried only at the explicit one-second
     * deadline, never on every foreground poll. */
    assert(recovery.recovery_active && !recovery.empty_streak_active);
    assert(!issue3_qmi_recovery_service_tick(&recovery, 1099U));
    assert(!issue3_qmi_recovery_service_tick(&recovery, 1999U));
    assert(issue3_qmi_recovery_service_tick(&recovery, 2000U));

    /* The first recovered raw batch is anchored at its actual current read
     * time; the old pre-gap clock is not reused. */
    issue3_qmi_recovery_note_sample(&recovery, 15000U);
    assert(recovery.last_sample_valid && recovery.last_sample_ms == 15000U);
    Issue3QmiClock recovered = {0};
    issue3_qmi_clock_anchor(&recovered, 15000000ULL, 4U);
    assert(issue3_qmi_clock_sample_ms(&recovered) >= 14800U);
    assert(!recovery.recovery_active);
    assert(!issue3_qmi_recovery_service_tick(&recovery, 15499U));
    assert(issue3_qmi_recovery_service_tick(&recovery, 15500U));
    puts("QMI 500ms无成功原始样本看门狗、单次有界恢复和当前时刻重锚：通过");
}

typedef struct {
    uint8_t regs[256];
    uint8_t op_kind[128];
    uint8_t op_reg[128];
    uint8_t op_value[128];
    uint8_t op_count;
    uint8_t ctrl9_count;
    uint8_t fail_write_reg;
    uint8_t fail_read_reg;
    uint8_t fail_ctrl9;
    uint8_t reset_count_lsb;
    uint8_t reset_status;
    uint8_t stubborn_rd_mode;
} QmiRecoveryMock;

static void qmi_recovery_mock_init(QmiRecoveryMock *mock)
{
    memset(mock, 0, sizeof(*mock));
    mock->fail_write_reg = 0xFFU;
    mock->fail_read_reg = 0xFFU;
    mock->regs[ISSUE3_QMI_REG_FIFO_CTRL] =
        ISSUE3_QMI_FIFO_CTRL_CONFIG;
}

static void qmi_recovery_mock_record(QmiRecoveryMock *mock, uint8_t kind,
                                     uint8_t reg, uint8_t value)
{
    assert(mock->op_count < (uint8_t)sizeof(mock->op_kind));
    mock->op_kind[mock->op_count] = kind;
    mock->op_reg[mock->op_count] = reg;
    mock->op_value[mock->op_count] = value;
    mock->op_count++;
}

static int qmi_recovery_mock_write(void *ctx, uint8_t reg, uint8_t value)
{
    QmiRecoveryMock *mock = (QmiRecoveryMock *)ctx;
    qmi_recovery_mock_record(mock, 'W', reg, value);
    if (reg == mock->fail_write_reg) return -1;
    mock->regs[reg] = value;
    return 0;
}

static int qmi_recovery_mock_read(void *ctx, uint8_t reg, uint8_t *value)
{
    QmiRecoveryMock *mock = (QmiRecoveryMock *)ctx;
    qmi_recovery_mock_record(mock, 'R', reg, mock->regs[reg]);
    if (reg == mock->fail_read_reg) return -1;
    *value = mock->regs[reg];
    if (reg == ISSUE3_QMI_REG_FIFO_CTRL && mock->stubborn_rd_mode)
        *value |= ISSUE3_QMI_FIFO_CTRL_RD_MODE;
    return 0;
}

static int qmi_recovery_mock_ctrl9(void *ctx, uint8_t command)
{
    QmiRecoveryMock *mock = (QmiRecoveryMock *)ctx;
    qmi_recovery_mock_record(mock, 'C', ISSUE3_QMI_REG_CTRL1, command);
    if (mock->fail_ctrl9) return -1;
    if (command == ISSUE3_QMI_CMD_RST_FIFO) {
        mock->ctrl9_count++;
        mock->regs[ISSUE3_QMI_REG_FIFO_COUNT_LSB] =
            mock->reset_count_lsb;
        mock->regs[ISSUE3_QMI_REG_FIFO_STATUS] = mock->reset_status;
        mock->regs[ISSUE3_QMI_REG_FIFO_CTRL] =
            ISSUE3_QMI_FIFO_CTRL_CONFIG;
    }
    return 0;
}

static Issue3QmiRecoveryIo qmi_recovery_mock_io(QmiRecoveryMock *mock)
{
    Issue3QmiRecoveryIo io = {
        mock,
        qmi_recovery_mock_write,
        qmi_recovery_mock_read,
        qmi_recovery_mock_ctrl9
    };
    return io;
}

static void test_qmi_fifo_state_and_formal_recovery(void)
{
    QmiRecoveryMock mock;
    Issue3QmiRecoveryIo io;

    /* Count/status contradictions and asserted flags are never legal empty
     * observations.  A normal four-sample watermark is a valid non-empty
     * state even though the WTM flag is set. */
    assert(issue3_qmi_fifo_classify(0U, 0U) == ISSUE3_QMI_FIFO_EMPTY);
    assert(issue3_qmi_fifo_classify(
               0U, ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY) ==
           ISSUE3_QMI_FIFO_INCONSISTENT);
    assert(issue3_qmi_fifo_classify(
               6U, 0U) == ISSUE3_QMI_FIFO_INCONSISTENT);
    assert(issue3_qmi_fifo_classify(
               0U, ISSUE3_QMI_FIFO_STATUS_WTM) ==
           ISSUE3_QMI_FIFO_INCONSISTENT);
    assert(issue3_qmi_fifo_classify(
               0U, ISSUE3_QMI_FIFO_STATUS_FULL) ==
           ISSUE3_QMI_FIFO_INCONSISTENT);
    assert(issue3_qmi_fifo_classify(
               0U, ISSUE3_QMI_FIFO_STATUS_OVERFLOW) ==
           ISSUE3_QMI_FIFO_OVERFLOW);
    assert(issue3_qmi_fifo_classify(24U,
                                    ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY |
                                    ISSUE3_QMI_FIFO_STATUS_WTM) ==
           ISSUE3_QMI_FIFO_NONEMPTY);
    assert(issue3_qmi_fifo_classify(0U, 0x04U) ==
           ISSUE3_QMI_FIFO_INCONSISTENT);
    Issue3QmiRecovery inconsistent_recovery;
    issue3_qmi_recovery_init(&inconsistent_recovery, 0U);
    uint8_t inconsistent_due = 0U;
    const uint8_t inconsistent_statuses[] = {
        ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY,
        ISSUE3_QMI_FIFO_STATUS_WTM,
        ISSUE3_QMI_FIFO_STATUS_FULL,
        ISSUE3_QMI_FIFO_STATUS_OVERFLOW,
        0x01U /* count MSB says non-zero while LSB/NOT_EMPTY say empty */
    };
    for (uint8_t i = 0U; i < (uint8_t)sizeof(inconsistent_statuses); i++) {
        inconsistent_due = 0U;
        Issue3QmiFifoObservation observation = issue3_qmi_recovery_observe_fifo(
            &inconsistent_recovery,
            0U, inconsistent_statuses[i], 0U, &inconsistent_due);
        assert(observation != ISSUE3_QMI_FIFO_EMPTY);
        assert(!inconsistent_due && !inconsistent_recovery.empty_streak_active);
    }

    /* This is the production executor, not a timer-only test: verify the
     * first operations are read-mode exit then CTRL9 RST_FIFO, all formal
     * stream registers are written/read back, and the final count/status is
     * read as two independent registers. */
    qmi_recovery_mock_init(&mock);
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) == 0);
    assert(mock.op_kind[0] == 'W' &&
           mock.op_reg[0] == ISSUE3_QMI_REG_FIFO_CTRL);
    assert(mock.op_kind[1] == 'C' &&
           mock.op_value[1] == ISSUE3_QMI_CMD_RST_FIFO);
    assert(mock.ctrl9_count == 1U);
    assert(mock.regs[ISSUE3_QMI_REG_CTRL1] == ISSUE3_QMI_CTRL1_CONFIG);
    assert(mock.regs[ISSUE3_QMI_REG_CTRL2] == ISSUE3_QMI_CTRL2_CONFIG);
    assert(mock.regs[ISSUE3_QMI_REG_CTRL3] == ISSUE3_QMI_CTRL3_CONFIG);
    assert(mock.regs[ISSUE3_QMI_REG_CTRL6] == ISSUE3_QMI_CTRL6_CONFIG);
    assert(mock.regs[ISSUE3_QMI_REG_CTRL7] == ISSUE3_QMI_CTRL7_CONFIG);
    assert(mock.regs[ISSUE3_QMI_REG_CTRL8] == ISSUE3_QMI_CTRL8_CONFIG);
    assert(mock.regs[ISSUE3_QMI_REG_FIFO_WTM] ==
           ISSUE3_QMI_FIFO_WTM_CONFIG);
    assert(mock.op_reg[mock.op_count - 2U] ==
           ISSUE3_QMI_REG_FIFO_COUNT_LSB);
    assert(mock.op_reg[mock.op_count - 1U] ==
           ISSUE3_QMI_REG_FIFO_STATUS);

    /* Each stale status that the old 0x23 mask missed must make formal
     * recovery fail closed. */
    const uint8_t bad_statuses[] = {
        ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY,
        ISSUE3_QMI_FIFO_STATUS_WTM,
        ISSUE3_QMI_FIFO_STATUS_FULL,
        ISSUE3_QMI_FIFO_STATUS_OVERFLOW,
        0x04U
    };
    for (uint8_t i = 0U; i < (uint8_t)sizeof(bad_statuses); i++) {
        qmi_recovery_mock_init(&mock);
        mock.reset_status = bad_statuses[i];
        io = qmi_recovery_mock_io(&mock);
        assert(issue3_qmi_recovery_execute(&io) < 0);
    }
    qmi_recovery_mock_init(&mock);
    mock.reset_count_lsb = 6U; /* non-zero count with NOT_EMPTY clear */
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) < 0);
    qmi_recovery_mock_init(&mock);
    mock.stubborn_rd_mode = 1U;
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) < 0);
    qmi_recovery_mock_init(&mock);
    mock.fail_ctrl9 = 1U;
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) < 0);
    qmi_recovery_mock_init(&mock);
    mock.fail_read_reg = ISSUE3_QMI_REG_CTRL3;
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) < 0);

    /* Run the same shared observation decision and formal executor through a
     * watchdog sequence.  Contradictions never start the 500 ms timer; a
     * legal empty run triggers once, retries only at +1 s, then the first raw
     * batch clears recovery and anchors a current-time frame. */
    Issue3QmiRecovery recovery;
    issue3_qmi_recovery_init(&recovery, 0U);
    uint8_t due = 0U;
    due = 0U;
    assert(issue3_qmi_recovery_observe_fifo(
               &recovery, 0U, ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY,
               0U, &due) == ISSUE3_QMI_FIFO_INCONSISTENT);
    assert(!due && !recovery.empty_streak_active);
    for (uint32_t now = 0U; now < 500U; now += 100U) {
        due = 0U;
        assert(issue3_qmi_recovery_observe_fifo(
                   &recovery, 0U, 0U, now, &due) == ISSUE3_QMI_FIFO_EMPTY);
        assert(!due);
    }
    due = 0U;
    assert(issue3_qmi_recovery_observe_fifo(
               &recovery, 0U, 0U, 500U, &due) == ISSUE3_QMI_FIFO_EMPTY);
    assert(due && recovery.recovery_active);
    qmi_recovery_mock_init(&mock);
    mock.fail_ctrl9 = 1U;
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) < 0);
    due = 0U;
    assert(issue3_qmi_recovery_observe_fifo(
               &recovery, 0U, 0U, 600U, &due) == ISSUE3_QMI_FIFO_EMPTY);
    assert(!due);
    due = 0U;
    assert(issue3_qmi_recovery_observe_fifo(
               &recovery, 0U, 0U, 1499U, &due) == ISSUE3_QMI_FIFO_EMPTY);
    assert(!due);
    due = 0U;
    assert(issue3_qmi_recovery_observe_fifo(
               &recovery, 0U, 0U, 1500U, &due) == ISSUE3_QMI_FIFO_EMPTY);
    assert(due);
    qmi_recovery_mock_init(&mock);
    io = qmi_recovery_mock_io(&mock);
    assert(issue3_qmi_recovery_execute(&io) == 0);
    issue3_qmi_recovery_note_sample(&recovery, 2000U);
    Issue3QmiClock clock = {0};
    issue3_qmi_clock_anchor(&clock, 2000000ULL, 4U);
    assert(issue3_qmi_clock_sample_ms(&clock) >= 1892U);
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    uint8_t slot = 0U, is_new = 0U;
    assert(issue3_frame_queue_open_runtime(
               &queue, 2000U, 2000U, 0U, 0, 0, &slot, &is_new) >= 0);
    assert(is_new);

    /* The status helper is intentionally order-independent: both events in
     * one frame remain visible after either retry order. */
    uint32_t status = issue3_qmi_recovery_status_or(0U, 0U, 1U);
    status = issue3_qmi_recovery_status_or(status, 1U, 0U);
    assert((status & ISSUE3_QMI_STATUS_RECOVERY_FAIL) != 0U &&
           (status & ISSUE3_QMI_STATUS_RECOVERY_OK) != 0U);
    status = issue3_qmi_recovery_status_or(0U, 1U, 0U);
    status = issue3_qmi_recovery_status_or(status, 0U, 1U);
    assert((status & ISSUE3_QMI_STATUS_RECOVERY_FAIL) != 0U &&
           (status & ISSUE3_QMI_STATUS_RECOVERY_OK) != 0U);
    puts("QMI FIFO 状态一致性、正式寄存器恢复顺序、失败/成功累计诊断：通过");
}

typedef enum {
    QMI_STALL_NORMAL = 0,
    QMI_STALL_LEGAL_EMPTY,
    QMI_STALL_INCONSISTENT,
    QMI_STALL_CTRL9_FAILURE,
    QMI_STALL_FIFO_DATA_FAILURE
} QmiStallMode;

typedef struct {
    Issue3QmiRecovery recovery;
    Issue3QmiClock clock;
    QmiRecoveryMock mock;
    Issue3QmiRecoveryIo io;
    uint32_t recovery_attempts;
    uint32_t recovery_ok;
    uint32_t recovery_failed;
    uint32_t last_raw_ms;
    uint32_t frame_status;
} QmiStallProductionMock;

static void qmi_stall_production_init(QmiStallProductionMock *run)
{
    memset(run, 0, sizeof(*run));
    issue3_qmi_recovery_init(&run->recovery, 0U);
    qmi_recovery_mock_init(&run->mock);
    run->io = qmi_recovery_mock_io(&run->mock);
}

/* This is the production service decision in a controlled host harness:
 * every 100 ms turn records the status observation, then only a successful
 * raw FIFO read calls note_sample().  Failure modes deliberately return to
 * the same formal recovery executor used by main.c. */
static void qmi_stall_production_poll(QmiStallProductionMock *run,
                                      uint32_t now_ms, QmiStallMode mode)
{
    uint8_t count_lsb = 24U;
    uint8_t fifo_status = ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY |
                          ISSUE3_QMI_FIFO_STATUS_WTM;
    if (mode == QMI_STALL_LEGAL_EMPTY) {
        count_lsb = 0U;
        fifo_status = 0U;
    } else if (mode == QMI_STALL_INCONSISTENT) {
        count_lsb = 0U;
        fifo_status = ISSUE3_QMI_FIFO_STATUS_NOT_EMPTY;
    }

    uint8_t recovery_due = issue3_qmi_recovery_service_tick(
        &run->recovery, now_ms);
    Issue3QmiFifoObservation observation = issue3_qmi_recovery_observe_fifo(
        &run->recovery, count_lsb, fifo_status, now_ms, &recovery_due);
    if (mode == QMI_STALL_LEGAL_EMPTY)
        assert(observation == ISSUE3_QMI_FIFO_EMPTY);
    else if (mode == QMI_STALL_INCONSISTENT)
        assert(observation == ISSUE3_QMI_FIFO_INCONSISTENT);
    else
        assert(observation == ISSUE3_QMI_FIFO_NONEMPTY);

    if (mode == QMI_STALL_NORMAL) {
        if (!run->clock.valid)
            issue3_qmi_clock_anchor(&run->clock, (uint64_t)now_ms * 1000ULL,
                                    4U);
        issue3_qmi_recovery_note_sample(&run->recovery, now_ms);
        run->last_raw_ms = now_ms;
        return;
    }
    if (!recovery_due) return;

    /* main.c invalidates before every stalled-stream reset, even when the
     * register operation itself later fails. */
    issue3_qmi_clock_invalidate(&run->clock);
    run->recovery_attempts++;
    qmi_recovery_mock_init(&run->mock);
    if (mode == QMI_STALL_CTRL9_FAILURE)
        run->mock.fail_ctrl9 = 1U;
    /* Model a FIFO_DATA/cleanup failure by making the subsequent formal
     * configuration read fail; no raw sample has been acknowledged. */
    if (mode == QMI_STALL_FIFO_DATA_FAILURE)
        run->mock.fail_read_reg = ISSUE3_QMI_REG_CTRL3;
    int result = issue3_qmi_recovery_execute(&run->io);
    run->frame_status |= ISSUE3_QMI_STATUS_EMPTY_RECOVERY;
    if (result == 0) {
        run->recovery_ok++;
        run->frame_status = issue3_qmi_recovery_status_or(
            run->frame_status, 1U, 0U);
    } else {
        run->recovery_failed++;
        run->frame_status = issue3_qmi_recovery_status_or(
            run->frame_status, 0U, 1U);
    }
}

static void test_qmi_stall_watchdog_production_paths(void)
{
    QmiStallProductionMock run;

    /* Reproduce LOG0010: eight seconds of successful raw batches followed by
     * 600 ms of legal empty observations.  Recovery occurs at 500 ms, the
     * old clock is invalidated, and the next successful batch anchors near
     * its current time instead of continuing from ~8.2 s. */
    qmi_stall_production_init(&run);
    for (uint32_t now = 0U; now <= 8000U; now += 100U)
        qmi_stall_production_poll(&run, now, QMI_STALL_NORMAL);
    assert(run.last_raw_ms == 8000U);
    for (uint32_t now = 8100U; now <= 8600U; now += 100U)
        qmi_stall_production_poll(&run, now, QMI_STALL_LEGAL_EMPTY);
    assert(run.recovery_attempts == 1U && run.recovery_ok == 1U);
    assert(run.recovery_failed == 0U && !run.clock.valid);
    qmi_stall_production_poll(&run, 8700U, QMI_STALL_NORMAL);
    assert(run.clock.valid && run.last_raw_ms == 8700U);
    assert(issue3_qmi_clock_sample_ms(&run.clock) > 8243U);
    assert(run.clock.reanchor_count == 1U);
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    uint8_t slot = 0U, is_new = 0U;
    assert(issue3_frame_queue_open_runtime(
               &queue, run.last_raw_ms, run.last_raw_ms, 0U, 0, 0,
               &slot, &is_new) >= 0);
    assert(is_new);

    /* A count/status contradiction now reaches the same deadline and reset;
     * it no longer prevents recovery merely because it is not legal empty. */
    qmi_stall_production_init(&run);
    qmi_stall_production_poll(&run, 0U, QMI_STALL_NORMAL);
    for (uint32_t now = 100U; now <= 600U; now += 100U)
        qmi_stall_production_poll(&run, now, QMI_STALL_INCONSISTENT);
    assert(run.recovery_attempts == 1U && run.recovery_ok == 1U);
    assert(run.clock.reanchor_count == 1U);

    /* A non-empty FIFO whose CTRL9 request fails still counts as no
     * successful raw sample.  The formal executor fails, and retry is gated
     * to +1 s rather than every 100 ms. */
    qmi_stall_production_init(&run);
    qmi_stall_production_poll(&run, 0U, QMI_STALL_NORMAL);
    for (uint32_t now = 100U; now <= 1500U; now += 100U)
        qmi_stall_production_poll(&run, now, QMI_STALL_CTRL9_FAILURE);
    assert(run.recovery_attempts == 2U && run.recovery_failed == 2U);
    assert(run.recovery_ok == 0U && run.clock.reanchor_count == 1U);

    /* A non-empty FIFO_DATA path that yields no complete raw sample also
     * triggers the same recovery, and a subsequent batch can re-anchor. */
    qmi_stall_production_init(&run);
    qmi_stall_production_poll(&run, 0U, QMI_STALL_NORMAL);
    for (uint32_t now = 100U; now <= 600U; now += 100U)
        qmi_stall_production_poll(&run, now, QMI_STALL_FIFO_DATA_FAILURE);
    assert(run.recovery_attempts == 1U && run.recovery_ok == 0U);
    assert(run.recovery_failed == 1U && run.clock.reanchor_count == 1U);
    qmi_stall_production_poll(&run, 700U, QMI_STALL_NORMAL);
    assert(run.clock.valid && run.last_raw_ms == 700U);
    puts("QMI LOG0010 停滞 watchdog：合法空/状态矛盾/非空读取失败均恢复，时间轴重锚：通过");
}

static void test_qmi_batch_preflight_and_pending_slot(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    for (uint8_t i = 0; i < 7U; i++) queue.start_ms[i] = (uint32_t)i * 1000U;
    queue.count = 7U;
    queue.pending_valid = 1U;
    queue.pending_slot = 7U;

    uint32_t first_ms = 0, last_ms = 0;
    uint16_t fifo_count = 128U;
    /* An invalid clock must be preflighted as one native 128-sample span.
     * With seven queued seconds plus the SD-owned slot, the whole batch is
     * deferred and the hardware FIFO remains untouched. */
    assert(!issue3_qmi_batch_span_fits(&queue, 0U, 0U, 7000000ULL,
                                       fifo_count, &first_ms, &last_ms));
    assert(first_ms == 2468U && last_ms == 7000U);
    assert(fifo_count == 128U);

    /* Once older software queue space is released, the same entire batch
     * fits; no per-sample admission or partial FIFO drain is involved. */
    (void)issue3_frame_queue_commit_oldest(&queue, 0, 0);
    (void)issue3_frame_queue_commit_oldest(&queue, 0, 0);
    assert(issue3_qmi_batch_span_fits(&queue, 0U, 0U, 7000000ULL,
                                      fifo_count, &first_ms, &last_ms));
    fifo_count = 0U;
    assert(queue.pending_valid && queue.count == 5U);

    issue3_frame_queue_release_pending(&queue, 7U);
    uint8_t slot = 0, is_new = 0;
    assert(issue3_frame_queue_open(&queue, 7000U, 7000U, 0, 0,
                                   &slot, &is_new) == ISSUE3_FRAME_OPEN_NEW);
    assert(slot == 7U && is_new == 1U);
}

static void test_startup_time_saturation(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    uint32_t first = 0xFFFFFFFFU, last = 0xFFFFFFFFU;
    assert(issue3_qmi_batch_span_fits(&queue, 0U, 0U, 0ULL, 128U,
                                      &first, &last));
    assert(first == 0U && last == 4531U);
    assert(issue3_icp_batch_span_fits(&queue, 0U, 4U, &first, &last));
    assert(first == 0U && last == 0U);

    Issue3QmiClock clock = {0};
    issue3_qmi_clock_anchor(&clock, 0ULL, 128U);
    assert(clock.clock_us == 0ULL);
    assert(issue3_qmi_clock_sample_ms(&clock) == 0U);
}

static void test_pending_slot_guard(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.start_ms[0] = 0U;
    queue.count = 1U;
    queue.head = 0U;
    issue3_frame_queue_claim_pending(&queue, 1U);
    uint8_t slot = 0, is_new = 0;
    /* The pending physical slot is blocked even though logical count is only
     * one; this is the wraparound that previously reused SD-owned bytes. */
    queue.head = 2U;
    queue.count = 7U;
    for (uint8_t i = 0; i < 7U; i++) queue.start_ms[(2U + i) % 8U] = i * 1000U;
    assert(issue3_frame_queue_open(&queue, 7000U, 10000U, 0, 0,
                                   &slot, &is_new) == ISSUE3_FRAME_OPEN_BLOCKED);
    issue3_frame_queue_release_pending(&queue, 1U);
    assert(issue3_frame_queue_open(&queue, 7000U, 10000U, 0, 0,
                                   &slot, &is_new) == ISSUE3_FRAME_OPEN_NEW);
}

static void test_full_queue_pending_admission(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.count = ISSUE3_FRAME_QUEUE_CAPACITY;
    for (uint8_t i = 0; i < queue.count; i++)
        queue.start_ms[i] = (uint32_t)i * 1000U;

    uint8_t slot = 0, is_new = 0;
    /* SD/FAT work may be pending while a sample still belongs to an
     * already-open, non-pending frame.  This is the exact admission case
     * that frame_slot_for() must not reject at the queue-capacity boundary. */
    assert(issue3_frame_queue_open_runtime(
               &queue, 3000U, 10000U, 1U, 0, 0, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_EXISTING);
    assert(slot == 3U && !is_new && queue.count == ISSUE3_FRAME_QUEUE_CAPACITY);

    /* A genuinely new second has no safe ring slot while the external SD
     * operation is pending, so it remains blocked rather than overwriting. */
    assert(issue3_frame_queue_open_runtime(
               &queue, 8000U, 10000U, 1U, 0, 0, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_BLOCKED);

    /* Even an existing timestamp is blocked if its physical slot is the one
     * owned by the SD writer; it must never be written through or reused. */
    issue3_frame_queue_claim_pending(&queue, 3U);
    assert(issue3_frame_queue_open_runtime(
               &queue, 3000U, 10000U, 1U, 0, 0, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_BLOCKED);
    issue3_frame_queue_release_pending(&queue, 3U);
    assert(issue3_frame_queue_open_runtime(
               &queue, 3000U, 10000U, 1U, 0, 0, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_EXISTING);
}

static void test_batch_admission_respects_pending_slot(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.count = ISSUE3_FRAME_QUEUE_CAPACITY;
    for (uint8_t i = 0; i < queue.count; i++)
        queue.start_ms[i] = (uint32_t)i * 1000U;

    /* A full queue may still accept a batch whose entire span belongs to
     * retained, non-pending frames.  The same span must be rejected when the
     * SD writer owns one of those physical slots; otherwise the FIFO reader
     * consumes samples that frame_add_* cannot safely append. */
    queue.pending_valid = 1U;
    queue.pending_slot = 6U;
    assert(issue3_frame_queue_span_fits(&queue, 3000U, 3035U));
    assert(issue3_qmi_batch_span_fits(&queue, 1U, 3000000ULL,
                                      3035000ULL, 2U, &(uint32_t){0},
                                      &(uint32_t){0}));
    assert(issue3_icp_batch_span_fits(&queue, 3500U, 2U,
                                      &(uint32_t){0}, &(uint32_t){0}));

    queue.pending_slot = 3U;
    assert(!issue3_frame_queue_span_fits(&queue, 3000U, 3035U));
    uint32_t first_ms = 0U, last_ms = 0U;
    assert(!issue3_qmi_batch_span_fits(&queue, 1U, 3000000ULL,
                                       3035000ULL, 2U,
                                       &first_ms, &last_ms));
    assert(first_ms == 3000U && last_ms == 3035U);
    assert(!issue3_icp_batch_span_fits(&queue, 3500U, 2U,
                                       &first_ms, &last_ms));
    assert(first_ms == 3000U && last_ms == 3500U);
    puts("批次预检区分非 pending 帧与 SD 待写物理槽：通过");
}

static void test_old_sensor_prefix_recovery(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.head = 1U;
    queue.count = 7U;
    for (uint8_t i = 0U; i < queue.count; i++)
        queue.start_ms[(queue.head + i) % ISSUE3_FRAME_QUEUE_CAPACITY] =
            (uint32_t)(i + 1U) * 1000U;
    /* Physical slot 0 is the just-committed second 0 and remains owned by
     * SD even though queue.head now retains second 1. */
    queue.pending_valid = 1U;
    queue.pending_slot = 0U;
    queue.start_ms[0] = 0U;

    assert(issue3_frame_queue_timestamp_unrecoverable(&queue, 0U));
    assert(issue3_frame_queue_timestamp_unrecoverable(&queue, 500U));
    assert(issue3_frame_queue_timestamp_unrecoverable(&queue, 5000U) == 0U);

    Issue3FrameQueue empty = {0};
    issue3_frame_queue_init(&empty);
    empty.pending_valid = 1U;
    empty.pending_slot = 3U;
    empty.start_ms[3] = 9000U;
    assert(issue3_frame_queue_timestamp_unrecoverable(&empty, 9500U));

    uint8_t discard = 0U, admit = 0U;
    issue3_icp_batch_prefix_plan(&queue, 0U, 4U, 4U, &discard, &admit);
    assert(discard == 2U && admit == 2U);

    /* A future point blocked by the full/pending ring is not declared lost;
     * the caller must leave it in the FIFO for a later turn. */
    issue3_icp_batch_prefix_plan(&queue, 7000U, 4U, 4U,
                                 &discard, &admit);
    assert(discard == 0U && admit == 2U);

    queue.pending_valid = 0U;
    issue3_icp_batch_prefix_plan(&queue, 7000U, 4U, 4U,
                                 &discard, &admit);
    assert(discard == 0U && admit == 4U);
    puts("GZP/ICP 旧 pending 前缀有界丢弃、未来样本保留与恢复：通过");
}

/* Five-minute accelerated production-admission regression.  It keeps the
 * real eight-slot queue and repeatedly rolls an SD-owned physical slot while
 * QMI/GZP/ICP producers continue on their native time grids.  The mock does
 * not copy frame bytes (the four-hour asynchronous test verifies SD bytes and
 * CRC); it deliberately exercises the shared ownership/admission decisions
 * that caused the field stop. */
#define ISSUE3_PENDING_RECOVERY_MS 300000U
#define ISSUE3_RECOVERY_QMI_FIFO 128U
#define ISSUE3_RECOVERY_ICP_FIFO 16U

typedef struct {
    Issue3FrameQueue queue;
    uint32_t now_ms;
    uint32_t next_sequence;
    uint8_t pending;
    uint32_t pending_until_ms;
    uint32_t pending_start_ms;
    uint32_t qmi_ts[ISSUE3_RECOVERY_QMI_FIFO];
    uint16_t qmi_count;
    uint64_t qmi_next_us;
    uint32_t qmi_input;
    uint32_t qmi_persisted;
    uint32_t gzp_input;
    uint32_t gzp_persisted;
    uint32_t gzp_missed;
    uint32_t gzp_first_miss_ms;
    uint32_t gzp_last_persist_ms;
    Issue3GzpResultQueue gzp_results;
    uint32_t icp_ts[ISSUE3_RECOVERY_ICP_FIFO];
    uint8_t icp_count;
    uint32_t icp_input;
    uint32_t icp_persisted;
    uint32_t icp_lost;
    uint32_t icp_first_loss_ms;
    uint32_t icp_last_persist_ms;
} PendingRecoveryMock;

static void recovery_commit(void *ctx, uint8_t slot, uint32_t start_ms)
{
    PendingRecoveryMock *mock = (PendingRecoveryMock *)ctx;
    assert(!mock->pending);
    issue3_frame_queue_claim_pending(&mock->queue, slot);
    mock->pending = 1U;
    mock->pending_until_ms = mock->now_ms + 200U;
    mock->pending_start_ms = start_ms;

    /* Force one old result and one old FIFO prefix into the just-owned slot.
     * They must be consumed as diagnosed loss rather than pinning the queue
     * head; later grid points must still be admitted. */
    if (start_ms < 20000U) {
        mock->gzp_input++;
        assert(issue3_gzp_result_queue_push(&mock->gzp_results, start_ms,
                                            0x302010U, 0x40));
        assert(mock->icp_count < ISSUE3_RECOVERY_ICP_FIFO);
        memmove(mock->icp_ts + 1U, mock->icp_ts,
                mock->icp_count * sizeof(mock->icp_ts[0]));
        mock->icp_ts[0] = start_ms;
        mock->icp_count++;
        mock->icp_input++;
    }
}

static int recovery_open(PendingRecoveryMock *mock, uint32_t timestamp_ms)
{
    uint8_t slot = 0U, is_new = 0U;
    int result = issue3_frame_queue_open_runtime(
        &mock->queue, timestamp_ms, mock->now_ms, mock->pending,
        recovery_commit, mock, &slot, &is_new);
    if (result >= 0 && is_new)
        mock->next_sequence++;
    return result;
}

static void recovery_service_qmi(PendingRecoveryMock *mock)
{
    if (mock->qmi_count == 0U) return;
    uint32_t first = mock->qmi_ts[0];
    uint32_t last = mock->qmi_ts[mock->qmi_count - 1U];
    if (!issue3_frame_queue_span_fits(&mock->queue, first, last)) return;
    for (uint16_t i = 0U; i < mock->qmi_count; i++)
        assert(recovery_open(mock, mock->qmi_ts[i]) >= 0);
    mock->qmi_persisted += mock->qmi_count;
    mock->qmi_count = 0U;
}

static void recovery_service_gzp(PendingRecoveryMock *mock)
{
    for (;;) {
        const Issue3GzpResult *result =
            issue3_gzp_result_queue_peek(&mock->gzp_results);
        if (!result) return;
        if (issue3_frame_queue_timestamp_unrecoverable(
                &mock->queue, result->timestamp_ms)) {
            if (mock->gzp_missed++ == 0U)
                mock->gzp_first_miss_ms = mock->now_ms;
            issue3_gzp_result_queue_pop(&mock->gzp_results);
            continue;
        }
        if (issue3_frame_queue_span_fits(&mock->queue, result->timestamp_ms,
                                         result->timestamp_ms) &&
            recovery_open(mock, result->timestamp_ms) >= 0) {
            mock->gzp_persisted++;
            mock->gzp_last_persist_ms = mock->now_ms;
            issue3_gzp_result_queue_pop(&mock->gzp_results);
            continue;
        }
        if (issue3_time_reached(
                mock->now_ms, result->timestamp_ms +
                                   (uint32_t)ISSUE3_FRAME_RETENTION_MS)) {
            if (mock->gzp_missed++ == 0U)
                mock->gzp_first_miss_ms = mock->now_ms;
            issue3_gzp_result_queue_pop(&mock->gzp_results);
            continue;
        }
        return;
    }
}

static void recovery_service_icp(PendingRecoveryMock *mock)
{
    if (mock->icp_count == 0U) return;
    uint8_t limit = mock->icp_count > 4U ? 4U : mock->icp_count;
    uint8_t discard = 0U, admit = 0U;
    issue3_icp_batch_prefix_plan(&mock->queue, mock->icp_ts[0], limit,
                                 limit, &discard, &admit);
    if (discard == 0U && admit == 0U) return;
    uint8_t consumed = (uint8_t)(discard + admit);
    for (uint8_t i = 0U; i < consumed; i++) {
        if (i < discard) {
            if (mock->icp_lost++ == 0U)
                mock->icp_first_loss_ms = mock->now_ms;
        } else {
            assert(recovery_open(mock, mock->icp_ts[i]) >= 0);
            mock->icp_persisted++;
            mock->icp_last_persist_ms = mock->now_ms;
        }
    }
    if (consumed < mock->icp_count)
        memmove(mock->icp_ts, mock->icp_ts + consumed,
                (mock->icp_count - consumed) * sizeof(mock->icp_ts[0]));
    mock->icp_count = (uint8_t)(mock->icp_count - consumed);
}

static void test_pending_sensor_recovery_five_minutes(void)
{
    PendingRecoveryMock mock = {0};
    issue3_frame_queue_init(&mock.queue);
    issue3_gzp_result_queue_init(&mock.gzp_results);
    for (mock.now_ms = 0U; mock.now_ms < ISSUE3_PENDING_RECOVERY_MS;
         mock.now_ms += 100U) {
        if (mock.pending && issue3_time_reached(mock.now_ms,
                                                mock.pending_until_ms)) {
            issue3_frame_queue_release_pending(&mock.queue,
                                                mock.queue.pending_slot);
            mock.pending = 0U;
        }

        (void)issue3_frame_queue_commit_ready(&mock.queue, mock.now_ms,
                                              recovery_commit, &mock);
        /* Ensure the current second exists before producers are serviced. */
        (void)recovery_open(&mock, mock.now_ms);

        while (mock.qmi_next_us <= (uint64_t)mock.now_ms * 1000ULL) {
            assert(mock.qmi_count < ISSUE3_RECOVERY_QMI_FIFO);
            mock.qmi_ts[mock.qmi_count++] =
                (uint32_t)(mock.qmi_next_us / 1000ULL);
            mock.qmi_next_us += ISSUE3_QMI_SAMPLE_PERIOD_US;
            mock.qmi_input++;
        }
        mock.gzp_input++;
        assert(issue3_gzp_result_queue_push(&mock.gzp_results, mock.now_ms,
                                            0x302010U, 0x40));
        if ((mock.now_ms % 500U) == 0U) {
            assert(mock.icp_count < ISSUE3_RECOVERY_ICP_FIFO);
            mock.icp_ts[mock.icp_count++] = mock.now_ms;
            mock.icp_input++;
        }
        recovery_service_qmi(&mock);
        recovery_service_gzp(&mock);
        recovery_service_icp(&mock);
    }

    assert(mock.next_sequence > 200U);
    assert(mock.qmi_input > 8000U);
    assert(mock.qmi_persisted + mock.qmi_count == mock.qmi_input);
    assert(mock.gzp_missed > 0U && mock.gzp_persisted > 1000U);
    assert(mock.gzp_last_persist_ms > mock.gzp_first_miss_ms + 100000U);
    assert(mock.gzp_persisted + mock.gzp_missed +
           mock.gzp_results.count == mock.gzp_input);
    assert(mock.icp_lost > 0U && mock.icp_persisted > 400U);
    assert(mock.icp_last_persist_ms > mock.icp_first_loss_ms + 100000U);
    assert(mock.icp_persisted + mock.icp_lost + mock.icp_count ==
           mock.icp_input);
    printf("5分钟 pending 槽滚动恢复：QMI 输入/落盘=%lu/%lu，GZP 输入/落盘/漏采=%lu/%lu/%lu，ICP 输入/落盘/漏采=%lu/%lu/%lu，帧序列=%lu：通过\n",
           (unsigned long)mock.qmi_input,
           (unsigned long)mock.qmi_persisted,
           (unsigned long)mock.gzp_input,
           (unsigned long)mock.gzp_persisted,
           (unsigned long)mock.gzp_missed,
           (unsigned long)mock.icp_input,
           (unsigned long)mock.icp_persisted,
           (unsigned long)mock.icp_lost,
           (unsigned long)mock.next_sequence);
}

static void test_runtime_open_is_transactional_when_blocked(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.head = 1U;
    queue.count = 6U;
    queue.pending_valid = 1U;
    queue.pending_slot = 0U;
    for (uint8_t i = 0; i < queue.count; i++)
        queue.start_ms[(queue.head + i) % ISSUE3_FRAME_QUEUE_CAPACITY] =
            (uint32_t)i * 1000U;
    uint32_t before[ISSUE3_FRAME_QUEUE_CAPACITY];
    memcpy(before, queue.start_ms, sizeof(before));
    uint8_t slot = 0U, is_new = 0U;
    assert(issue3_frame_queue_open_runtime(
               &queue, 8000U, 10000U, 1U, 0, 0, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_BLOCKED);
    /* A blocked delayed timestamp must not leave an uninitialised appended
     * second in the ring.  The next service turn must see exactly the same
     * queue state after the failed admission. */
    assert(queue.head == 1U && queue.count == 6U &&
           queue.pending_valid && queue.pending_slot == 0U);
    assert(memcmp(before, queue.start_ms, sizeof(before)) == 0);
    puts("队列阻塞 admission 无部分修改：通过");
}

static void test_span_rejects_before_head(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    queue.head = 0U;
    queue.count = 5U;
    for (uint8_t i = 0; i < queue.count; i++)
        queue.start_ms[i] = 3000U + (uint32_t)i * 1000U;
    /* A batch reaching 2000 ms cannot be appended before the current head.
     * The preflight must reject it, matching frame_queue_open(2000)'s LATE
     * result before any FIFO_DATA read occurs. */
    assert(!issue3_frame_queue_span_fits(&queue, 2000U, 4000U));
    assert(issue3_frame_queue_span_fits(&queue, 3000U, 4000U));
    uint8_t slot = 0, is_new = 0;
    assert(issue3_frame_queue_open(&queue, 2000U, 10000U,
                                   0, 0, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_LATE);

    queue.head = 0U;
    queue.count = 7U;
    for (uint8_t i = 0; i < queue.count; i++) queue.start_ms[i] = i * 1000U;
    issue3_frame_queue_claim_pending(&queue, 7U);
    uint32_t first_ms = 0, last_ms = 0;
    assert(!issue3_icp_batch_span_fits(&queue, 7000U, 2U,
                                       &first_ms, &last_ms));
}

static void test_gzp_grid(void)
{
    Issue3GzpSchedule schedule = {0, 0};
    uint32_t plan, missed;
    assert(issue3_gzp_plan(&schedule, 0, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 0 && schedule.next_ms == 100);
    assert(issue3_gzp_plan(&schedule, 101, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 100 && missed == 0 && schedule.next_ms == 200);
    assert(issue3_gzp_plan(&schedule, 210, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 200 && missed == 0 && schedule.next_ms == 300);
    assert(issue3_gzp_plan(&schedule, 310, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 300 && missed == 0 && schedule.next_ms == 400);
    assert(issue3_gzp_plan(&schedule, 751, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 751 && missed == 4 && schedule.next_ms == 800);

    schedule = (Issue3GzpSchedule){100, 0};
    assert(issue3_gzp_skip_pending(&schedule, 350) == 3);
    assert(schedule.next_ms == 400); /* busy/timeout never replays 100..300 */

    schedule = (Issue3GzpSchedule){100, 0};
    assert(issue3_gzp_plan(&schedule, 201, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 200 && missed == 1 && schedule.next_ms == 300);
    schedule = (Issue3GzpSchedule){100, 0};
    assert(issue3_gzp_plan(&schedule, 210, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 200 && missed == 1 && schedule.next_ms == 300);
    schedule = (Issue3GzpSchedule){100, 0};
    assert(issue3_gzp_plan(&schedule, 251, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 251 && missed == 2 && schedule.next_ms == 300);
    schedule = (Issue3GzpSchedule){100, 0};
    assert(issue3_gzp_plan(&schedule, 310, &plan, &missed) == ISSUE3_GZP_START);
    assert(plan == 300 && missed == 2 && schedule.next_ms == 400);
}

typedef struct {
    uint32_t missed;
    uint8_t starts;
    uint8_t fail;
} GzpRuntimeMock;

static int gzp_start_mock(void *ctx)
{
    GzpRuntimeMock *mock = (GzpRuntimeMock *)ctx;
    mock->starts++;
    return mock->fail ? -1 : 0;
}

static void gzp_note_mock(void *ctx, uint32_t count)
{
    GzpRuntimeMock *mock = (GzpRuntimeMock *)ctx;
    mock->missed += count;
}

static void test_gzp_runtime_branch(void)
{
    Issue3GzpSchedule schedule = {100, 0};
    GzpRuntimeMock mock = {0};
    uint32_t plan = 0;
    assert(issue3_gzp_dispatch(&schedule, 210, gzp_start_mock, &mock,
                               gzp_note_mock, &mock, &plan) ==
           ISSUE3_GZP_START);
    assert(plan == 200 && schedule.next_ms == 300);
    assert(mock.starts == 1 && mock.missed == 1);

    schedule = (Issue3GzpSchedule){100, 0};
    mock = (GzpRuntimeMock){0, 0, 1};
    assert(issue3_gzp_dispatch(&schedule, 210, gzp_start_mock, &mock,
                               gzp_note_mock, &mock, &plan) ==
           ISSUE3_GZP_START_FAILED);
    assert(plan == 200 && schedule.next_ms == 300);
    assert(mock.starts == 1 && mock.missed == 2);

    schedule = (Issue3GzpSchedule){100, 0};
    mock = (GzpRuntimeMock){0};
    assert(issue3_gzp_dispatch(&schedule, 251, gzp_start_mock, &mock,
                               gzp_note_mock, &mock, &plan) ==
           ISSUE3_GZP_START);
    assert(plan == 251 && mock.starts == 1 && mock.missed == 2);
}

static void test_gzp_result_queue(void)
{
    Issue3GzpResultQueue queue;
    issue3_gzp_result_queue_init(&queue);
    for (uint32_t i = 0U; i < ISSUE3_GZP_RESULT_QUEUE_CAPACITY; i++)
        assert(issue3_gzp_result_queue_push(&queue, i * 100U,
                                            0x100000U + i,
                                            (int16_t)i));
    assert(!issue3_gzp_result_queue_push(&queue, 900U, 0U, 0));
    for (uint32_t i = 0U; i < ISSUE3_GZP_RESULT_QUEUE_CAPACITY; i++) {
        const Issue3GzpResult *result =
            issue3_gzp_result_queue_peek(&queue);
        assert(result && result->timestamp_ms == i * 100U &&
               result->pressure_raw == 0x100000U + i &&
               result->temperature_raw == (int16_t)i);
        issue3_gzp_result_queue_pop(&queue);
    }
    assert(queue.count == 0U && !issue3_gzp_result_queue_peek(&queue));
}

static void test_read_side_decimation(void)
{
    uint8_t qmi_phase = 0U;
    uint8_t gzp_phase = 0U;
    /* The phase is caller-owned: split the sequence at arbitrary FIFO and
     * conversion boundaries and verify that only even-numbered successful
     * raw items are retained. */
    for (uint8_t i = 0U; i < 16U; i++) {
        assert(issue3_decimation_keep_next(&qmi_phase) == (uint8_t)(i % 2U == 0U));
        if (i == 3U || i == 9U)
            assert(qmi_phase == (uint8_t)((i + 1U) & 1U));
    }
    for (uint8_t i = 0U; i < 10U; i++)
        assert(issue3_decimation_keep_next(&gzp_phase) == (uint8_t)(i % 2U == 0U));
    /* A failed read/conversion does not call this helper, so the next
     * successful result still observes the same phase. */
    assert(gzp_phase == 0U);
    assert(issue3_decimation_keep_next(&gzp_phase) == 1U);
    assert(issue3_decimation_keep_next(&gzp_phase) == 0U);
}

static void test_heartbeat_schedule(void)
{
    Issue3HeartbeatSchedule schedule;
    uint32_t sequence = 0;
    issue3_heartbeat_schedule_init(&schedule, 1800000U);
    assert(schedule.next_ms == 1800000U && schedule.sequence == 0U);

    /* The startup packet is emitted once and consumes seq=0. */
    assert(issue3_heartbeat_startup(&schedule, &sequence));
    assert(sequence == 0U && schedule.startup_sent == 1U);
    assert(!issue3_heartbeat_startup(&schedule, &sequence));
    assert(schedule.sequence == 0U && schedule.in_flight == 1U &&
           schedule.next_ms == 1800000U);
    assert(!issue3_heartbeat_periodic_due(&schedule, 1800000U, &sequence));
    issue3_heartbeat_complete(&schedule);
    assert(schedule.sequence == 1U && schedule.in_flight == 0U);

    assert(!issue3_heartbeat_periodic_due(&schedule, 1799999U, &sequence));
    assert(issue3_heartbeat_periodic_due(&schedule, 1802000U, &sequence));
    assert(sequence == 1U && schedule.in_flight == 1U &&
           schedule.next_ms == 3600000U);
    issue3_heartbeat_complete(&schedule);

    /* A late service call skips old grid points but emits only one current
     * packet and leaves the next deadline on the absolute grid. */
    assert(issue3_heartbeat_periodic_due(&schedule, 5401000U, &sequence));
    assert(sequence == 2U && schedule.in_flight == 1U &&
           schedule.next_ms == 7200000U);
    issue3_heartbeat_complete(&schedule);
    assert(!issue3_heartbeat_periodic_due(&schedule, 7199999U, &sequence));
}

static void test_nf_broadcast_schedule(void)
{
    uint32_t id = 0x504342U;
    uint32_t a = issue3_nf_copy_offset_ms(id, 7, 1);
    uint32_t b = issue3_nf_copy_offset_ms(id, 7, 2);
    assert(a >= ISSUE3_NF_COPY2_MIN_MS && a <= ISSUE3_NF_COPY2_MAX_MS);
    assert(b >= ISSUE3_NF_COPY3_MIN_MS && b <= ISSUE3_NF_COPY3_MAX_MS);
    assert(a < b);
    assert(a == issue3_nf_copy_offset_ms(id, 7, 1));
    assert(b == issue3_nf_copy_offset_ms(id, 7, 2));
    assert(a != issue3_nf_copy_offset_ms(0x414243U, 0, 1) ||
           b != issue3_nf_copy_offset_ms(0x414243U, 0, 2));
    assert(a != issue3_nf_copy_offset_ms(id, 1, 1) ||
           b != issue3_nf_copy_offset_ms(id, 1, 2));

    Issue3NfBroadcast broadcast = {0};
    uint32_t sequence = 0;
    uint8_t copy = 0;
    assert(issue3_nf_broadcast_begin(&broadcast, id, 7, 1000));
    assert(issue3_nf_broadcast_due(&broadcast, 1000, &sequence, &copy));
    assert(sequence == 7 && copy == 0);
    issue3_nf_broadcast_complete(&broadcast);
    assert(!issue3_nf_broadcast_due(&broadcast, 1000 + a - 1,
                                    &sequence, &copy));
    assert(issue3_nf_broadcast_due(&broadcast, 1000 + a,
                                   &sequence, &copy) && copy == 1);
    issue3_nf_broadcast_complete(&broadcast);
    assert(issue3_nf_broadcast_due(&broadcast, 1000 + b,
                                   &sequence, &copy) && copy == 2);
    issue3_nf_broadcast_complete(&broadcast);
    assert(!broadcast.active);

    /* A failed physical copy is represented by completing that copy; it
     * cannot hold the following copy or the next absolute logical heartbeat. */
    assert(issue3_nf_broadcast_begin(&broadcast, id, 8, 2000));
    assert(issue3_nf_broadcast_due(&broadcast, 2000, &sequence, &copy));
    issue3_nf_broadcast_complete(&broadcast);
    assert(issue3_nf_broadcast_due(&broadcast, 2000 + a,
                                   &sequence, &copy) && copy == 1);

    /* Every physical copy uses the shared firmware payload builder with the
     * same logical sequence but its own send-time uptime. */
    uint8_t first[32], second[32], third[32];
    issue3_heartbeat_build_payload(first, id, 3, 1, 7, 1000,
                                   0xA1, 0xB1, 0xC1, 0xD1, 0x03);
    issue3_heartbeat_build_payload(second, id, 3, 1, 7, 11000,
                                   0xA1, 0xB1, 0xC1, 0xD1, 0x03);
    issue3_heartbeat_build_payload(third, id, 3, 1, 7, 21000,
                                   0xA1, 0xB1, 0xC1, 0xD1, 0x03);
    assert(memcmp(first, second, 9) == 0 && memcmp(second, third, 9) == 0);
    assert(first[9] != second[9] || first[10] != second[10] ||
           first[11] != second[11] || first[12] != second[12]);
}

typedef struct {
    Issue3SpiBus bus;
    Issue3SdTransfer sd;
    uint32_t xfers;
    uint8_t sd_cs_low;
    uint8_t nf_cs_low;
    uint16_t token_wait;
    uint16_t busy_wait;
} SharedSpiMock;

static void shared_sd_cs(void *ctx, uint8_t high)
{
    SharedSpiMock *mock = (SharedSpiMock *)ctx;
    if (!high) assert(!mock->nf_cs_low);
    mock->sd_cs_low = (uint8_t)!high;
}

static uint8_t shared_sd_xfer(void *ctx, uint8_t out)
{
    SharedSpiMock *mock = (SharedSpiMock *)ctx;
    /* The initial idle clock is sent with SD CS high; all transaction bytes
     * that follow are sent while the SD owner is held. */
    assert(!mock->nf_cs_low);
    mock->xfers++;
    if (mock->sd.phase == ISSUE3_SD_PHASE_RESP) return 0x00U;
    if (mock->sd.phase == ISSUE3_SD_PHASE_TOKEN) {
        if (mock->token_wait != 0U) {
            mock->token_wait--;
            return 0xFFU;
        }
        return 0xFEU;
    }
    if (mock->sd.phase == ISSUE3_SD_PHASE_WRITE_RESP) return 0x05U;
    if (mock->sd.phase == ISSUE3_SD_PHASE_WRITE_BUSY) {
        if (mock->busy_wait != 0U) {
            mock->busy_wait--;
            return 0x00U;
        }
        return 0xFFU;
    }
    if (mock->sd.phase == ISSUE3_SD_PHASE_READ_DATA) return 0xA5U;
    (void)out;
    return 0xFFU;
}

static uint8_t shared_nf_copy(SharedSpiMock *mock, uint32_t now_ms,
                              uint32_t sequence, uint32_t *uptime_out)
{
    uint8_t msg[32];
    if (!issue3_spi_bus_try_acquire(&mock->bus, ISSUE3_SPI_OWNER_NF))
        return 0U;
    assert(!mock->sd_cs_low);
    mock->nf_cs_low = 1U;
    issue3_heartbeat_build_payload(msg, 0x414243U, 3U, 1U, sequence,
                                   now_ms, 0xA1U, 0xB1U, 0xC1U, 0xD1U,
                                   0x03U);
    mock->xfers += 34U; /* command plus the fixed 32-byte payload */
    mock->nf_cs_low = 0U;
    issue3_spi_bus_release(&mock->bus, ISSUE3_SPI_OWNER_NF);
    *uptime_out = ((uint32_t)msg[9] << 24) | ((uint32_t)msg[10] << 16) |
                  ((uint32_t)msg[11] << 8) | msg[12];
    return 1U;
}

static void test_shared_spi_ownership(void)
{
    const uint8_t targets[] = {
        ISSUE3_SD_PHASE_CMD,
        ISSUE3_SD_PHASE_TOKEN,
        ISSUE3_SD_PHASE_WRITE_DATA,
        ISSUE3_SD_PHASE_WRITE_BUSY
    };
    for (uint8_t case_index = 0U;
         case_index < (uint8_t)(sizeof(targets) / sizeof(targets[0]));
         case_index++) {
        SharedSpiMock mock = {0};
        issue3_spi_bus_init(&mock.bus);
        uint8_t op = targets[case_index] == ISSUE3_SD_PHASE_TOKEN ?
                     ISSUE3_SD_OP_READ : ISSUE3_SD_OP_WRITE;
        mock.token_wait = targets[case_index] == ISSUE3_SD_PHASE_TOKEN ? 3U : 0U;
        mock.busy_wait = targets[case_index] == ISSUE3_SD_PHASE_WRITE_BUSY ? 3U : 0U;
        uint8_t read_data[512] = {0};
        uint8_t write_data[512] = {0};
        assert(issue3_spi_bus_try_acquire(&mock.bus,
                                          ISSUE3_SPI_OWNER_SD));
        assert(issue3_sd_transfer_start(&mock.sd, op, 7U,
                                        op == ISSUE3_SD_OP_READ ? read_data : NULL,
                                        op == ISSUE3_SD_OP_WRITE ? write_data : NULL,
                                        shared_sd_cs, shared_sd_xfer,
                                        &mock) == 0);
        uint8_t checked = 0U;
        uint32_t guard = 0U;
        while (mock.sd.active) {
            if (!checked && mock.sd.phase == targets[case_index]) {
                uint32_t before_xfers = mock.xfers;
                uint32_t blocked_uptime = 0U;
                uint8_t before_phase = mock.sd.phase;
                uint16_t before_index = mock.sd.index;
                assert(!shared_nf_copy(&mock, 100U, 4U, &blocked_uptime));
                assert(mock.xfers == before_xfers);
                assert(mock.sd.phase == before_phase &&
                       mock.sd.index == before_index);
                assert(mock.bus.owner == ISSUE3_SPI_OWNER_SD);
                checked = 1U;
            }
            (void)issue3_sd_transfer_step(&mock.sd, shared_sd_cs,
                                          shared_sd_xfer, &mock);
            if (!mock.sd.active && mock.sd.result_ready)
                issue3_spi_bus_release(&mock.bus, ISSUE3_SPI_OWNER_SD);
            assert(++guard < 1000U);
        }
        assert(checked);
        assert(issue3_sd_transfer_take_result(&mock.sd) == 0);
        assert(mock.bus.owner == ISSUE3_SPI_OWNER_NONE);

        Issue3NfBroadcast broadcast = {0};
        uint32_t seq = 0U, sent_uptime = 0U;
        uint8_t copy = 0U;
        assert(issue3_nf_broadcast_begin(&broadcast, 0x414243U, 4U, 0U));
        assert(issue3_nf_broadcast_due(&broadcast, 0U, &seq, &copy));
        assert(shared_nf_copy(&mock, 0U, seq, &sent_uptime));
        assert(sent_uptime == 0U && copy == 0U);
        issue3_nf_broadcast_complete(&broadcast);
        assert(broadcast.next_copy == 1U);
    }

    /* The real send path is exercised for all three copies: the sequence is
     * held constant, while the payload builder receives each copy's actual
     * send-time uptime. */
    SharedSpiMock copies = {0};
    issue3_spi_bus_init(&copies.bus);
    Issue3NfBroadcast broadcast = {0};
    uint32_t send_uptime[ISSUE3_NF_BROADCAST_COPIES] = {0};
    assert(issue3_nf_broadcast_begin(&broadcast, 0x414243U, 9U, 1000U));
    for (uint8_t i = 0U; i < ISSUE3_NF_BROADCAST_COPIES; i++) {
        uint32_t due_ms = 1000U + broadcast.offset_ms[i];
        uint32_t seq = 0U, uptime = 0U;
        uint8_t copy = 0U;
        assert(issue3_nf_broadcast_due(&broadcast, due_ms, &seq, &copy));
        assert(copy == i && seq == 9U);
        assert(shared_nf_copy(&copies, due_ms, seq, &uptime));
        assert(uptime == due_ms);
        send_uptime[i] = uptime;
        issue3_nf_broadcast_complete(&broadcast);
    }
    assert(!broadcast.active && send_uptime[0] < send_uptime[1] &&
           send_uptime[1] < send_uptime[2]);

    /* NF ownership also prevents a new SD transaction from asserting its
     * chip-select.  This is the inverse half of the production arbitration. */
    SharedSpiMock mock = {0};
    issue3_spi_bus_init(&mock.bus);
    assert(issue3_spi_bus_try_acquire(&mock.bus, ISSUE3_SPI_OWNER_NF));
    assert(!issue3_spi_bus_try_acquire(&mock.bus, ISSUE3_SPI_OWNER_SD));
    assert(mock.bus.owner == ISSUE3_SPI_OWNER_NF);
    issue3_spi_bus_release(&mock.bus, ISSUE3_SPI_OWNER_NF);
    puts("共享 SPI 所有权：CMD/token/data/busy 期间延后 NF、释放后单次发送：通过");
}

static void test_fat_incremental(void)
{
    FatMock mock = {0};
    mock.fail_lba = UINT32_MAX;
    Issue3FatScan scan = {.cursor = 2U};
    uint8_t buf[512];
    uint32_t found = 0;
    int result = issue3_fat_scan_step(&scan, 2, 700, 0, 4,
                                      fat_read, &mock, buf, &found);
    assert(result == ISSUE3_FAT_FOUND && found == 2 && mock.reads == 1);

    mock = (FatMock){0};
    mock.fail_lba = UINT32_MAX;
    for (uint32_t c = 2; c < 512U; c++) fat_set(&mock, c, 0x0FFFFFFF);
    scan = (Issue3FatScan){.cursor = 2U};
    result = issue3_fat_scan_step(&scan, 2, 700, 0, 4,
                                  fat_read, &mock, buf, &found);
    assert(result == ISSUE3_FAT_BUDGET && mock.reads == 4);
    result = issue3_fat_scan_step(&scan, 2, 700, 0, 4,
                                  fat_read, &mock, buf, &found);
    assert(result == ISSUE3_FAT_FOUND && found == 512 && mock.reads == 5);

    mock = (FatMock){0};
    mock.fail_lba = UINT32_MAX;
    for (uint32_t c = 2; c < 1280U; c++) fat_set(&mock, c, 0x0FFFFFFF);
    scan = (Issue3FatScan){.cursor = 2U};
    do {
        result = issue3_fat_scan_step(&scan, 2, 1400, 0, 4,
                                      fat_read, &mock, buf, &found);
    } while (result == ISSUE3_FAT_BUDGET);
    assert(result == ISSUE3_FAT_FOUND && found == 1280 && mock.reads >= 10);

    mock = (FatMock){0};
    mock.fail_lba = UINT32_MAX;
    for (uint32_t c = 2; c < 2U + 700U; c++) fat_set(&mock, c, 0x0FFFFFFF);
    scan = (Issue3FatScan){.cursor = 2U};
    do {
        uint32_t before = mock.reads;
        result = issue3_fat_scan_step(&scan, 2, 700, 0, 4,
                                      fat_read, &mock, buf, &found);
        assert(mock.reads - before <= 4U);
        assert(mock.reads <= 4U * 64U);
    } while (result == ISSUE3_FAT_BUDGET);
    assert(result == ISSUE3_FAT_FULL);

    mock = (FatMock){0};
    mock.fail_lba = UINT32_MAX;
    for (uint32_t c = 2; c < 702U; c++) fat_set(&mock, c, 0x0FFFFFFF);
    fat_set(&mock, 10, 0); /* free cluster is reached only after wrap */
    scan = (Issue3FatScan){.cursor = 500U};
    do {
        result = issue3_fat_scan_step(&scan, 2, 700, 0, 4,
                                      fat_read, &mock, buf, &found);
        assert(result == ISSUE3_FAT_BUDGET || result == ISSUE3_FAT_FOUND);
    } while (result == ISSUE3_FAT_BUDGET);
    assert(found == 10);

    mock = (FatMock){0};
    mock.fail_lba = 0;
    scan = (Issue3FatScan){.cursor = 2U};
    result = issue3_fat_scan_step(&scan, 2, 700, 0, 4,
                                  fat_read, &mock, buf, &found);
    assert(result == ISSUE3_FAT_IO && !scan.complete);
}

static void test_fat_cache_cannot_alias_metadata(void)
{
    FatMock mock = {0};
    mock.fail_lba = UINT32_MAX;
    /* Cluster 2 is the first free entry; the next real free entry is 130.
     * After returning cluster 2, the scan cache still contains FAT sector 0.
     * Simulate the firmware lending that same scratch buffer to a root/
     * metadata operation and filling it with text bytes.  Without the shared
     * invalidation helper, the next allocation would parse those bytes as
     * another free FAT entry (typically cluster 3). */
    for (uint32_t c = 3U; c < 130U; c++)
        fat_set(&mock, c, 0x0FFFFFFFU);
    fat_set(&mock, 130U, 0U);
    Issue3FatScan scan = {.cursor = 2U};
    uint8_t scratch[512];
    uint32_t found = 0U;
    assert(issue3_fat_scan_step(&scan, 2U, 140U, 0U, 1U,
                                fat_read, &mock, scratch, &found) ==
           ISSUE3_FAT_FOUND);
    assert(found == 2U && scan.cache_valid);
    fat_set(&mock, 2U, 0x0FFFFFFFU);
    memset(scratch, 0, sizeof(scratch)); /* root/README bytes, not FAT */
    issue3_fat_scan_cache_invalidate(&scan);
    assert(!scan.cache_valid);
    do {
        int result = issue3_fat_scan_step(&scan, 2U, 140U, 0U, 1U,
                                          fat_read, &mock, scratch, &found);
        if (result == ISSUE3_FAT_FOUND) break;
        assert(result == ISSUE3_FAT_BUDGET);
    } while (1);
    assert(found == 130U);
}

int main(void)
{
    test_frame_boundary();
    test_runtime_wfi_decision();
    test_runtime_wfi_entry_races();
    test_short_wait_race();
    test_sd_transfer_shared_state_machine();
    test_frame_sequence_allocation();
    test_qmi_reanchor();
    test_qmi_clock_batch_lag_correction();
    test_qmi_empty_fifo_watchdog();
    test_qmi_fifo_state_and_formal_recovery();
    test_qmi_stall_watchdog_production_paths();
    test_qmi_batch_preflight_and_pending_slot();
    test_startup_time_saturation();
    test_pending_slot_guard();
    test_full_queue_pending_admission();
    test_batch_admission_respects_pending_slot();
    test_old_sensor_prefix_recovery();
    test_pending_sensor_recovery_five_minutes();
    test_runtime_open_is_transactional_when_blocked();
    test_span_rejects_before_head();
    test_gzp_grid();
    test_gzp_runtime_branch();
    test_gzp_result_queue();
    test_read_side_decimation();
    test_heartbeat_schedule();
    test_nf_broadcast_schedule();
    test_shared_spi_ownership();
    test_fat_incremental();
    test_fat_cache_cannot_alias_metadata();
    puts("Issue #3 共享 C 调度逻辑测试：通过");
    return 0;
}
