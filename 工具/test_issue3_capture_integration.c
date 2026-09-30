#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../固件/源码/issue3_logic.h"

#define TEST_MAX_SAMPLES 320U
#define TEST_MAX_GZP 160U
#define TEST_IDS_PER_FRAME 40U
#define TEST_GZP_PER_FRAME 16U
#define TEST_FAT_SECTORS 16U
#define TEST_DISK_SECTORS 512U
#define TEST_CLUSTER_SPC 2U
#define TEST_FRAME_MAGIC 0x33545354U /* TST3 */
#define TEST_QMI_FIFO_CAPACITY 128U
#define TEST_QMI_WATERMARK 4U
#define TEST_ICP_FIFO_CAPACITY 16U
#define TEST_ICP_PER_FRAME 2U
#define TEST_MAX_ICP 64U

typedef struct {
    uint8_t fat[TEST_FAT_SECTORS][512];
    uint32_t reads;
    uint32_t fail_lba;
} FatMock;

typedef struct {
    Issue3FatScan scan;
    FatMock *fat;
    uint8_t buf[512];
} StartupMock;

typedef struct {
    uint32_t timestamp_ms[TEST_QMI_FIFO_CAPACITY];
    uint32_t id[TEST_QMI_FIFO_CAPACITY];
    uint16_t head;
    uint16_t count;
    uint16_t max_count;
    uint8_t irq_pending;
    uint8_t overflow;
} QmiFifoMock;

typedef struct {
    Issue3FrameQueue queue;
    uint16_t qmi_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t qmi_ids[ISSUE3_FRAME_QUEUE_CAPACITY][TEST_IDS_PER_FRAME];
    uint16_t gzp_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t gzp_plans[ISSUE3_FRAME_QUEUE_CAPACITY][TEST_GZP_PER_FRAME];
    uint32_t icp_input_count;
    uint32_t mts_input_count;
    uint32_t icp_consumed_count;
    uint32_t icp_lost_count;
    uint32_t mts_consumed_count;
    uint32_t icp_fifo_ts[TEST_ICP_FIFO_CAPACITY];
    uint32_t icp_fifo_id[TEST_ICP_FIFO_CAPACITY];
    uint8_t icp_fifo_count;
    uint32_t icp_next_ms;
    uint32_t icp_expected_id[TEST_MAX_ICP];
    uint32_t icp_expected_count;
    uint8_t mts_result_ready;
    uint32_t mts_result_ms;
    /* The committed frame remains in its physical queue slot while the
     * sector writer consumes it.  This is the same ownership model used by
     * main.c; there is deliberately no independent pending-frame copy. */
    uint8_t slot_data[ISSUE3_FRAME_QUEUE_CAPACITY][674];
    uint16_t icp_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t icp_id[ISSUE3_FRAME_QUEUE_CAPACITY][TEST_ICP_PER_FRAME];
    uint32_t icp_ts[ISSUE3_FRAME_QUEUE_CAPACITY][TEST_ICP_PER_FRAME];
    uint16_t mts_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint16_t pending_off;
    uint8_t pending_slot;
    uint8_t pending;
    uint8_t boundary_pending;
    Issue3SectorBuffer sector;
    uint8_t sector_data[512];
    Issue3LogPrefetch log;
    Issue3FatScan log_scan;
    FatMock *fat;
    uint8_t fat_buf[512];

    uint8_t disk[TEST_DISK_SECTORS][512];
    uint8_t disk_valid[TEST_DISK_SECTORS];
    uint32_t write_order[TEST_DISK_SECTORS];
    uint16_t write_count;
    uint32_t current_lba;
    uint8_t sd_write_active;
    uint32_t sd_write_lba;
    uint16_t sd_write_off;
    uint32_t sd_write_elapsed_ms;
    uint32_t links;
    uint32_t committed_frames;
    uint32_t complete_size;
    uint32_t metadata_size;
    uint32_t writes;
    uint32_t now_ms;
    uint32_t simulated_sd_ms;
    uint32_t sd_write_ms;
    uint32_t fat_read_ms;

    QmiFifoMock qmi_fifo;
    uint64_t qmi_next_us;
    uint32_t qmi_next_id;
    uint32_t qmi_input_count;
    uint32_t qmi_overflow_count;
    uint32_t qmi_serviced_while_pending;
    Issue3QmiClock qmi_clock;
    uint16_t qmi_fail_after;
    uint32_t qmi_reset_count;
    uint32_t qmi_lost_samples;
    uint8_t qmi_read_mode;
    uint8_t qmi_stream_ready;
    uint32_t qmi_defer_until_ms;
    uint8_t recording;

    Issue3GzpSchedule gzp_schedule;
    uint8_t gzp_pending;
    uint8_t gzp_result_ready;
    uint32_t gzp_plan_ms;
    uint32_t gzp_started_ms;
    uint32_t gzp_missed;
    uint32_t gzp_schedule_missed;
    uint32_t gzp_result_missed;
    uint32_t gzp_offgrid_starts;
    uint32_t gzp_expected[TEST_MAX_GZP];
    uint32_t gzp_plan_count;
    uint8_t sd_full;
    uint8_t sd_io;
    uint8_t allocation_terminal_count;
    uint8_t terminal_pending;
    uint8_t terminal_reason;
    uint32_t expected_qmi_persisted;
    uint32_t expected_gzp_persisted;
    uint32_t expected_icp_persisted;
    uint32_t expected_mts_persisted;
    uint32_t expected_qmi_ids[TEST_MAX_SAMPLES];
    uint32_t expected_qmi_id_count;
} CaptureMock;

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void fat_set(FatMock *fat, uint32_t cluster, uint32_t value)
{
    uint32_t lba = (cluster * 4U) / 512U;
    uint16_t off = (uint16_t)((cluster * 4U) & 511U);
    assert(lba < TEST_FAT_SECTORS);
    put32(&fat->fat[lba][off], value);
}

static int fat_read(void *ctx, uint32_t lba, uint8_t *buf)
{
    FatMock *fat = (FatMock *)ctx;
    assert(lba < TEST_FAT_SECTORS);
    fat->reads++;
    if (lba == fat->fail_lba) return -1;
    memcpy(buf, fat->fat[lba], 512);
    return 0;
}

static int startup_alloc_step(void *ctx, uint32_t *cluster)
{
    StartupMock *mock = (StartupMock *)ctx;
    int result = issue3_fat_scan_step(&mock->scan, 2, 1400, 0, 1,
                                      fat_read, mock->fat, mock->buf, cluster);
    if (result == ISSUE3_FAT_FOUND) fat_set(mock->fat, *cluster, 0x0FFFFFFFU);
    return result;
}

static void occupy_before(FatMock *fat, uint32_t first_free)
{
    for (uint32_t cluster = 2; cluster < first_free; cluster++)
        fat_set(fat, cluster, 0x0FFFFFFFU);
}

static void test_startup_allocation_case(uint32_t first_free)
{
    FatMock fat = {0};
    StartupMock mock = {0};
    fat.fail_lba = UINT32_MAX;
    occupy_before(&fat, first_free);
    mock.fat = &fat;
    mock.scan.cursor = 2;
    uint32_t cluster = 0;
    assert(issue3_fat_alloc_until_terminal(startup_alloc_step, &mock,
                                           &cluster) == ISSUE3_FAT_FOUND);
    assert(cluster == first_free);
    assert(mock.scan.complete == 0);
}

static void test_startup_allocation(void)
{
    test_startup_allocation_case(2);
    test_startup_allocation_case(512);
    test_startup_allocation_case(1024);
    test_startup_allocation_case(1280);

    FatMock full_fat = {0};
    StartupMock full = {0};
    full_fat.fail_lba = UINT32_MAX;
    occupy_before(&full_fat, 1402);
    full.fat = &full_fat;
    full.scan.cursor = 2;
    uint32_t cluster = 0;
    assert(issue3_fat_alloc_until_terminal(startup_alloc_step, &full,
                                           &cluster) == ISSUE3_FAT_FULL);
    assert(full.scan.complete == 1);

    FatMock io_fat = {0};
    StartupMock io = {0};
    io_fat.fail_lba = 0;
    io.fat = &io_fat;
    io.scan.cursor = 2;
    assert(issue3_fat_alloc_until_terminal(startup_alloc_step, &io,
                                           &cluster) == ISSUE3_FAT_IO);
}

static int prefetch_alloc(void *ctx, uint32_t *cluster)
{
    CaptureMock *capture = (CaptureMock *)ctx;
    int result = issue3_fat_scan_step(
        &capture->log_scan, 2, 1400, 0, 1, fat_read, capture->fat,
        capture->fat_buf, cluster);
    if (result == ISSUE3_FAT_FOUND) fat_set(capture->fat, *cluster, 0x0FFFFFFFU);
    capture->now_ms += capture->fat_read_ms;
    capture->simulated_sd_ms += capture->fat_read_ms;
    return result;
}

static int sector_write(void *ctx, uint32_t lba, const uint8_t *sector)
{
    CaptureMock *capture = (CaptureMock *)ctx;
    assert(lba < TEST_DISK_SECTORS);
    memcpy(capture->disk[lba], sector, 512);
    capture->disk_valid[lba] = 1;
    assert(capture->write_count < TEST_DISK_SECTORS);
    capture->write_order[capture->write_count++] = lba;
    capture->writes++;
    capture->now_ms += capture->sd_write_ms;
    capture->simulated_sd_ms += capture->sd_write_ms;
    return 0;
}

/* Mirror the firmware's runtime SD transaction boundary: a sector is copied
 * to the card only after 8-byte SPI steps have returned to the sensor loop.
 * The old test callback advanced the whole sector in one call and therefore
 * could not expose FIFO/GZP starvation. */
static void sd_write_step(CaptureMock *capture)
{
    if (!capture->sd_write_active) return;
    uint16_t left = (uint16_t)(512U - capture->sd_write_off);
    uint16_t chunk = issue3_sd_step_chunk(left);
    uint32_t step_ms = (capture->sd_write_ms + 63U) / 64U;
    if (step_ms == 0U) step_ms = 1U;
    capture->now_ms += step_ms;
    capture->simulated_sd_ms += step_ms;
    capture->sd_write_off = (uint16_t)(capture->sd_write_off + chunk);
    capture->sd_write_elapsed_ms += step_ms;
    if (capture->sd_write_off < 512U) return;

    uint32_t lba = capture->sd_write_lba;
    assert(lba < TEST_DISK_SECTORS);
    memcpy(capture->disk[lba], capture->sector_data, 512U);
    capture->disk_valid[lba] = 1U;
    assert(capture->write_count < TEST_DISK_SECTORS);
    capture->write_order[capture->write_count++] = lba;
    capture->writes++;
    capture->current_lba = lba + 1U;
    capture->sector.offset = 0U;
    capture->sector.total_written += ISSUE3_SECTOR_BYTES;
    capture->sd_write_active = 0U;
    capture->sd_write_off = 0U;
    uint32_t previous = capture->log.current_cluster;
    int advance = issue3_log_prefetch_advance_sector(&capture->log);
    if (advance == ISSUE3_LOG_NEEDS_SUCCESSOR)
        capture->boundary_pending = 1U;
    if (capture->log.current_cluster != previous)
        capture->links++;
}

static void note_gzp(void *ctx, uint32_t count)
{
    CaptureMock *capture = (CaptureMock *)ctx;
    capture->gzp_missed += count;
    capture->gzp_result_missed += count;
}

static void note_gzp_schedule(void *ctx, uint32_t count)
{
    CaptureMock *capture = (CaptureMock *)ctx;
    capture->gzp_missed += count;
    capture->gzp_schedule_missed += count;
}

static int start_gzp(void *ctx)
{
    CaptureMock *capture = (CaptureMock *)ctx;
    capture->gzp_pending = 1;
    capture->gzp_result_ready = 0;
    capture->gzp_plan_ms = capture->gzp_schedule.plan_ms;
    capture->gzp_started_ms = capture->now_ms;
    if (capture->gzp_plan_ms % ISSUE3_GZP_PERIOD_MS != 0U)
        capture->gzp_offgrid_starts++;
    return 0;
}

static void commit_frame(void *ctx, uint8_t slot_index, uint32_t start_ms)
{
    CaptureMock *capture = (CaptureMock *)ctx;
    assert(!capture->pending);
    uint8_t *frame = capture->slot_data[slot_index];
    memset(frame, 0, 674U);
    put32(&frame[0], TEST_FRAME_MAGIC);
    put32(&frame[4], start_ms);
    put16(&frame[8], capture->qmi_count[slot_index]);
    put16(&frame[10], capture->gzp_count[slot_index]);
    put32(&frame[12], capture->gzp_missed);
    for (uint16_t i = 0; i < capture->qmi_count[slot_index]; i++)
        put32(&frame[16U + i * 4U],
              capture->qmi_ids[slot_index][i]);
    for (uint16_t i = 0; i < capture->gzp_count[slot_index]; i++)
        put32(&frame[176U + i * 4U],
              capture->gzp_plans[slot_index][i]);
    put16(&frame[240], capture->icp_count[slot_index]);
    put16(&frame[242], capture->mts_count[slot_index]);
    assert(capture->icp_count[slot_index] <= TEST_ICP_PER_FRAME);
    for (uint16_t i = 0; i < capture->icp_count[slot_index]; i++) {
        put32(&frame[244U + i * 8U], capture->icp_id[slot_index][i]);
        put32(&frame[248U + i * 8U], capture->icp_ts[slot_index][i]);
    }
    capture->pending_slot = slot_index;
    capture->pending_off = 0;
    capture->pending = 1;
    issue3_frame_queue_claim_pending(&capture->queue, slot_index);
    capture->committed_frames++;
    capture->qmi_count[slot_index] = 0;
    capture->gzp_count[slot_index] = 0;
    capture->icp_count[slot_index] = 0;
    capture->mts_count[slot_index] = 0;
}

static uint8_t capture_frame_work_pending(const CaptureMock *capture)
{
    return (uint8_t)(capture->pending || capture->sd_write_active ||
                     capture->boundary_pending);
}

static int open_frame(CaptureMock *capture, uint32_t timestamp_ms,
                      uint8_t *slot, uint8_t *is_new)
{
    int result = issue3_frame_queue_open_runtime(
        &capture->queue, timestamp_ms, capture->now_ms,
        capture_frame_work_pending(capture), commit_frame, capture,
        slot, is_new);
    if (result < 0) return result;
    if (*is_new) {
        capture->qmi_count[*slot] = 0;
        capture->gzp_count[*slot] = 0;
        capture->icp_count[*slot] = 0;
        memset(capture->icp_id[*slot], 0, sizeof(capture->icp_id[*slot]));
        memset(capture->icp_ts[*slot], 0, sizeof(capture->icp_ts[*slot]));
        capture->mts_count[*slot] = 0;
    }
    return result;
}

static void add_qmi(CaptureMock *capture, uint32_t timestamp_ms, uint32_t id)
{
    uint8_t slot = 0, is_new = 0;
    int result = open_frame(capture, timestamp_ms, &slot, &is_new);
    assert(result >= 0);
    assert(capture->qmi_count[slot] < TEST_IDS_PER_FRAME);
    capture->qmi_ids[slot][capture->qmi_count[slot]++] = id;
}

static void add_gzp(CaptureMock *capture, uint32_t timestamp_ms)
{
    uint8_t slot = 0, is_new = 0;
    int result = open_frame(capture, timestamp_ms, &slot, &is_new);
    assert(result >= 0);
    assert(capture->gzp_count[slot] < TEST_GZP_PER_FRAME);
    capture->gzp_plans[slot][capture->gzp_count[slot]++] = timestamp_ms;
}

static void add_icp(CaptureMock *capture, uint32_t timestamp_ms,
                    uint32_t sample_id)
{
    uint8_t slot = 0, is_new = 0;
    assert(open_frame(capture, timestamp_ms, &slot, &is_new) >= 0);
    assert(capture->icp_count[slot] < TEST_ICP_PER_FRAME);
    uint16_t index = capture->icp_count[slot];
    capture->icp_id[slot][index] = sample_id;
    capture->icp_ts[slot][index] = timestamp_ms;
    capture->icp_count[slot]++;
}

static void push_icp_sample(CaptureMock *capture, uint32_t timestamp_ms)
{
    uint32_t sample_id = capture->icp_input_count;
    capture->icp_input_count++;
    if (capture->icp_fifo_count >= TEST_ICP_FIFO_CAPACITY) {
        capture->icp_lost_count++;
        return;
    }
    capture->icp_fifo_ts[capture->icp_fifo_count] = timestamp_ms;
    capture->icp_fifo_id[capture->icp_fifo_count] = sample_id;
    capture->icp_fifo_count++;
    assert(capture->icp_expected_count < TEST_MAX_ICP);
    capture->icp_expected_id[capture->icp_expected_count++] = sample_id;
}

static void push_icp_batch(CaptureMock *capture, uint32_t now_ms,
                           uint8_t samples)
{
    assert(samples > 0U && samples <= 16U);
    assert(capture->icp_fifo_count + samples <= TEST_ICP_FIFO_CAPACITY);
    uint32_t first_ms = 0, last_ms = 0;
    (void)issue3_icp_batch_span_fits(&capture->queue, now_ms, samples,
                                     &first_ms, &last_ms);
    for (uint8_t i = 0; i < samples; i++)
        push_icp_sample(capture, first_ms + (uint32_t)i * 500U);
}

static void service_icp_fifo(CaptureMock *capture, uint32_t now_ms)
{
    (void)now_ms;
    if (capture->icp_fifo_count == 0U) return;
    uint8_t level = capture->icp_fifo_count;
    uint8_t take = level > 4U ? 4U : level;
    uint8_t discard = 0U, admit = 0U;
    issue3_icp_batch_prefix_plan(&capture->queue,
                                 capture->icp_fifo_ts[0], take, take,
                                 &discard, &admit);
    if (discard == 0U && admit == 0U) return;
    for (uint8_t i = 0; i < (uint8_t)(discard + admit); i++) {
        if (i < discard) {
            capture->icp_lost_count++;
        } else {
            add_icp(capture, capture->icp_fifo_ts[i],
                    capture->icp_fifo_id[i]);
            capture->icp_consumed_count++;
        }
    }
    uint8_t consumed = (uint8_t)(discard + admit);
    if (consumed < level) {
        memmove(capture->icp_fifo_ts, capture->icp_fifo_ts + consumed,
                (size_t)(level - consumed) * sizeof(capture->icp_fifo_ts[0]));
        memmove(capture->icp_fifo_id, capture->icp_fifo_id + consumed,
                (size_t)(level - consumed) * sizeof(capture->icp_fifo_id[0]));
    }
    capture->icp_fifo_count = (uint8_t)(level - consumed);
}

static void produce_icp(CaptureMock *capture)
{
    while (capture->recording && capture->icp_next_ms <= capture->now_ms) {
        push_icp_sample(capture, capture->icp_next_ms);
        capture->icp_next_ms += 500U;
    }
}

static void produce_mts_result(CaptureMock *capture, uint32_t timestamp_ms)
{
    assert(capture->mts_input_count == 0U);
    capture->mts_input_count = 1U;
    capture->mts_result_ready = 1U;
    capture->mts_result_ms = timestamp_ms;
}

static void service_mts_result(CaptureMock *capture)
{
    if (!capture->mts_result_ready) return;
    uint8_t slot = 0, is_new = 0;
    int result = open_frame(capture, capture->mts_result_ms, &slot, &is_new);
    if (result < 0) return;
    capture->mts_count[slot]++;
    capture->mts_result_ready = 0U;
    capture->mts_consumed_count++;
}

/* Hardware-facing FIFO model: samples accumulate at the native ODR while
 * the main loop is busy.  The IRQ only becomes pending at the same four-
 * sample watermark configured by the firmware; service drains the complete
 * FIFO batch in acquisition order. */
static void qmi_fifo_push(CaptureMock *capture, uint32_t timestamp_ms,
                          uint32_t id)
{
    QmiFifoMock *fifo = &capture->qmi_fifo;
    if (fifo->count >= TEST_QMI_FIFO_CAPACITY) {
        fifo->overflow = 1;
        capture->qmi_overflow_count++;
        return; /* explicit test failure is asserted by each scenario */
    }
    uint16_t index = (uint16_t)((fifo->head + fifo->count) %
                                TEST_QMI_FIFO_CAPACITY);
    fifo->timestamp_ms[index] = timestamp_ms;
    fifo->id[index] = id;
    fifo->count++;
    if (fifo->count > fifo->max_count) fifo->max_count = fifo->count;
    if (fifo->count >= TEST_QMI_WATERMARK) fifo->irq_pending = 1;
    capture->qmi_input_count++;
}

static void produce_qmi(CaptureMock *capture)
{
    while (capture->recording && capture->qmi_next_id < TEST_MAX_SAMPLES &&
           capture->qmi_next_us <= (uint64_t)capture->now_ms * 1000ULL) {
        qmi_fifo_push(capture, (uint32_t)(capture->qmi_next_us / 1000ULL),
                      capture->qmi_next_id++);
        capture->qmi_next_us += ISSUE3_QMI_SAMPLE_PERIOD_US;
    }
}

static void service_qmi_fifo(CaptureMock *capture)
{
    QmiFifoMock *fifo = &capture->qmi_fifo;
    if (fifo->count == 0 || capture->now_ms < capture->qmi_defer_until_ms)
        return;
    if (capture->pending) capture->qmi_serviced_while_pending++;
    if (!fifo->irq_pending && capture->recording)
        return;
    /* qmi_fifo_service() in the firmware commits one due frame before it
     * enters CTRL9 read mode.  Mirror that ordering before draining bytes. */
    (void)issue3_frame_queue_commit_ready(&capture->queue,
                                           capture->now_ms,
                                           commit_frame, capture);
    if (fifo->overflow) {
        /* This is the firmware's explicit overflow/reset boundary.  The
         * normal scenarios assert that it is never reached. */
        capture->qmi_lost_samples += fifo->count;
        fifo->head = 0;
        fifo->count = 0;
        fifo->irq_pending = 0;
        fifo->overflow = 0;
        return;
    }
    uint16_t count = fifo->count;
    if (!capture->qmi_clock.valid)
        issue3_qmi_clock_anchor(&capture->qmi_clock,
                                (uint64_t)capture->now_ms * 1000ULL,
                                count);
    capture->qmi_read_mode = 1U;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t index = (uint16_t)((fifo->head + i) %
                                    TEST_QMI_FIFO_CAPACITY);
        if (capture->qmi_fail_after != 0xFFFFU &&
            i == capture->qmi_fail_after) {
            capture->qmi_read_mode = 0U;
            capture->qmi_lost_samples += count - i;
            capture->qmi_reset_count++;
            capture->qmi_stream_ready = 1U;
            issue3_qmi_clock_invalidate(&capture->qmi_clock);
            /* This is the firmware's partial-read recovery: reset FIFO,
             * restore stream mode and leave the pin low so a later
             * watermark produces a new rising edge. */
            fifo->head = 0;
            fifo->count = 0;
            fifo->irq_pending = 0;
            fifo->overflow = 0;
            capture->qmi_fail_after = 0xFFFFU;
            return;
        }
        /* Every raw FIFO item is consumed even when this individual sample
         * cannot be admitted to its frame.  A stale/pending point must not
         * reject the later suffix of the same hardware batch. */
        add_qmi(capture, fifo->timestamp_ms[index], fifo->id[index]);
    }
    capture->qmi_read_mode = 0U;
    fifo->head = 0;
    fifo->count = 0;
    fifo->irq_pending = 0;
    issue3_qmi_clock_complete_batch(&capture->qmi_clock);
}

static void service_gzp(CaptureMock *capture)
{
    uint8_t span_ok = issue3_frame_queue_span_fits(
        &capture->queue, capture->gzp_plan_ms, capture->gzp_plan_ms);
    if (capture->gzp_pending && !capture->gzp_result_ready &&
        capture->now_ms - capture->gzp_started_ms >= 20U &&
        !capture->boundary_pending) {
        capture->gzp_pending = 0U;
        capture->gzp_result_ready = 1U;
    }
    if (capture->gzp_result_ready &&
        issue3_frame_queue_timestamp_unrecoverable(
            &capture->queue, capture->gzp_plan_ms)) {
        capture->gzp_result_ready = 0U;
        note_gzp(capture, 1U);
    } else if (capture->gzp_result_ready &&
               !capture->boundary_pending && span_ok) {
        add_gzp(capture, capture->gzp_plan_ms);
        assert(capture->gzp_plan_count < TEST_MAX_GZP);
        capture->gzp_expected[capture->gzp_plan_count++] = capture->gzp_plan_ms;
        capture->gzp_result_ready = 0U;
    } else if (capture->gzp_result_ready &&
               issue3_time_reached(capture->now_ms,
                                   capture->gzp_plan_ms +
                                   (uint32_t)ISSUE3_FRAME_RETENTION_MS)) {
        /* A result can be late because the queue is full, but it must not
         * keep the scheduler in a permanent pending state. */
        capture->gzp_result_ready = 0U;
        note_gzp(capture, 1U);
    }
    if (!capture->gzp_pending && !capture->gzp_result_ready &&
        capture->recording) {
        uint32_t plan = 0;
        int decision = issue3_gzp_dispatch(
            &capture->gzp_schedule, capture->now_ms, start_gzp, capture,
            note_gzp_schedule, capture, &plan);
        assert(decision != ISSUE3_GZP_START_FAILED);
    }
}

static void handle_prefetch_result(CaptureMock *capture, int result)
{
    if (result == ISSUE3_LOG_PREFETCH_FULL) {
        capture->sd_full = 1;
        capture->terminal_pending = 1;
        capture->terminal_reason = 1U;
    } else if (result == ISSUE3_LOG_PREFETCH_IO) {
        capture->sd_io = 1;
        capture->terminal_pending = 1;
        capture->terminal_reason = 2U;
    }
}

/* FULL/IO is only terminal at the real cluster boundary.  The firmware
 * remembers the allocator result while the current cluster is still being
 * written; any incomplete frame is then released without exposing a partial
 * frame through the directory length. */
static void finish_terminal(CaptureMock *capture)
{
    if (capture->allocation_terminal_count != 0U) return;
    capture->allocation_terminal_count = 1U;
    capture->log.active = 0;
    capture->boundary_pending = 0;
    capture->metadata_size = capture->complete_size;
    if (capture->pending) {
        issue3_frame_queue_release_pending(&capture->queue,
                                           capture->pending_slot);
        capture->pending = 0;
    }
}

static void service_once(CaptureMock *capture)
{
    if (capture->sd_write_active) {
        sd_write_step(capture);
        return;
    }
    if (capture->boundary_pending) {
        uint8_t was_pending = capture->boundary_pending;
        if (capture->terminal_pending) {
            finish_terminal(capture);
            return;
        }
        int result = issue3_log_prefetch_step(&capture->log,
                                              prefetch_alloc, capture);
        handle_prefetch_result(capture, result);
        if (capture->terminal_pending) {
            finish_terminal(capture);
            return;
        }
        if (was_pending && result == ISSUE3_LOG_PREFETCH_READY) {
            assert(issue3_log_prefetch_link_ready(&capture->log) == 1);
            capture->boundary_pending = 0;
            capture->links++;
        }
        return;
    }
    if (capture->pending) {
        uint16_t chunk = (uint16_t)(674U - capture->pending_off);
        if (chunk > (uint16_t)(512U - capture->sector.offset))
            chunk = (uint16_t)(512U - capture->sector.offset);
        for (uint16_t i = 0U; i < chunk; i++)
            capture->sector_data[capture->sector.offset + i] =
                capture->slot_data[capture->pending_slot][capture->pending_off + i];
        capture->sector.offset = (uint16_t)(capture->sector.offset + chunk);
        capture->pending_off = (uint16_t)(capture->pending_off + chunk);
        if (capture->pending_off >= 674U) {
            capture->complete_size += 674U;
            issue3_frame_queue_release_pending(&capture->queue,
                                               capture->pending_slot);
            capture->pending = 0U;
        }
        if (capture->sector.offset == ISSUE3_SECTOR_BYTES) {
            capture->sd_write_active = 1U;
            capture->sd_write_lba = capture->current_lba;
            capture->sd_write_off = 0U;
            capture->sd_write_elapsed_ms = 0U;
            capture->sector.offset = ISSUE3_SECTOR_BYTES;
        }
        return;
    }
    if (issue3_frame_queue_commit_ready(&capture->queue, capture->now_ms,
                                        commit_frame, capture)) return;
    if (capture->log.active && !capture->log.next_ready) {
        if (capture->terminal_pending) return;
        int result = issue3_log_prefetch_step(&capture->log,
                                              prefetch_alloc, capture);
        handle_prefetch_result(capture, result);
    }
}

static void flush_tail(CaptureMock *capture)
{
    if (capture->sector.offset != 0U) {
        for (uint16_t i = capture->sector.offset; i < 512U; i++)
            capture->sector_data[i] = 0;
        assert(sector_write(capture, capture->current_lba,
                            capture->sector_data) == 0);
        capture->sector.offset = 0;
    }
    capture->metadata_size = capture->complete_size;
}

static uint8_t log_byte(const CaptureMock *capture, uint32_t offset)
{
    uint32_t sector = offset / 512U;
    uint16_t in_sector = (uint16_t)(offset % 512U);
    assert(sector < capture->write_count);
    uint32_t lba = capture->write_order[sector];
    assert(capture->disk_valid[lba]);
    return capture->disk[lba][in_sector];
}

static void verify_disk_log(const CaptureMock *capture)
{
    uint8_t seen[TEST_MAX_SAMPLES] = {0};
    uint32_t qmi_seen = 0;
    uint32_t parsed_gzp[TEST_MAX_GZP];
    uint32_t gzp_seen = 0;
    uint32_t icp_seen = 0, mts_seen = 0;
    uint8_t icp_expected_seen[TEST_MAX_ICP] = {0};
    uint32_t previous_icp_ts = 0U;
    uint8_t have_previous_icp_ts = 0U;
    assert(capture->metadata_size % 674U == 0U);
    uint32_t frame_count = capture->metadata_size / 674U;
    assert(frame_count <= capture->committed_frames);
    for (uint32_t frame = 0; frame < frame_count; frame++) {
        uint32_t base = frame * 674U;
        uint8_t header[16];
        for (uint16_t i = 0; i < sizeof(header); i++)
            header[i] = log_byte(capture, base + i);
        assert(get32(&header[0]) == TEST_FRAME_MAGIC);
        uint16_t qmi_count = get16(&header[8]);
        uint16_t gzp_count = get16(&header[10]);
        uint8_t aux[4];
        for (uint8_t b = 0; b < sizeof(aux); b++)
            aux[b] = log_byte(capture, base + 240U + b);
        icp_seen += get16(&aux[0]);
        mts_seen += get16(&aux[2]);
        uint16_t icp_count = get16(&aux[0]);
        assert(icp_count <= TEST_ICP_PER_FRAME);
        for (uint16_t i = 0; i < icp_count; i++) {
            uint8_t raw_id[4], raw_ts[4];
            for (uint8_t b = 0; b < 4U; b++) {
                raw_id[b] = log_byte(capture, base + 244U + i * 8U + b);
                raw_ts[b] = log_byte(capture, base + 248U + i * 8U + b);
            }
            uint32_t id = get32(raw_id);
            uint32_t ts = get32(raw_ts);
            uint32_t expected_index = capture->icp_expected_count;
            for (uint32_t j = 0U; j < capture->icp_expected_count; j++) {
                if (capture->icp_expected_id[j] == id) {
                    expected_index = j;
                    break;
                }
            }
            assert(expected_index < capture->icp_expected_count);
            assert(!icp_expected_seen[expected_index]);
            icp_expected_seen[expected_index] = 1U;
            if (have_previous_icp_ts)
                assert(ts == previous_icp_ts + 500U);
            previous_icp_ts = ts;
            have_previous_icp_ts = 1U;
        }
        assert(qmi_count <= TEST_IDS_PER_FRAME);
        assert(gzp_count <= TEST_GZP_PER_FRAME);
        for (uint16_t i = 0; i < qmi_count; i++) {
            uint8_t raw[4];
            for (uint8_t b = 0; b < 4; b++)
                raw[b] = log_byte(capture, base + 16U + i * 4U + b);
            uint32_t id = get32(raw);
            assert(id < capture->qmi_input_count && !seen[id]);
            seen[id] = 1;
            qmi_seen++;
        }
        for (uint16_t i = 0; i < gzp_count; i++) {
            uint8_t raw[4];
            for (uint8_t b = 0; b < 4; b++)
                raw[b] = log_byte(capture, base + 176U + i * 4U + b);
            uint32_t plan = get32(raw);
            assert(gzp_seen == 0 || plan > parsed_gzp[gzp_seen - 1U]);
            parsed_gzp[gzp_seen++] = plan;
        }
    }
    uint32_t expected_qmi = capture->expected_qmi_persisted != 0U ?
        capture->expected_qmi_persisted :
        (capture->qmi_input_count >= capture->qmi_lost_samples ?
         capture->qmi_input_count - capture->qmi_lost_samples : 0U);
    uint32_t expected_gzp = capture->expected_gzp_persisted != 0U ?
        capture->expected_gzp_persisted : capture->gzp_plan_count;
    uint32_t expected_icp = capture->expected_icp_persisted != 0U ?
        capture->expected_icp_persisted :
        (capture->icp_input_count >= capture->icp_lost_count ?
         capture->icp_input_count - capture->icp_lost_count : 0U);
    uint32_t expected_mts = capture->expected_mts_persisted != 0U ?
        capture->expected_mts_persisted : capture->mts_input_count;
    assert(qmi_seen == expected_qmi);
    assert(gzp_seen == expected_gzp);
    assert(icp_seen == expected_icp);
    assert(mts_seen == expected_mts);
    assert(capture->icp_consumed_count == expected_icp);
    for (uint32_t i = 0U; i < capture->icp_expected_count; i++)
        assert(icp_expected_seen[i] == 1U);
    assert(capture->mts_consumed_count == capture->mts_input_count);
    if (capture->expected_qmi_id_count != 0U) {
        assert(capture->expected_qmi_id_count == expected_qmi);
        for (uint32_t i = 0; i < capture->expected_qmi_id_count; i++) {
            uint32_t id = capture->expected_qmi_ids[i];
            assert(id < capture->qmi_input_count && seen[id] == 1);
        }
    } else if (capture->qmi_overflow_count == 0U &&
               capture->qmi_lost_samples == 0U) {
        for (uint32_t id = 0; id < expected_qmi; id++)
            assert(seen[id] == 1);
    }
    for (uint32_t i = 0; i < gzp_seen; i++)
        assert(parsed_gzp[i] == capture->gzp_expected[i]);
}

static void init_capture(CaptureMock *capture, FatMock *fat,
                          uint32_t first_free, uint32_t sd_write_ms,
                          uint32_t fat_read_ms, uint32_t qmi_defer_until_ms)
{
    fat->fail_lba = UINT32_MAX;
    occupy_before(fat, first_free);
    capture->fat = fat;
    capture->sd_write_ms = sd_write_ms;
    capture->fat_read_ms = fat_read_ms;
    capture->qmi_defer_until_ms = qmi_defer_until_ms;
    capture->qmi_fail_after = 0xFFFFU;
    capture->qmi_stream_ready = 1U;
    capture->qmi_clock.valid = 1U;
    capture->recording = 1;
    capture->log.active = 1;
    capture->log.current_cluster = 2;
    capture->log.sectors_per_cluster = TEST_CLUSTER_SPC;
    capture->log_scan.cursor = 2;
    capture->gzp_schedule.next_ms = 0;
    issue3_frame_queue_init(&capture->queue);
}

static void run_capture_case(const char *name, uint32_t sd_write_ms,
                             uint32_t fat_read_ms,
                             uint32_t qmi_defer_until_ms,
                             uint16_t minimum_fifo_depth,
                             uint8_t require_zero_qmi_overflow)
{
    assert(ISSUE3_FRAME_RETENTION_MS == 7489ULL);
    assert(ISSUE3_FRAME_QUEUE_CAPACITY == 8U);
    assert(ISSUE3_SD_READ_MAX_MS == 29ULL);
    assert(ISSUE3_SD_WRITE_MAX_MS == 13ULL);
    FatMock fat = {0};
    CaptureMock capture = {0};
    init_capture(&capture, &fat, 1024U, sd_write_ms, fat_read_ms,
                 qmi_defer_until_ms);

    const uint32_t run_until_ms = 12000U;
    uint32_t guard = 0;
    while (capture.now_ms < run_until_ms ||
           capture.qmi_next_id < TEST_MAX_SAMPLES || capture.pending ||
           capture.sd_write_active ||
           capture.queue.count != 0 || capture.gzp_pending ||
           capture.gzp_result_ready || capture.qmi_fifo.count != 0 ||
           capture.icp_fifo_count != 0) {
        capture.recording = capture.now_ms < run_until_ms ||
                            capture.qmi_next_id < TEST_MAX_SAMPLES;
        produce_qmi(&capture);
        produce_icp(&capture);
        service_gzp(&capture);
        service_qmi_fifo(&capture);
        service_icp_fifo(&capture, capture.now_ms);
        uint32_t before = capture.now_ms;
        service_once(&capture);
        if (capture.now_ms == before) capture.now_ms += 5U;
        assert(++guard < 300000U);
    }
    capture.recording = 0;
    service_qmi_fifo(&capture);
    service_icp_fifo(&capture, capture.now_ms);
    flush_tail(&capture);

    if (require_zero_qmi_overflow) {
        assert(capture.qmi_overflow_count == 0U);
    } else if (capture.qmi_overflow_count != 0U) {
        printf("%s：压力退化诊断，QMI FIFO 溢出 %lu 次（不视为正常采样通过）\n",
               name, (unsigned long)capture.qmi_overflow_count);
    }
    assert(capture.qmi_serviced_while_pending > 0U);
    assert(capture.qmi_fifo.count == 0);
    assert(capture.qmi_fifo.max_count >= minimum_fifo_depth);
    assert(capture.qmi_fifo.max_count <= TEST_QMI_FIFO_CAPACITY);
    if (require_zero_qmi_overflow)
        assert(capture.qmi_input_count == TEST_MAX_SAMPLES);
    assert(capture.writes > 0 && capture.simulated_sd_ms > 0);
    assert(capture.links > 0);
    assert(fat.reads > 1U);
    if (require_zero_qmi_overflow) {
        /* A normal bounded SD transaction must not turn foreground latency
         * into a GZP schedule loss.  The strict assertion is intentionally
         * separate from the declared worst-case diagnostic below. */
        assert(capture.gzp_missed == 0U);
        assert(capture.icp_lost_count == 0U);
        assert(capture.icp_consumed_count == capture.icp_input_count);
    } else {
        printf("%s：压力场景 GZP 网格计划 %lu、实际启动 %lu、落盘 %lu、漏采 %lu、迟到实际点 %lu（不视为正常通过）\n",
               name, (unsigned long)(capture.gzp_schedule.next_ms / 100U),
               (unsigned long)capture.gzp_plan_count,
               (unsigned long)capture.gzp_plan_count,
               (unsigned long)capture.gzp_missed,
               (unsigned long)capture.gzp_offgrid_starts);
    }
    assert(capture.gzp_schedule_missed + capture.gzp_plan_count -
           capture.gzp_offgrid_starts ==
           capture.gzp_schedule.next_ms / 100U);
    verify_disk_log(&capture);
    uint32_t qmi_persisted = capture.qmi_input_count >= capture.qmi_lost_samples ?
        capture.qmi_input_count - capture.qmi_lost_samples : 0U;
    printf("%s：QMI 输入 %lu/落盘 %lu，GZP 网格计划 %lu/实际启动 %lu/落盘 %lu/漏采 %lu，ICP 输入 %lu/落盘 %lu/漏采 %lu\n",
           name, (unsigned long)capture.qmi_input_count,
           (unsigned long)qmi_persisted,
           (unsigned long)(capture.gzp_schedule.next_ms / 100U),
           (unsigned long)capture.gzp_plan_count,
           (unsigned long)capture.gzp_plan_count,
           (unsigned long)capture.gzp_missed,
           (unsigned long)capture.icp_input_count,
           (unsigned long)capture.icp_consumed_count,
           (unsigned long)capture.icp_lost_count);
    if (!require_zero_qmi_overflow && capture.icp_lost_count != 0U)
        printf("%s：ICP 压力退化诊断，漏采 %lu（不视为正常通过）\n",
               name, (unsigned long)capture.icp_lost_count);
    if (require_zero_qmi_overflow) {
        printf("%s：通过（FIFO 峰值 %u，SD 模拟耗时 %lu ms）\n", name,
               capture.qmi_fifo.max_count,
               (unsigned long)capture.simulated_sd_ms);
    } else {
        printf("%s：压力场景诊断完成（FIFO 峰值 %u，SD 模拟耗时 %lu ms）\n",
               name, capture.qmi_fifo.max_count,
               (unsigned long)capture.simulated_sd_ms);
    }
}

static void seed_runtime_frames(CaptureMock *capture)
{
    /* Four 674-byte frames with two sectors per cluster force the fourth
     * frame to hit the real cluster boundary while still only partially
     * written.  The first three complete frames must remain parseable. */
    add_qmi(capture, 0U, 0U);
    add_qmi(capture, 1000U, 1U);
    add_qmi(capture, 2000U, 2U);
    add_qmi(capture, 3000U, 3U);
    capture->qmi_input_count = 4U;
    capture->now_ms = 20000U;
}

static void run_runtime_fat_case(const char *name, uint32_t first_free,
                                 uint32_t fail_lba, uint8_t expect_full)
{
    FatMock fat = {0};
    CaptureMock capture = {0};
    if (expect_full) {
        fat.fail_lba = UINT32_MAX;
        /* One free cluster remains for the first boundary; the subsequent
         * scan then completes the full data-cluster range. */
        occupy_before(&fat, 1401U);
        capture.fat = &fat;
        capture.sd_write_ms = 1U;
        capture.fat_read_ms = 1U;
        capture.log.active = 1;
        capture.log.current_cluster = 2;
        capture.log.sectors_per_cluster = TEST_CLUSTER_SPC;
        capture.log_scan.cursor = 2;
        issue3_frame_queue_init(&capture.queue);
    } else {
        init_capture(&capture, &fat, first_free, 1U, 1U, 0U);
        fat.fail_lba = fail_lba;
    }
    seed_runtime_frames(&capture);
    uint32_t guard = 0;
    while (capture.log.active) {
        service_once(&capture);
        assert(guard < 10000U);
    }
    /* A terminal result must not be emitted again after recording stops. */
    uint8_t terminal_count = capture.allocation_terminal_count;
    service_once(&capture);
    assert(capture.allocation_terminal_count == terminal_count);
    assert(terminal_count == 1);
    if (expect_full) {
        assert(capture.sd_full && !capture.sd_io);
        assert(capture.metadata_size == 2022U);
        capture.expected_qmi_persisted = 3U;
    } else {
        assert(!capture.sd_full && capture.sd_io);
        assert(capture.metadata_size == 674U);
        capture.expected_qmi_persisted = 1U;
    }
    flush_tail(&capture);
    verify_disk_log(&capture);
    printf("%s：通过\n", name);
}

static void test_runtime_fat_terminals(void)
{
    /* Multi-budget search keeps recording alive until the successor appears. */
    FatMock fat = {0};
    CaptureMock capture = {0};
    init_capture(&capture, &fat, 1024U, 1U, 1U, 0U);
    capture.pending = 0;
    uint32_t guard = 0;
    while (!capture.log.next_ready) {
        assert(capture.log.active && !capture.sd_full && !capture.sd_io);
        service_once(&capture);
        assert(++guard < 10000U);
    }
    assert(capture.log.active && capture.log.next_ready);
    run_runtime_fat_case("运行期 FAT 完整扫描确认满卡", 0U, UINT32_MAX, 1);
    run_runtime_fat_case("运行期 FAT I/O 失败独立终止", 0U, 0U, 0);
}

static void test_qmi_fifo_overflow_boundary(void)
{
    CaptureMock capture = {0};
    capture.recording = 1;
    capture.qmi_next_id = 0;
    for (uint16_t i = 0; i < TEST_QMI_FIFO_CAPACITY; i++)
        qmi_fifo_push(&capture, (uint32_t)i * 35U, i);
    qmi_fifo_push(&capture, 4480U, TEST_QMI_FIFO_CAPACITY);
    assert(capture.qmi_fifo.overflow && capture.qmi_overflow_count == 1);
    service_qmi_fifo(&capture);
    assert(!capture.qmi_fifo.overflow && capture.qmi_fifo.count == 0);
    puts("QMI FIFO 满容量溢出/清理边界：通过");
}

static void test_qmi_mid_batch_failure_recovery(void)
{
    FatMock fat = {0};
    CaptureMock capture = {0};
    init_capture(&capture, &fat, 1024U, 1U, 1U, 0U);

    /* Enter read mode for an eight-sample watermark batch, then fail on the
     * fifth FIFO_DATA transaction.  Four already decoded samples remain in
     * the queue; the unread suffix is discarded only by the explicit FIFO
     * reset/re-stream recovery path. */
    for (uint32_t i = 0; i < 8U; i++)
        qmi_fifo_push(&capture, i * 36U, i);
    capture.qmi_fail_after = 4U;
    service_qmi_fifo(&capture);
    assert(capture.qmi_read_mode == 0U);
    assert(capture.qmi_reset_count == 1U);
    assert(capture.qmi_lost_samples == 4U);
    assert(capture.qmi_stream_ready == 1U);
    assert(capture.qmi_fifo.count == 0U && !capture.qmi_fifo.irq_pending);
    assert(!capture.qmi_clock.valid && capture.qmi_clock.gap_pending);
    assert(capture.qmi_clock.reanchor_count == 1U);

    /* Reset makes the watermark line low.  A later native batch crosses the
     * watermark again, proving that an edge-triggered EXTI can wake service;
     * the invalid clock is anchored from this complete batch only. */
    capture.now_ms = 2000U;
    for (uint32_t i = 0; i < 8U; i++)
        qmi_fifo_push(&capture, 1700U + i * 36U, 8U + i);
    assert(capture.qmi_fifo.irq_pending);
    service_qmi_fifo(&capture);
    assert(capture.qmi_fifo.count == 0U);
    assert(capture.qmi_clock.valid && !capture.qmi_clock.gap_pending);
    assert(capture.qmi_clock.reanchor_count == 1U);

    capture.recording = 0;
    capture.now_ms = 20000U;
    uint32_t guard = 0;
    while (capture.pending || capture.queue.count != 0U) {
        service_once(&capture);
        assert(++guard < 10000U);
    }
    flush_tail(&capture);
    capture.expected_qmi_persisted = 12U;
    capture.expected_qmi_id_count = 12U;
    for (uint32_t i = 0; i < 4U; i++) capture.expected_qmi_ids[i] = i;
    for (uint32_t i = 0; i < 8U; i++)
        capture.expected_qmi_ids[4U + i] = 8U + i;
    verify_disk_log(&capture);
    puts("QMI FIFO 中途读取失败/复位/重新产生 watermark/重锚回归：通过");
}

static void test_pending_slot_ownership(void)
{
    FatMock fat = {0};
    CaptureMock capture = {0};
    init_capture(&capture, &fat, 1024U, 1U, 1U, 0U);

    /* Commit second 0 into slot 0, then write only its first sector. */
    add_qmi(&capture, 0U, 0U);
    capture.qmi_input_count = 1U;
    add_gzp(&capture, 0U);
    capture.gzp_expected[capture.gzp_plan_count++] = 0U;
    capture.now_ms = 8000U;
    assert(issue3_frame_queue_commit_ready(&capture.queue, capture.now_ms,
                                           commit_frame, &capture) == 1);
    assert(capture.pending && capture.pending_slot == 0U);
    uint8_t old_copy[674];
    memcpy(old_copy, capture.slot_data[capture.pending_slot], sizeof(old_copy));
    service_once(&capture);
    assert(capture.pending && capture.pending_off == 512U);

    /* While the old bytes are still being written, populate every other
     * queue slot and make all three producers try to wrap onto slot 0. */
    add_qmi(&capture, 1000U, 1U);
    add_gzp(&capture, 1000U);
    capture.gzp_expected[capture.gzp_plan_count++] = 1000U;
    uint32_t next_id = 2U;
    for (uint32_t second = 2000U; second <= 7000U; second += 1000U)
        add_qmi(&capture, second, next_id++);
    capture.qmi_input_count = next_id;
    assert(capture.queue.count == 7U);
    push_icp_batch(&capture, 8000U, 2U);
    produce_mts_result(&capture, 8000U);
    service_icp_fifo(&capture, 8000U);
    service_mts_result(&capture);
    /* The existing frame-7 ICP point is safe to consume immediately; the
     * future 8000 ms point remains in FIFO because slot 0 is still pending.
     * MTS at the same future second is likewise retained. */
    assert(capture.icp_fifo_count == 1U && capture.mts_result_ready);

    /* Finish the second sector and verify the source slot was never reused. */
    while (capture.pending) service_once(&capture);
    assert(memcmp(old_copy, capture.slot_data[0], sizeof(old_copy)) == 0);
    assert(!capture.queue.pending_valid);
    service_icp_fifo(&capture, 8000U);
    service_mts_result(&capture);
    assert(capture.icp_fifo_count == 0U && !capture.mts_result_ready);
    assert(capture.icp_consumed_count == 2U && capture.mts_consumed_count == 1U);
    add_qmi(&capture, 8000U, next_id++);
    capture.qmi_input_count = next_id;
    add_gzp(&capture, 8000U);
    capture.gzp_expected[capture.gzp_plan_count++] = 8000U;
    assert(capture.queue.count == 8U);

    /* Persist all queued frames from the actual slot-backed writer and then
     * parse the simulated SD bytes; no private pending copy is involved. */
    capture.now_ms = 20000U;
    uint32_t guard = 0;
    while (capture.pending || capture.queue.count != 0U) {
        service_once(&capture);
        assert(++guard < 10000U);
    }
    flush_tail(&capture);
    verify_disk_log(&capture);
    puts("pending 槽跨秒/部分扇区/ICP-MTS-QMI 所有权回归：通过");
}

static void test_full_queue_pending_existing_samples(void)
{
    FatMock fat = {0};
    CaptureMock capture = {0};
    init_capture(&capture, &fat, 1024U, 1U, 1U, 0U);

    /* Fill all eight retained seconds.  The SD/FAT operation remains pending,
     * but samples timestamped in an existing non-pending frame must still
     * reach that frame rather than becoming a false queue miss. */
    for (uint32_t second = 0U; second < 8000U; second += 1000U)
        add_qmi(&capture, second, second / 1000U);
    assert(capture.queue.count == ISSUE3_FRAME_QUEUE_CAPACITY);
    capture.sd_write_active = 1U;
    capture.boundary_pending = 1U;

    add_qmi(&capture, 3000U, 100U);
    add_gzp(&capture, 3000U);
    push_icp_batch(&capture, 3500U, 2U);
    service_icp_fifo(&capture, 3500U);

    uint8_t slot = 0U, is_new = 0U;
    assert(open_frame(&capture, 8000U, &slot, &is_new) ==
           ISSUE3_FRAME_OPEN_BLOCKED);
    assert(capture.queue.count == ISSUE3_FRAME_QUEUE_CAPACITY);
    assert(capture.qmi_count[3U] == 2U);
    assert(capture.gzp_count[3U] == 1U);
    assert(capture.icp_count[3U] == 2U);
    puts("SD pending 满队列下 GZP 单点/ICP 批次/QMI 批次追加回归：通过");
}

static void test_invalid_clock_batch_preflight(void)
{
    Issue3FrameQueue queue = {0};
    issue3_frame_queue_init(&queue);
    for (uint8_t i = 0; i < 7U; i++) queue.start_ms[i] = (uint32_t)i * 1000U;
    queue.count = 7U;
    issue3_frame_queue_claim_pending(&queue, 7U);
    uint16_t fifo_samples = 128U;
    uint32_t first_ms = 0, last_ms = 0;
    assert(!issue3_qmi_batch_span_fits(&queue, 0U, 0U, 7000000ULL,
                                       fifo_samples, &first_ms, &last_ms));
    assert(first_ms == 2468U && last_ms == 7000U);
    assert(fifo_samples == 128U); /* deferred before any FIFO_DATA read */
    (void)issue3_frame_queue_commit_oldest(&queue, 0, 0);
    (void)issue3_frame_queue_commit_oldest(&queue, 0, 0);
    assert(issue3_qmi_batch_span_fits(&queue, 0U, 0U, 7000000ULL,
                                      fifo_samples, &first_ms, &last_ms));
    puts("QMI 时钟失效/7 槽/128 样本整批预检：通过");
}

int main(void)
{
    test_startup_allocation();
    test_runtime_fat_terminals();
    test_qmi_fifo_overflow_boundary();
    test_qmi_mid_batch_failure_recovery();
    test_pending_slot_ownership();
    test_full_queue_pending_existing_samples();
    test_invalid_clock_batch_preflight();
    run_capture_case("常规 QMI FIFO/主循环/分步 SD 120ms 集成", 120U, 7U, 0U, 4U, 1U);
    run_capture_case("分步 SD 120ms/128 样本容量附近延迟批次", 120U, 7U, 4300U, 100U, 1U);
    run_capture_case("固件声明最坏 SD 边界",
                     (uint32_t)ISSUE3_SD_WRITE_MAX_MS,
                     (uint32_t)ISSUE3_SD_READ_MAX_MS, 0U, 4U, 0U);
    puts("Issue #3 完整采集-真实 QMI FIFO-主循环-分步 SD 落盘集成测试：正常场景通过；最坏上界单独报告");
    return 0;
}
