/*
 * Deterministic four-hour acquisition/storage integration run.
 *
 * This is intentionally a host-side executable, not a second scheduling
 * implementation: frame admission, FAT scan budgeting, sector buffering,
 * GZP grid decisions and QMI clock anchoring all call the inline C core in
 * 固件/源码/issue3_logic.h.  Only the sensor FIFO, SD media and monotonic
 * clock are mocks.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../固件/源码/issue3_logic.h"

#define LONG_RUN_MS (4UL * 60UL * 60UL * 1000UL)
#define LONG_QMI_FIFO 128U
#define LONG_QMI_PER_FRAME 40U
#define LONG_GZP_PER_FRAME 10U
#define LONG_ICP_PER_FRAME 2U
#define LONG_SPC 2U
#define LONG_CLUSTERS 20000U
#define LONG_FAT_SECTORS ((LONG_CLUSTERS * 4U + 511U) / 512U)
#define LONG_DISK_SECTORS 40000U

typedef struct {
    uint32_t ts[LONG_QMI_FIFO];
    uint32_t id[LONG_QMI_FIFO];
    uint16_t head, count, max_count;
    uint8_t irq_pending, overflow;
} LongQmiFifo;

typedef struct {
    Issue3FrameQueue queue;
    uint8_t slot_data[ISSUE3_FRAME_QUEUE_CAPACITY][674];
    uint16_t qmi_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t qmi_id[ISSUE3_FRAME_QUEUE_CAPACITY][LONG_QMI_PER_FRAME];
    uint16_t gzp_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t gzp_ts[ISSUE3_FRAME_QUEUE_CAPACITY][LONG_GZP_PER_FRAME];
    uint16_t icp_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint16_t mts_count[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t frame_seq; /* next sequence allocated for a new frame */
    uint32_t committed;

    uint32_t now_ms;
    uint64_t qmi_next_us;
    uint32_t qmi_next_id, qmi_input, qmi_lost;
    LongQmiFifo qmi_fifo;
    Issue3QmiClock qmi_clock;
    uint32_t qmi_resets;
    uint32_t qmi_serviced_while_pending;

    Issue3GzpSchedule gzp_schedule;
    uint8_t gzp_pending, gzp_result_ready;
    uint32_t gzp_plan_ms, gzp_started_ms;
    uint32_t gzp_input, gzp_persisted, gzp_missed;
    uint32_t gzp_schedule_missed, gzp_offgrid_starts;
    uint32_t gzp_offgrid_persisted;
    uint8_t gzp_pending_offgrid;
    uint32_t gzp_stop_tail_missed;

    uint32_t icp_next_ms, icp_input, icp_persisted, icp_lost;
    uint32_t icp_fifo_ts[16];
    uint8_t icp_fifo_count;
    uint32_t mts_next_ms, mts_input, mts_persisted;
    uint8_t mts_result_ready;
    uint32_t mts_result_ms;

    uint8_t pending;
    uint8_t pending_slot;
    uint16_t pending_off;
    Issue3SectorBuffer sector;
    uint8_t sector_data[512];
    uint32_t current_lba;
    uint32_t write_order[LONG_DISK_SECTORS];
    uint32_t write_count;
    uint32_t complete_size, metadata_size;

    Issue3LogPrefetch prefetch;
    Issue3FatScan fat_scan;
    uint8_t fat[LONG_FAT_SECTORS][512];
    uint8_t fat_buf[512];
    uint8_t *disk;
    uint8_t *disk_valid;
    uint8_t *stream;
    uint8_t log_active, terminal, boundary_pending;
    uint32_t link_count;
    uint8_t queue_max_count;
    uint32_t sd_read_ms, sd_write_ms;
} LongCapture;

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
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

static void fat_set(LongCapture *c, uint32_t cluster, uint32_t value)
{
    uint32_t lba = (cluster * 4U) / 512U;
    uint16_t off = (uint16_t)((cluster * 4U) & 511U);
    assert(lba < LONG_FAT_SECTORS);
    put32(&c->fat[lba][off], value);
}

static int fat_read(void *ctx, uint32_t lba, uint8_t *buf)
{
    LongCapture *c = (LongCapture *)ctx;
    if (lba >= LONG_FAT_SECTORS) return -1;
    memcpy(buf, c->fat[lba], 512U);
    /* Declared worst-case FAT-sector read; production QMI continues filling
     * during this elapsed time through the outer simulation loop. */
    c->now_ms += c->sd_read_ms;
    return 0;
}

static int alloc_step(void *ctx, uint32_t *cluster)
{
    LongCapture *c = (LongCapture *)ctx;
    int result = issue3_fat_scan_step(&c->fat_scan, 2U, LONG_CLUSTERS, 0U,
                                      1U, fat_read, c, c->fat_buf, cluster);
    if (result == ISSUE3_FAT_FOUND) fat_set(c, *cluster, 0x0FFFFFFFU);
    return result;
}

static int write_sector(void *ctx, uint32_t lba, const uint8_t *sector)
{
    LongCapture *c = (LongCapture *)ctx;
    assert(lba < LONG_DISK_SECTORS);
    memcpy(&c->disk[lba * 512U], sector, 512U);
    c->disk_valid[lba] = 1U;
    assert(c->write_count < LONG_DISK_SECTORS);
    memcpy(&c->stream[c->write_count * 512U], sector, 512U);
    c->write_order[c->write_count++] = lba;
    c->now_ms += c->sd_write_ms;
    return 0;
}

static void reset_slot(LongCapture *c, uint8_t slot, uint32_t start_ms,
                       uint32_t sequence)
{
    memset(c->slot_data[slot], 0, sizeof(c->slot_data[slot]));
    c->qmi_count[slot] = c->gzp_count[slot] = 0U;
    c->icp_count[slot] = c->mts_count[slot] = 0U;
    put32(&c->slot_data[slot][0], 0x33545354U);
    put32(&c->slot_data[slot][4], start_ms);
    put32(&c->slot_data[slot][8], sequence);
}

static void commit_frame(void *ctx, uint8_t slot, uint32_t start_ms)
{
    LongCapture *c = (LongCapture *)ctx;
    assert(!c->pending);
    uint8_t *frame = c->slot_data[slot];
    put32(&frame[0], 0x33545354U);
    put32(&frame[4], start_ms);
    /* The slot already received its identity when it was opened.  Committing
     * an older slot must not allocate or reuse a sequence number. */
    put16(&frame[12], c->qmi_count[slot]);
    put16(&frame[14], c->gzp_count[slot]);
    put16(&frame[16], c->icp_count[slot]);
    put16(&frame[18], c->mts_count[slot]);
    put32(&frame[20], c->gzp_missed);
    for (uint16_t i = 0; i < c->qmi_count[slot]; i++)
        put32(&frame[32U + i * 4U], c->qmi_id[slot][i]);
    for (uint16_t i = 0; i < c->gzp_count[slot]; i++)
        put32(&frame[200U + i * 4U], c->gzp_ts[slot][i]);
    c->pending = 1U;
    c->pending_slot = slot;
    c->pending_off = 0U;
    issue3_frame_queue_claim_pending(&c->queue, slot);
    c->committed++;
}

static int open_frame(LongCapture *c, uint32_t ts, uint8_t *slot)
{
    uint32_t old[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint8_t old_count = c->queue.count;
    for (uint8_t i = 0; i < old_count; i++)
        old[i] = c->queue.start_ms[issue3_frame_queue_index(&c->queue, i)];
    uint8_t is_new = 0U;
    int result = issue3_frame_queue_open_runtime(
        &c->queue, ts, c->now_ms,
        (uint8_t)(c->pending || c->boundary_pending),
        commit_frame, c, slot, &is_new);
    if (result < 0) return result;
    if (is_new) {
        for (uint8_t i = 0; i < c->queue.count; i++) {
            uint8_t index = issue3_frame_queue_index(&c->queue, i);
            uint8_t existed = 0U;
            for (uint8_t j = 0; j < old_count; j++)
                if (old[j] == c->queue.start_ms[index]) existed = 1U;
            if (!existed)
                reset_slot(c, index, c->queue.start_ms[index],
                           issue3_frame_sequence_take(&c->frame_seq));
        }
    }
    return result;
}

static void add_qmi(LongCapture *c, uint32_t ts, uint32_t id)
{
    uint8_t slot = 0U;
    if (open_frame(c, ts, &slot) < 0) { c->qmi_lost++; return; }
    if (c->qmi_count[slot] >= LONG_QMI_PER_FRAME) { c->qmi_lost++; return; }
    c->qmi_id[slot][c->qmi_count[slot]++] = id;
}

static void add_gzp(LongCapture *c, uint32_t ts)
{
    uint8_t slot = 0U;
    if (open_frame(c, ts, &slot) < 0) { c->gzp_missed++; return; }
    if (c->gzp_count[slot] >= LONG_GZP_PER_FRAME) { c->gzp_missed++; return; }
    c->gzp_ts[slot][c->gzp_count[slot]++] = ts;
    c->gzp_persisted++;
}

static void add_icp(LongCapture *c, uint32_t ts)
{
    uint8_t slot = 0U;
    if (open_frame(c, ts, &slot) < 0) { c->icp_lost++; return; }
    if (c->icp_count[slot] < LONG_ICP_PER_FRAME) {
        c->icp_count[slot]++;
        c->icp_persisted++;
    } else {
        c->icp_lost++;
    }
}

static void produce_qmi(LongCapture *c)
{
    while (c->qmi_next_us <= (uint64_t)c->now_ms * 1000ULL) {
        if (c->qmi_fifo.count >= LONG_QMI_FIFO) {
            c->qmi_fifo.overflow = 1U;
            c->qmi_lost++;
        } else {
            uint16_t index = (uint16_t)((c->qmi_fifo.head +
                                         c->qmi_fifo.count) % LONG_QMI_FIFO);
            c->qmi_fifo.ts[index] = (uint32_t)(c->qmi_next_us / 1000ULL);
            c->qmi_fifo.id[index] = c->qmi_next_id;
            c->qmi_fifo.count++;
            if (c->qmi_fifo.count > c->qmi_fifo.max_count)
                c->qmi_fifo.max_count = c->qmi_fifo.count;
            if (c->qmi_fifo.count >= 4U) c->qmi_fifo.irq_pending = 1U;
        }
        c->qmi_input++;
        c->qmi_next_id++;
        c->qmi_next_us += ISSUE3_QMI_SAMPLE_PERIOD_US;
    }
}

static void service_qmi(LongCapture *c)
{
    if (!c->qmi_fifo.count || !c->qmi_fifo.irq_pending) return;
    if (c->pending) c->qmi_serviced_while_pending++;
    /* Match main.c: enter FIFO read mode for every legal batch.  Admission is
     * decided one raw sample at a time so an old/pending prefix cannot strand
     * the later suffix in hardware. */
    (void)issue3_frame_queue_commit_ready(&c->queue, c->now_ms,
                                          commit_frame, c);
    if (c->qmi_fifo.overflow) {
        c->qmi_resets++;
        issue3_qmi_clock_invalidate(&c->qmi_clock);
        c->qmi_fifo.head = c->qmi_fifo.count = 0U;
        c->qmi_fifo.irq_pending = c->qmi_fifo.overflow = 0U;
        return;
    }
    uint16_t count = c->qmi_fifo.count;
    if (!c->qmi_clock.valid)
        issue3_qmi_clock_anchor(&c->qmi_clock, (uint64_t)c->now_ms * 1000ULL, count);
    for (uint16_t i = 0; i < count; i++) {
        uint16_t index = (uint16_t)((c->qmi_fifo.head + i) % LONG_QMI_FIFO);
        add_qmi(c, issue3_qmi_clock_sample_ms(&c->qmi_clock),
                c->qmi_fifo.id[index]);
    }
    c->qmi_fifo.head = c->qmi_fifo.count = 0U;
    c->qmi_fifo.irq_pending = 0U;
    issue3_qmi_clock_complete_batch(&c->qmi_clock);
}

static int start_gzp(void *ctx)
{
    LongCapture *c = (LongCapture *)ctx;
    c->gzp_pending = 1U;
    c->gzp_started_ms = c->now_ms;
    c->gzp_plan_ms = c->gzp_schedule.plan_ms;
    c->gzp_pending_offgrid =
        (uint8_t)(c->gzp_plan_ms % ISSUE3_GZP_PERIOD_MS != 0U);
    if (c->gzp_pending_offgrid) c->gzp_offgrid_starts++;
    c->gzp_input++;
    return 0;
}

static void note_gzp(void *ctx, uint32_t count)
{
    LongCapture *c = (LongCapture *)ctx;
    c->gzp_missed += count;
}

static void note_gzp_schedule(void *ctx, uint32_t count)
{
    LongCapture *c = (LongCapture *)ctx;
    c->gzp_missed += count;
    c->gzp_schedule_missed += count;
}

static void service_gzp(LongCapture *c)
{
    if (c->gzp_pending && c->now_ms - c->gzp_started_ms >= 20U) {
        c->gzp_pending = 0U;
        c->gzp_result_ready = 1U;
    }
    if (c->gzp_result_ready) {
        if (issue3_frame_queue_span_fits(&c->queue, c->gzp_plan_ms,
                                         c->gzp_plan_ms)) {
            add_gzp(c, c->gzp_plan_ms);
            if (c->gzp_pending_offgrid) c->gzp_offgrid_persisted++;
            c->gzp_result_ready = 0U;
        } else if (issue3_time_reached(c->now_ms,
                                       c->gzp_plan_ms +
                                       (uint32_t)ISSUE3_FRAME_RETENTION_MS)) {
            c->gzp_result_ready = 0U;
            note_gzp(c, 1U);
        }
    }
    if (c->gzp_pending || c->gzp_result_ready) return;
    uint32_t plan = 0U;
    (void)issue3_gzp_dispatch(&c->gzp_schedule, c->now_ms, start_gzp, c,
                              note_gzp_schedule, c, &plan);
}

static void produce_icp_mts(LongCapture *c, uint8_t recording)
{
    while (recording && c->icp_next_ms <= c->now_ms) {
        if (c->icp_fifo_count < 16U)
            c->icp_fifo_ts[c->icp_fifo_count++] = c->icp_next_ms;
        else c->icp_lost++;
        c->icp_input++;
        c->icp_next_ms += 500U;
    }
    if (recording && !c->mts_result_ready && c->mts_next_ms <= c->now_ms) {
        c->mts_result_ready = 1U;
        c->mts_result_ms = c->mts_next_ms;
        c->mts_input++;
        c->mts_next_ms += 1800000U;
    }
}

static void service_icp_mts(LongCapture *c)
{
    if (c->icp_fifo_count) {
        uint8_t take = c->icp_fifo_count > 4U ? 4U : c->icp_fifo_count;
        uint32_t first = 0U, last = 0U;
        uint32_t chunk_now = c->icp_fifo_ts[take - 1U];
        if (issue3_icp_batch_span_fits(&c->queue, chunk_now, take,
                                       &first, &last)) {
            for (uint8_t i = 0; i < take; i++) add_icp(c, c->icp_fifo_ts[i]);
            memmove(c->icp_fifo_ts, c->icp_fifo_ts + take,
                    (c->icp_fifo_count - take) * sizeof(uint32_t));
            c->icp_fifo_count = (uint8_t)(c->icp_fifo_count - take);
        }
    }
    if (c->mts_result_ready) {
        uint8_t slot = 0U;
        if (open_frame(c, c->mts_result_ms, &slot) >= 0) {
            c->mts_count[slot] = 1U;
            c->mts_persisted++;
            c->mts_result_ready = 0U;
        }
    }
}

static void service_sd(LongCapture *c)
{
    if (!c->log_active) return;
    if (c->pending) {
        uint16_t chunk = (uint16_t)(674U - c->pending_off);
        if (chunk > (uint16_t)(512U - c->sector.offset))
            chunk = (uint16_t)(512U - c->sector.offset);
        int result = issue3_sector_buffer_append(
            &c->sector, c->sector_data,
            &c->slot_data[c->pending_slot][c->pending_off], chunk,
            c->current_lba, write_sector, c);
        c->pending_off = (uint16_t)(c->pending_off + chunk);
        if (result > 0) {
            c->current_lba++;
            uint32_t previous_cluster = c->prefetch.current_cluster;
            int advance = issue3_log_prefetch_advance_sector(&c->prefetch);
            if (advance == ISSUE3_LOG_NEEDS_SUCCESSOR) c->boundary_pending = 1U;
            if (c->prefetch.current_cluster != previous_cluster &&
                c->prefetch.sector_index == 0U && advance > 0) {
                c->link_count++;
                c->current_lba = (c->prefetch.current_cluster - 2U) * LONG_SPC;
            }
        }
        if (c->pending_off == 674U) {
            c->complete_size += 674U;
            issue3_frame_queue_release_pending(&c->queue, c->pending_slot);
            c->pending = 0U;
        }
        return;
    }
    if (c->boundary_pending) {
        if (c->terminal) {
            c->metadata_size = c->complete_size;
            c->log_active = 0U;
            return;
        }
        int result = issue3_log_prefetch_step(&c->prefetch, alloc_step, c);
        if (result == ISSUE3_LOG_PREFETCH_READY) {
            assert(issue3_log_prefetch_link_ready(&c->prefetch));
            c->boundary_pending = 0U;
            c->current_lba = (c->prefetch.current_cluster - 2U) * LONG_SPC;
        } else if (result == ISSUE3_LOG_PREFETCH_FULL ||
                   result == ISSUE3_LOG_PREFETCH_IO) {
            c->terminal = 1U;
        }
        return;
    }
    if (issue3_frame_queue_commit_ready(&c->queue, c->now_ms,
                                        commit_frame, c)) return;
    if (!c->prefetch.next_ready) {
        int result = issue3_log_prefetch_step(&c->prefetch, alloc_step, c);
        if (result == ISSUE3_LOG_PREFETCH_FULL ||
            result == ISSUE3_LOG_PREFETCH_IO) c->terminal = 1U;
    }
}

static uint8_t log_byte(const LongCapture *c, uint32_t offset)
{
    uint32_t sector = offset / 512U;
    uint16_t in_sector = (uint16_t)(offset % 512U);
    assert(sector < c->write_count);
    return c->stream[sector * 512U + in_sector];
}

static void verify_log(LongCapture *c)
{
    uint8_t *seen = (uint8_t *)calloc(c->qmi_input ? c->qmi_input : 1U, 1U);
    uint32_t expected_gzp = LONG_RUN_MS / ISSUE3_GZP_PERIOD_MS;
    assert(seen != NULL);
    assert(c->metadata_size % 674U == 0U);
    uint32_t frames = c->metadata_size / 674U;
    uint32_t qmi_seen = 0U, gzp_seen = 0U, icp_seen = 0U, mts_seen = 0U;
    uint32_t gzp_duplicates = 0U, gzp_order_errors = 0U;
    uint32_t last_gzp = 0U;
    uint8_t have_gzp = 0U;
    uint32_t previous_start = 0U;
    uint32_t previous_sequence = 0U;
    uint32_t sequence_duplicates = 0U, sequence_gaps = 0U;
    for (uint32_t f = 0; f < frames; f++) {
        uint32_t base = f * 674U;
        uint8_t head[24];
        for (uint8_t i = 0; i < sizeof(head); i++) head[i] = log_byte(c, base + i);
        assert(get32(&head[0]) == 0x33545354U);
        uint32_t sequence = get32(&head[8]);
        if (f) {
            if (sequence <= previous_sequence) sequence_duplicates++;
            if (sequence != previous_sequence + 1U) sequence_gaps++;
        }
        assert(sequence == f);
        previous_sequence = sequence;
        uint32_t start = get32(&head[4]);
        if (f) assert(start == previous_start + 1000U);
        previous_start = start;
        uint16_t nq = get16(&head[12]);
        uint16_t ng = get16(&head[14]);
        icp_seen += get16(&head[16]);
        mts_seen += get16(&head[18]);
        for (uint16_t i = 0; i < nq; i++) {
            uint8_t raw[4];
            for (uint8_t j = 0; j < 4; j++) raw[j] = log_byte(c, base + 32U + i * 4U + j);
            uint32_t id = get32(raw);
            assert(id < c->qmi_input && !seen[id]);
            seen[id] = 1U; qmi_seen++;
        }
        for (uint16_t i = 0; i < ng; i++) {
            uint8_t raw[4];
            for (uint8_t j = 0; j < 4; j++) raw[j] = log_byte(c, base + 200U + i * 4U + j);
            uint32_t ts = get32(raw);
            if (have_gzp && ts <= last_gzp) {
                gzp_order_errors++;
            }
            if (ts > LONG_RUN_MS) {
                gzp_order_errors++;
            }
            last_gzp = ts;
            have_gzp = 1U;
            gzp_seen++;
        }
    }
    assert(qmi_seen + c->qmi_lost == c->qmi_input);
    assert(icp_seen + c->icp_lost == c->icp_input);
    assert(mts_seen == c->mts_persisted);
    assert(gzp_seen == c->gzp_persisted);
    assert(gzp_duplicates == 0U);
    assert(gzp_order_errors == 0U);
    assert(sequence_duplicates == 0U && sequence_gaps == 0U);
    if (gzp_seen + c->gzp_missed - c->gzp_offgrid_starts != expected_gzp)
        printf("GZP守恒异常：落盘=%lu 漏采=%lu 计划=%lu 启动=%lu\n",
               (unsigned long)gzp_seen, (unsigned long)c->gzp_missed,
               (unsigned long)expected_gzp, (unsigned long)c->gzp_input);
    assert(gzp_seen + c->gzp_missed - c->gzp_offgrid_starts ==
           expected_gzp);
    assert(c->queue.count == 0U && !c->pending);
    assert(c->qmi_fifo.count == 0U);
    assert(c->qmi_resets == 0U); /* no hidden permanent QMI stop */
    assert(c->qmi_serviced_while_pending > 0U);
    printf("帧序号统计：重复 %lu，缺口 %lu；GZP统计：计划 %lu，启动 %lu，落盘 %lu，漏采 %lu（停止尾部 %lu），重复 %lu，时序错误 %lu\n",
           (unsigned long)sequence_duplicates,
           (unsigned long)sequence_gaps,
           (unsigned long)expected_gzp, (unsigned long)c->gzp_input,
           (unsigned long)gzp_seen, (unsigned long)c->gzp_missed,
           (unsigned long)c->gzp_stop_tail_missed,
           (unsigned long)gzp_duplicates, (unsigned long)gzp_order_errors);
    free(seen);
}

static void run_scenario(const char *label, uint32_t read_ms,
                         uint32_t write_ms, uint8_t require_zero_gzp_loss)
{
    printf("开始%s（SD读 %lu ms，写 %lu ms）\n", label,
           (unsigned long)read_ms, (unsigned long)write_ms);
    fflush(stdout);
    LongCapture *c = (LongCapture *)calloc(1, sizeof(*c));
    assert(c != NULL);
    c->disk = (uint8_t *)calloc(LONG_DISK_SECTORS, 512U);
    c->disk_valid = (uint8_t *)calloc(LONG_DISK_SECTORS, 1U);
    c->stream = (uint8_t *)calloc(LONG_DISK_SECTORS, 512U);
    assert(c->disk && c->disk_valid && c->stream);
    issue3_frame_queue_init(&c->queue);
    c->qmi_clock.valid = 0U;
    c->qmi_next_us = 0U;
    c->icp_next_ms = 0U;
    c->mts_next_ms = 0U;
    c->gzp_schedule.next_ms = 0U;
    c->log_active = 1U;
    c->prefetch.active = 1U;
    c->prefetch.current_cluster = 2U;
    c->prefetch.sectors_per_cluster = LONG_SPC;
    c->fat_scan.cursor = 2U;
    c->current_lba = 0U;
    c->sd_read_ms = read_ms;
    c->sd_write_ms = write_ms;
    fat_set(c, 2U, 0x0FFFFFFFU);

    uint32_t guard = 0U;
    while (c->now_ms < LONG_RUN_MS || c->qmi_fifo.count || c->queue.count ||
           c->pending || c->gzp_pending || c->gzp_result_ready ||
           c->icp_fifo_count || c->mts_result_ready) {
        uint8_t recording = c->now_ms < LONG_RUN_MS;
        if (recording) {
            produce_qmi(c);
            produce_icp_mts(c, 1U);
        }
        if (recording || c->gzp_pending || c->gzp_result_ready)
            service_gzp(c);
        if (!recording && c->qmi_fifo.count)
            c->qmi_fifo.irq_pending = 1U; /* deterministic end-of-run drain */
        service_qmi(c);
        service_icp_mts(c);
        service_sd(c);
        if (c->queue.count > c->queue_max_count)
            c->queue_max_count = c->queue.count;
        if (c->now_ms < LONG_RUN_MS && c->now_ms == 0U) c->now_ms = 5U;
        else c->now_ms += 5U;
        assert(++guard < 20000000U);
    }
    if (c->sector.offset) {
        for (uint16_t i = c->sector.offset; i < 512U; i++) c->sector_data[i] = 0U;
        (void)write_sector(c, c->current_lba, c->sector_data);
        c->sector.offset = 0U;
    }
    /* The finite simulation stops at the exact four-hour boundary.  Any
     * grid point not reached by the scheduler before that boundary is an
     * explicit stop-tail miss, not an unaccounted sample. */
    {
        uint32_t expected_gzp = LONG_RUN_MS / ISSUE3_GZP_PERIOD_MS;
        uint32_t accounted = c->gzp_persisted + c->gzp_missed -
                             c->gzp_offgrid_starts;
        if (accounted < expected_gzp) {
            c->gzp_stop_tail_missed = expected_gzp - accounted;
            c->gzp_missed += c->gzp_stop_tail_missed;
        }
    }
    c->metadata_size = c->complete_size;
    assert(c->qmi_input > 400000U);
    assert(c->qmi_fifo.max_count >= 4U && c->qmi_fifo.max_count < LONG_QMI_FIFO);
    assert(c->queue_max_count >= 7U);
    assert(c->link_count > 1000U);
    if (require_zero_gzp_loss) {
        assert(c->gzp_missed == 0U);
        assert(c->qmi_lost == 0U);
        assert(c->icp_lost == 0U);
    } else {
        /* The declared maxima are timeout-edge successful operations.  This
         * is a fault/stress envelope, not a claim that 10 Hz remains
         * serviceable; every missed grid point must nevertheless be counted
         * and acquisition must resume without duplicate timestamps. */
        assert(c->gzp_missed > 0U);
    }
    verify_log(c);
    printf("%s：QMI输入 %lu/落盘 %lu，落盘帧 %lu，FIFO峰值 %u，QMI漏采 %lu，ICP输入 %lu/落盘 %lu/漏采 %lu\n",
           label,
           (unsigned long)c->qmi_input,
           (unsigned long)(c->qmi_input - c->qmi_lost),
           (unsigned long)(c->metadata_size / 674U),
           c->qmi_fifo.max_count, (unsigned long)c->qmi_lost,
           (unsigned long)c->icp_input, (unsigned long)c->icp_persisted,
           (unsigned long)c->icp_lost);
    free(c->disk_valid);
    free(c->stream);
    free(c->disk);
    free(c);
}

int main(void)
{
    run_scenario("4小时正常 SD 时序（GZP 目标场景）", 5U, 5U, 1U);
    run_scenario("4小时同步 SD 单步 20 ms 压力（仅诊断退化，不视为正常通过）",
                 20U, 20U, 0U);
    run_scenario("4小时 SD 声明上界压力（仅诊断退化，不视为采样通过）",
                 (uint32_t)ISSUE3_SD_READ_MAX_MS,
                 (uint32_t)ISSUE3_SD_WRITE_MAX_MS, 0U);
    return 0;
}
