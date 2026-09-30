/*
 * 四小时异步 SD 生产路径离散事件回归。
 *
 * 这里故意不使用旧的“整扇区一次写完”模拟：每个命令、token、数据块和
 * busy 轮询都通过 Issue3SdTransfer 推进，数据阶段每次最多 8 字节。传感器
 * 在每个步骤之间继续产数，QMI/GZP 在成功读取后通过持久二分相位写入 V4
 * 小帧，NF 也通过 Issue3SpiBus 与 SD 竞争同一条 SPI。
 * 测试结束后只从模拟卡上的目录、FAT 链和文件字节反解析结果。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../固件/源码/issue3_logic.h"

#define RUN_MS (4UL * 60UL * 60UL * 1000UL)
#define SECTOR 512U
#define FRAME_SIZE ISSUE3_FRAME_SIZE
#define FRAME_MAGIC ISSUE3_FRAME_MAGIC_LOG2
#define FRAME_VERSION ISSUE3_FRAME_VERSION
#define FRAME_HEADER ISSUE3_FRAME_HEADER_SIZE
#define QMI_N ISSUE3_FRAME_QMI_CAPACITY
#define GZP_N ISSUE3_FRAME_GZP_CAPACITY
#define ICP_N ISSUE3_FRAME_ICP_CAPACITY
#define FRAME_QMI_OFF ISSUE3_FRAME_QMI_OFFSET
#define FRAME_GZP_OFF ISSUE3_FRAME_GZP_OFFSET
#define FRAME_ICP_OFF ISSUE3_FRAME_ICP_OFFSET
#define FRAME_MTS_OFF ISSUE3_FRAME_MTS_OFFSET
#define FRAME_CRC_OFF ISSUE3_FRAME_CRC_OFFSET
#define SPC 64U
#define CLUSTERS 800U
#define FIRST_CLUSTER 2U
#define FAT0_LBA 1U
#define FAT_SECTORS (((CLUSTERS + 2U) * 4U + 511U) / 512U)
#define FAT1_LBA (FAT0_LBA + FAT_SECTORS)
#define ROOT_LBA (FAT1_LBA + FAT_SECTORS)
#define DATA_LBA (ROOT_LBA + 1U)
#define DISK_SECTORS (DATA_LBA + CLUSTERS * SPC)
#define QMI_WATERMARK 4U
#define ICP_FIFO_CAP 16U
#define NF_MAX_COPIES 32U
#define TIMER_POLL_PERIOD_MS 100U
#define QMI_STATUS_I2C_COST_US 300U
#define QMI_SAMPLE_I2C_COST_US 180U
#define ICP_POLL_I2C_COST_US 700U

enum { SDK_NONE = 0, SDK_DATA, SDK_FAT, SDK_ROOT, SDK_LINK };

/* Runtime FAT operations follow the same resumable phases as main.c.  The
 * scan scratch sector (fat_buf) is kept separate from the link sector
 * (sd_stage), so a link can invalidate a scan cache without losing bytes
 * needed by an in-flight asynchronous write. */
enum {
    RUNTIME_FAT_IDLE = 0,
    RUNTIME_FAT_READ,
    RUNTIME_FAT_MARK_PRIMARY,
    RUNTIME_FAT_MARK_SECONDARY
};

typedef struct {
    uint32_t ts_ms[128];
    uint32_t id[128];
    uint16_t head;
    uint16_t count;
    uint16_t max_count;
    uint8_t irq_pending;
    uint8_t overflow;
} QmiFifo;

typedef struct {
    uint32_t seq;
    uint32_t start_ms;
    uint8_t qmi_count;
    uint8_t gzp_count;
    uint8_t icp_count;
    uint8_t mts_valid;
    uint32_t qmi_id[QMI_N];
    uint32_t qmi_ts[QMI_N];
    uint32_t gzp_ts[GZP_N];
    uint32_t icp_ts[ICP_N];
    uint32_t icp_id[ICP_N];
    uint32_t mts_ts;
    uint16_t qmi_dropped;
    uint16_t gzp_missed;
    uint32_t status;
} FrameMeta;

typedef struct Capture Capture;

struct Capture {
    Issue3FrameQueue queue;
    uint8_t slots[ISSUE3_FRAME_QUEUE_CAPACITY][FRAME_SIZE];
    FrameMeta frames[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint32_t next_seq;
    uint64_t now_us;
    uint8_t recording;

    QmiFifo qmi;
    uint64_t qmi_next_us;
    uint32_t qmi_next_id;
    uint32_t qmi_input;
    uint32_t qmi_read;
    uint32_t qmi_persisted;
    uint32_t qmi_lost;
    uint32_t qmi_admission_drops;
    uint32_t qmi_drop_time;
    uint32_t qmi_drop_owner;
    uint32_t qmi_drop_queue;
    uint32_t qmi_fifo_resets;
    uint32_t qmi_overflow;
    uint8_t qmi_decimation_phase;
    uint8_t qmi_exti_enabled;
    uint8_t qmi_poll_due;
    uint32_t qmi_service_calls;
    uint64_t qmi_i2c_elapsed_us;
    uint8_t qmi_delay_started;
    uint64_t qmi_hold_until_us;

    Issue3GzpSchedule gzp;
    uint8_t gzp_pending;
    uint8_t gzp_ready;
    Issue3GzpResultQueue gzp_results;
    uint32_t gzp_started_ms;
    uint32_t gzp_plan_ms;
    uint32_t gzp_plan_count;
    uint32_t gzp_start_count;
    uint32_t gzp_persisted;
    uint32_t gzp_missed;
    uint32_t gzp_first_miss_ms;
    uint32_t gzp_note_calls;
    uint32_t gzp_block_logs;
    uint8_t gzp_decimation_phase;
    uint64_t last_loop_us;
    uint64_t previous_loop_us;

    uint32_t icp_next_ms;
    uint32_t icp_input;
    uint32_t icp_persisted;
    uint32_t icp_lost;
    uint8_t icp_poll_due;
    uint32_t icp_service_calls;
    uint64_t icp_i2c_elapsed_us;
    uint32_t icp_fifo_ts[ICP_FIFO_CAP];
    uint32_t icp_fifo_id[ICP_FIFO_CAP];
    uint8_t icp_fifo_count;
    uint32_t mts_next_ms;
    uint32_t mts_input;
    uint32_t mts_persisted;
    uint8_t mts_ready;
    uint32_t mts_ts;

    uint8_t *disk;
    uint8_t root[SECTOR];
    uint8_t fat_buf[SECTOR];
    uint8_t sd_stage[SECTOR];
    Issue3FatScan fat_scan;
    Issue3LogPrefetch prefetch;
    uint32_t fat_allocations;
    uint32_t fat_released;
    uint32_t fat_links;
    uint8_t data_boundary_pending;
    uint8_t fat_runtime_phase;
    uint32_t fat_runtime_sector;
    uint32_t fat_runtime_cluster;
    uint32_t runtime_fat_reads;
    uint32_t runtime_fat0_writes;
    uint32_t runtime_fat1_writes;
    uint8_t link_pending;
    uint8_t link_stage;
    uint8_t link_prefetch_advanced;
    uint32_t link_cluster;
    uint32_t link_next;
    uint32_t link_sector;

    Issue3SdTransfer sd;
    Issue3SpiBus spi;
    uint8_t sd_cs_low;
    uint8_t nf_cs_low;
    uint8_t sd_kind;
    uint32_t sd_lba;
    uint16_t sd_busy_left;
    uint16_t sd_data_index;
    uint32_t sd_steps;
    uint32_t sd_transactions;
    uint32_t sd_read_transactions;
    uint32_t sd_write_transactions;
    uint64_t sd_elapsed_us;
    uint64_t sd_total_elapsed_us;
    uint64_t sd_max_transaction_us;
    uint64_t wfi_count;
    uint64_t wfi_while_sd;
    uint64_t foreground_turns;
    uint32_t existing_pending_admissions;
    Issue3FrameOpenReason last_open_reason;
    uint64_t recording_stop_us;
    uint32_t next_timer_ms;
    uint8_t setup_mode;

    uint8_t sector_buf[SECTOR];
    uint16_t sector_fill;
    uint32_t data_cluster;
    uint16_t data_sector;
    uint32_t durable_bytes;
    uint32_t logical_bytes;
    uint32_t metadata_size;
    uint32_t metadata_target;
    uint8_t metadata_pending;
    uint8_t writer_pending;
    uint8_t writer_slot;
    uint16_t writer_off;
    uint32_t data_sectors_written;

    Issue3HeartbeatSchedule heartbeat;
    Issue3NfBroadcast broadcast;
    uint32_t nf_device_id;
    uint32_t nf_copy_count;
    uint32_t nf_logical_count;
    uint32_t nf_seq[NF_MAX_COPIES];
    uint8_t nf_copy[NF_MAX_COPIES];
    uint32_t nf_uptime[NF_MAX_COPIES];
    uint32_t nf_deferred;
    uint32_t nf_deferred_phase[ISSUE3_SD_PHASE_WRITE_BUSY + 1U];
    uint8_t sleep_irq_masked;
    uint32_t sleep_deadline_ms;
};

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t get_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(crc & 1U));
    }
    return ~crc;
}

static uint32_t fat_lba(uint32_t cluster)
{
    return FAT0_LBA + (cluster * 4U) / SECTOR;
}

static uint16_t fat_off(uint32_t cluster)
{
    return (uint16_t)((cluster * 4U) & (SECTOR - 1U));
}

static uint32_t fat_get(const Capture *c, uint32_t lba, uint16_t off)
{
    return get32(&c->disk[lba * SECTOR + off]) & 0x0FFFFFFFU;
}

/* Synchronous FAT mutation is confined to card setup and final validation
 * cleanup.  Runtime allocation/link paths below never call this helper. */
static void fat_put(Capture *c, uint32_t lba, uint16_t off, uint32_t value)
{
    put32(&c->disk[lba * SECTOR + off], value);
    put32(&c->disk[(lba + FAT_SECTORS) * SECTOR + off], value);
    if (c->fat_scan.cache_valid && c->fat_scan.cached_lba == lba)
        put32(&c->fat_buf[off], value);
}

static uint32_t data_lba(uint32_t cluster, uint16_t sector)
{
    return DATA_LBA + (cluster - FIRST_CLUSTER) * SPC + sector;
}

static uint8_t *disk_sector(Capture *c, uint32_t lba)
{
    assert(lba < DISK_SECTORS);
    return &c->disk[lba * SECTOR];
}

static int fat_scan_read_startup(void *ctx, uint32_t lba, uint8_t *buf)
{
    Capture *c = (Capture *)ctx;
    if (lba >= FAT1_LBA) return -1;
    memcpy(buf, disk_sector(c, lba), SECTOR);
    return 0;
}

static void sd_cs_cb(void *ctx, uint8_t high)
{
    Capture *c = (Capture *)ctx;
    if (high) {
        c->sd_cs_low = 0U;
        assert(!c->nf_cs_low);
    } else {
        assert(issue3_spi_bus_owned(&c->spi, ISSUE3_SPI_OWNER_SD));
        assert(!c->nf_cs_low);
        c->sd_cs_low = 1U;
    }
}

static uint8_t sd_xfer_cb(void *ctx, uint8_t out)
{
    Capture *c = (Capture *)ctx;
    Issue3SdTransfer *t = &c->sd;
    assert(issue3_spi_bus_owned(&c->spi, ISSUE3_SPI_OWNER_SD));
    /* start/finish exchange one trailing clock with CS high; the bus owner
     * remains SD until the transfer state is fully released. */
    (void)t;
    if (t->phase == ISSUE3_SD_PHASE_RESP) return 0x00U;
    if (t->phase == ISSUE3_SD_PHASE_TOKEN) return 0xFEU;
    if (t->phase == ISSUE3_SD_PHASE_READ_DATA)
        return disk_sector(c, t->lba)[c->sd_data_index++];
    if (t->phase == ISSUE3_SD_PHASE_WRITE_DATA) {
        c->sd_stage[c->sd_data_index++] = out;
        return 0x00U;
    }
    if (t->phase == ISSUE3_SD_PHASE_WRITE_RESP) return 0x05U;
    if (t->phase == ISSUE3_SD_PHASE_WRITE_BUSY) {
        if (c->sd_busy_left != 0U) {
            c->sd_busy_left--;
            return 0x00U;
        }
        return 0xFFU;
    }
    return 0xFFU;
}

static void sd_complete(Capture *c)
{
    if (c->sd.result != 0U) return;
    if (c->sd.op == ISSUE3_SD_OP_WRITE)
        memcpy(disk_sector(c, c->sd.lba), c->sd_stage, SECTOR);
    c->sd_transactions++;
    if (c->sd.op == ISSUE3_SD_OP_READ) c->sd_read_transactions++;
    else c->sd_write_transactions++;
}

static uint8_t sd_phase_data(const Issue3SdTransfer *t)
{
    return t->phase == ISSUE3_SD_PHASE_READ_DATA ||
           t->phase == ISSUE3_SD_PHASE_WRITE_DATA;
}

static uint32_t sd_step_us(const Issue3SdTransfer *t, uint16_t old_index)
{
    /* Model the production hardware SPI1 path: 2 MHz gives 4 us per byte;
     * ISSUE3_SD_SPI_XFER_MAX_US adds the measured register/CS/status-poll
     * overhead.  The old 225 us/byte software-SPI fiction is intentionally
     * gone, so this test cannot claim the obsolete ~4020 B/s throughput. */
    if (sd_phase_data(t)) {
        uint16_t left = (uint16_t)(SECTOR - old_index);
        uint16_t bytes = issue3_sd_step_chunk(left);
        return (uint32_t)bytes * (uint32_t)ISSUE3_SD_SPI_XFER_MAX_US;
    }
    return (uint32_t)ISSUE3_SD_SPI_XFER_MAX_US;
}

static int sd_start(Capture *c, uint8_t op, uint8_t kind, uint32_t lba,
                    uint8_t *read_buf, const uint8_t *write_buf)
{
    if (!issue3_spi_bus_try_acquire(&c->spi, ISSUE3_SPI_OWNER_SD)) return -1;
    if (issue3_sd_transfer_start(&c->sd, op, lba, read_buf, write_buf,
                                 sd_cs_cb, sd_xfer_cb, c) != 0) {
        issue3_spi_bus_release(&c->spi, ISSUE3_SPI_OWNER_SD);
        return -1;
    }
    c->sd_kind = kind;
    c->sd_lba = lba;
    c->sd_busy_left = 8U;
    c->sd_data_index = 0U;
    c->sd_elapsed_us = 0U;
    return 0;
}

static int sd_step(Capture *c)
{
    uint16_t old_index = c->sd.index;
    uint32_t elapsed = sd_step_us(&c->sd, old_index);
    int result = issue3_sd_transfer_step(&c->sd, sd_cs_cb, sd_xfer_cb, c);
    c->sd_steps++;
    if (!c->setup_mode) {
        c->now_us += elapsed;
        c->sd_elapsed_us += elapsed;
        c->sd_total_elapsed_us += elapsed;
        if (c->sd_elapsed_us > c->sd_max_transaction_us)
            c->sd_max_transaction_us = c->sd_elapsed_us;
    }
    if (!c->sd.active && c->sd.result_ready) {
        sd_complete(c);
        issue3_spi_bus_release(&c->spi, ISSUE3_SPI_OWNER_SD);
    }
    return result;
}

static int sd_take(Capture *c)
{
    int result = issue3_sd_transfer_take_result(&c->sd);
    if (!c->sd.active && !c->sd.result_ready)
        issue3_spi_bus_release(&c->spi, ISSUE3_SPI_OWNER_SD);
    return result;
}

static void runtime_allocation_ready(Capture *c)
{
    c->fat_allocations++;
    c->prefetch.next_cluster = c->fat_runtime_cluster;
    c->prefetch.next_ready = 1U;
    c->prefetch.allocation_pending = 0U;
    c->fat_runtime_phase = RUNTIME_FAT_IDLE;
}

/* Completion of one runtime FAT transaction.  This mirrors the production
 * allocator/link phases: a found entry is not exposed to the log writer until
 * both FAT mirrors are durable, and a cluster link is not committed until its
 * two mirror writes are durable. */
static void runtime_fat_sd_finished(Capture *c)
{
    assert(c->sd.result == 0U);
    if (c->sd_kind == SDK_FAT) {
        if (c->fat_runtime_phase == RUNTIME_FAT_READ) {
            c->fat_scan.cached_lba = c->fat_runtime_sector;
            c->fat_scan.cache_valid = 1U;
            c->runtime_fat_reads++;
            c->fat_runtime_phase = RUNTIME_FAT_IDLE;
        } else if (c->fat_runtime_phase == RUNTIME_FAT_MARK_PRIMARY) {
            c->runtime_fat0_writes++;
            c->fat_runtime_phase = RUNTIME_FAT_MARK_SECONDARY;
        } else if (c->fat_runtime_phase == RUNTIME_FAT_MARK_SECONDARY) {
            c->runtime_fat1_writes++;
            runtime_allocation_ready(c);
        } else {
            assert(!"unexpected runtime FAT phase");
        }
        return;
    }
    if (c->sd_kind == SDK_LINK) {
        if (c->link_stage == 0U) {
            c->runtime_fat_reads++;
            uint16_t off = fat_off(c->link_cluster);
            put32(&c->sd_stage[off], c->link_next);
            c->link_stage = 1U;
        } else if (c->link_stage == 1U) {
            c->runtime_fat0_writes++;
            c->link_stage = 2U;
        } else if (c->link_stage == 2U) {
            c->runtime_fat1_writes++;
            c->link_stage = 3U;
        } else {
            assert(!"unexpected runtime link phase");
        }
    }
}

/* Start one asynchronous FAT read/mark step.  The in-memory scan is the
 * shared issue3_fat_scan algorithm's bounded equivalent, while every card
 * access is issued through Issue3SdTransfer and resumed by later turns. */
static int runtime_fat_prefetch_step(Capture *c)
{
    Issue3LogPrefetch *p = &c->prefetch;
    if (!p->active || p->next_ready) return ISSUE3_LOG_PREFETCH_IDLE;
    if (c->sd.active) return ISSUE3_LOG_PREFETCH_BUDGET;
    if (c->fat_runtime_phase == RUNTIME_FAT_MARK_SECONDARY) {
        if (sd_start(c, ISSUE3_SD_OP_WRITE, SDK_FAT,
                     c->fat_runtime_sector + FAT_SECTORS, 0,
                     c->fat_buf) == 0)
            return ISSUE3_LOG_PREFETCH_BUDGET;
        return ISSUE3_LOG_PREFETCH_BUDGET;
    }
    if (c->fat_runtime_phase != RUNTIME_FAT_IDLE)
        return ISSUE3_LOG_PREFETCH_BUDGET;

    uint32_t limit = FIRST_CLUSTER + CLUSTERS;
    if (c->fat_scan.cursor < FIRST_CLUSTER || c->fat_scan.cursor >= limit)
        c->fat_scan.cursor = FIRST_CLUSTER;
    uint32_t lba = fat_lba(c->fat_scan.cursor);
    uint32_t sector_first = ((c->fat_scan.cursor * 4U) / SECTOR) * 128U;
    uint32_t begin = c->fat_scan.cursor > sector_first ?
                     c->fat_scan.cursor : sector_first;
    uint32_t end = sector_first + 128U;
    if (end > limit) end = limit;
    if (begin >= end) {
        c->fat_scan.cursor = FIRST_CLUSTER;
        c->fat_scan.scanned_clusters = 0U;
        return ISSUE3_LOG_PREFETCH_BUDGET;
    }

    if (!c->fat_scan.cache_valid || c->fat_scan.cached_lba != lba) {
        if (sd_start(c, ISSUE3_SD_OP_READ, SDK_FAT, lba, c->fat_buf, 0) != 0)
            return ISSUE3_LOG_PREFETCH_BUDGET;
        c->fat_runtime_sector = lba;
        c->fat_runtime_phase = RUNTIME_FAT_READ;
        p->allocation_pending = 1U;
        return ISSUE3_LOG_PREFETCH_BUDGET;
    }

    for (uint32_t cluster = begin; cluster < end; cluster++) {
        uint16_t off = fat_off(cluster);
        uint32_t value = get32(&c->fat_buf[off]) & 0x0FFFFFFFU;
        if (value == 0U) {
            put32(&c->fat_buf[off], 0x0FFFFFFFU);
            c->fat_runtime_cluster = cluster;
            c->fat_runtime_sector = lba;
            c->fat_scan.cursor = cluster + 1U < limit ? cluster + 1U :
                                  FIRST_CLUSTER;
            c->fat_scan.scanned_clusters = 0U;
            if (sd_start(c, ISSUE3_SD_OP_WRITE, SDK_FAT, lba, 0,
                         c->fat_buf) != 0)
                return ISSUE3_LOG_PREFETCH_BUDGET;
            c->fat_runtime_phase = RUNTIME_FAT_MARK_PRIMARY;
            p->allocation_pending = 1U;
            return ISSUE3_LOG_PREFETCH_BUDGET;
        }
    }

    c->fat_scan.scanned_clusters += end - begin;
    c->fat_scan.cursor = end < limit ? end : FIRST_CLUSTER;
    if (c->fat_scan.scanned_clusters >= CLUSTERS) {
        c->fat_scan.complete = 1U;
        p->sd_full = 1U;
        return ISSUE3_LOG_PREFETCH_FULL;
    }
    return ISSUE3_LOG_PREFETCH_BUDGET;
}

static void start_runtime_link(Capture *c, uint32_t old_cluster,
                               uint32_t next_cluster, uint8_t advanced)
{
    assert(!c->link_pending);
    issue3_fat_scan_cache_invalidate(&c->fat_scan);
    c->link_pending = 1U;
    c->link_stage = 0U;
    c->link_prefetch_advanced = advanced;
    c->link_cluster = old_cluster;
    c->link_next = next_cluster;
    c->link_sector = fat_lba(old_cluster);
}

static void service_runtime_link(Capture *c)
{
    if (!c->link_pending || c->sd.active) return;
    if (c->link_stage == 0U) {
        if (sd_start(c, ISSUE3_SD_OP_READ, SDK_LINK, c->link_sector,
                     c->sd_stage, 0) == 0)
            return;
    } else if (c->link_stage == 1U) {
        if (sd_start(c, ISSUE3_SD_OP_WRITE, SDK_LINK, c->link_sector,
                     0, c->sd_stage) == 0)
            return;
    } else if (c->link_stage == 2U) {
        if (sd_start(c, ISSUE3_SD_OP_WRITE, SDK_LINK,
                     c->link_sector + FAT_SECTORS, 0, c->sd_stage) == 0)
            return;
    } else {
        if (!c->link_prefetch_advanced)
            assert(issue3_log_prefetch_link_ready(&c->prefetch));
        c->data_cluster = c->link_next;
        c->data_sector = 0U;
        c->data_boundary_pending = 0U;
        c->fat_links++;
        c->link_pending = 0U;
        c->link_stage = 0U;
        c->link_prefetch_advanced = 0U;
    }
}

static void run_transfer(Capture *c, uint8_t op, uint8_t kind, uint32_t lba,
                         uint8_t *read_buf, const uint8_t *write_buf)
{
    assert(sd_start(c, op, kind, lba, read_buf, write_buf) == 0);
    while (c->sd.active) (void)sd_step(c);
    assert(sd_take(c) == 0);
}

static void init_root(Capture *c)
{
    memset(c->root, 0, SECTOR);
    memcpy(&c->root[0], "LOG0001 BIN", 11);
    c->root[11] = 0x20U;
    put16(&c->root[26], (uint16_t)FIRST_CLUSTER);
    put32(&c->root[28], 0U);
    run_transfer(c, ISSUE3_SD_OP_WRITE, SDK_ROOT, ROOT_LBA, 0, c->root);
}

static void prepare_fat_chain(Capture *c)
{
    /* Start with a genuinely free FAT.  The log allocator obtains its first
     * cluster here; every later successor is obtained through the shared
     * Issue3LogPrefetch/Issue3FatScan state during the asynchronous run. */
    memset(&c->fat_scan, 0, sizeof(c->fat_scan));
    c->fat_scan.cursor = FIRST_CLUSTER;
    uint32_t first = 0U;
    assert(issue3_fat_scan_step(&c->fat_scan, FIRST_CLUSTER, CLUSTERS,
                                FAT0_LBA, 1U, fat_scan_read_startup, c,
                                c->fat_buf, &first) == ISSUE3_FAT_FOUND);
    assert(first == FIRST_CLUSTER);
    fat_put(c, fat_lba(first), fat_off(first), 0x0FFFFFFFU);
    c->fat_allocations = 1U;
    issue3_fat_scan_cache_invalidate(&c->fat_scan);
    c->fat_scan.cursor = FIRST_CLUSTER + 1U;
    c->fat_scan.scanned_clusters = 0U;

    /* Exercise the same SD read/write transaction path used by FAT metadata
     * while materialising the initial mirror.  Runtime allocations update the
     * two mirrors in the simulated card and are checked from final bytes. */
    for (uint32_t s = 0; s < FAT_SECTORS; s++)
        run_transfer(c, ISSUE3_SD_OP_READ, SDK_FAT, FAT0_LBA + s,
                     c->fat_buf, 0);
    for (uint32_t s = 0; s < FAT_SECTORS; s++) {
        run_transfer(c, ISSUE3_SD_OP_WRITE, SDK_FAT, FAT0_LBA + s,
                     0, disk_sector(c, FAT0_LBA + s));
        run_transfer(c, ISSUE3_SD_OP_WRITE, SDK_FAT, FAT1_LBA + s,
                     0, disk_sector(c, FAT1_LBA + s));
    }
    assert(fat_get(c, fat_lba(FIRST_CLUSTER), fat_off(FIRST_CLUSTER)) ==
           0x0FFFFFFFU);
    c->prefetch.active = 1U;
    c->prefetch.current_cluster = FIRST_CLUSTER;
    c->prefetch.sectors_per_cluster = SPC;
    c->prefetch.sector_index = 0U;
    c->data_cluster = FIRST_CLUSTER;
    c->data_sector = 0U;
}

static void frame_init(FrameMeta *f, uint32_t start, uint32_t seq)
{
    memset(f, 0, sizeof(*f));
    f->start_ms = start;
    f->seq = seq;
}

static void commit_frame(void *ctx, uint8_t slot, uint32_t start_ms);
static uint8_t capture_sd_work_pending(const Capture *c, uint32_t now_ms);

static void frame_new_slots(Capture *c, uint8_t old_count,
                            const uint32_t *old_start)
{
    for (uint8_t i = 0; i < c->queue.count; i++) {
        uint8_t index = issue3_frame_queue_index(&c->queue, i);
        uint8_t existed = 0U;
        for (uint8_t j = 0; j < old_count; j++)
            if (old_start[j] == c->queue.start_ms[index]) existed = 1U;
        if (!existed)
        {
            uint32_t sequence = issue3_frame_sequence_take(&c->next_seq);
            frame_init(&c->frames[index], c->queue.start_ms[index], sequence);
        }
    }
}

static int open_frame(Capture *c, uint32_t ts, uint8_t *slot)
{
    uint32_t old_start[ISSUE3_FRAME_QUEUE_CAPACITY];
    uint8_t old_count = c->queue.count;
    for (uint8_t i = 0; i < old_count; i++)
        old_start[i] = c->queue.start_ms[
            issue3_frame_queue_index(&c->queue, i)];
    uint8_t is_new = 0U;
    uint32_t now_ms = (uint32_t)(c->now_us / 1000ULL);
    uint8_t sd_pending = capture_sd_work_pending(c, now_ms);
    Issue3FrameOpenReason reason = ISSUE3_FRAME_OPEN_REASON_NONE;
    int result = issue3_frame_queue_open_runtime_ex(
        &c->queue, ts, now_ms, sd_pending, commit_frame, c, slot, &is_new,
        &reason);
    c->last_open_reason = reason;
    if (sd_pending && c->queue.count == ISSUE3_FRAME_QUEUE_CAPACITY &&
        result == ISSUE3_FRAME_OPEN_EXISTING)
        c->existing_pending_admissions++;
    if (result < 0) return result;
    if (is_new) frame_new_slots(c, old_count, old_start);
    return result;
}

static int add_qmi(Capture *c, uint32_t ts, uint32_t id)
{
    uint8_t slot = 0U;
    if (open_frame(c, ts, &slot) < 0) {
        c->qmi_admission_drops++;
        if (c->last_open_reason == ISSUE3_FRAME_OPEN_REASON_TIME)
            c->qmi_drop_time++;
        else if (c->last_open_reason == ISSUE3_FRAME_OPEN_REASON_OWNER)
            c->qmi_drop_owner++;
        else
            c->qmi_drop_queue++;
        return 0;
    }
    FrameMeta *f = &c->frames[slot];
    assert(f->qmi_count < QMI_N);
    f->qmi_ts[f->qmi_count] = ts;
    f->qmi_id[f->qmi_count++] = id;
    c->qmi_persisted++;
    return 1;
}

static int add_gzp(Capture *c, uint32_t ts)
{
    uint8_t slot = 0U;
    int open = open_frame(c, ts, &slot);
    if (open < 0) return 0;
    FrameMeta *f = &c->frames[slot];
    assert(f->gzp_count < GZP_N);
    f->gzp_ts[f->gzp_count++] = ts;
    return 1;
}

static void add_icp(Capture *c, uint32_t ts, uint32_t id)
{
    uint8_t slot = 0U;
    assert(open_frame(c, ts, &slot) >= 0);
    FrameMeta *f = &c->frames[slot];
    assert(f->icp_count < ICP_N);
    f->icp_ts[f->icp_count] = ts;
    f->icp_id[f->icp_count++] = id;
}

static int add_mts(Capture *c, uint32_t ts)
{
    uint8_t slot = 0U;
    if (open_frame(c, ts, &slot) < 0) return 0;
    FrameMeta *f = &c->frames[slot];
    assert(!f->mts_valid);
    f->mts_valid = 1U;
    f->mts_ts = ts;
    return 1;
}

static void build_frame(Capture *c, uint8_t slot)
{
    FrameMeta *f = &c->frames[slot];
    uint8_t *out = c->slots[slot];
    memset(out, 0, FRAME_SIZE);
    put32(&out[0], FRAME_MAGIC);
    put16(&out[4], FRAME_VERSION);
    put16(&out[6], FRAME_HEADER);
    put32(&out[8], f->seq);
    put32(&out[12], f->start_ms);
    out[16] = f->qmi_count;
    out[17] = f->gzp_count;
    out[18] = f->icp_count;
    out[19] = f->mts_valid;
    put32(&out[20], f->status);
    put16(&out[24], f->qmi_dropped);
    put16(&out[26], 0U);
    put16(&out[28], 0U);
    put16(&out[30], f->gzp_missed);
    for (uint8_t i = 0; i < f->qmi_count; i++) {
        uint8_t *p = &out[FRAME_QMI_OFF + i * 16U];
        put32(p, f->qmi_ts[i]);
        put16(p + 4, (uint16_t)f->qmi_id[i]);
        put16(p + 6, (uint16_t)(f->qmi_id[i] >> 16));
    }
    for (uint8_t i = 0; i < f->gzp_count; i++) {
        uint8_t *p = &out[FRAME_GZP_OFF + i * 10U];
        put32(p, f->gzp_ts[i]);
        p[4] = 0x10U; p[5] = 0x20U; p[6] = 0x30U;
        put16(p + 7, 0x0040U);
    }
    for (uint8_t i = 0; i < f->icp_count; i++) {
        uint8_t *p = &out[FRAME_ICP_OFF + i * 10U];
        put32(p, f->icp_ts[i]);
        p[4] = (uint8_t)f->icp_id[i];
        p[5] = (uint8_t)(f->icp_id[i] >> 8);
        p[6] = (uint8_t)(f->icp_id[i] >> 16);
    }
    if (f->mts_valid) {
        put32(&out[FRAME_MTS_OFF], f->mts_ts);
        put16(&out[FRAME_MTS_OFF + 4U], 0x0100U);
    }
    put32(&out[FRAME_CRC_OFF], crc32(out, FRAME_CRC_OFF));
}

static void commit_frame(void *ctx, uint8_t slot, uint32_t start_ms)
{
    Capture *c = (Capture *)ctx;
    assert(!c->writer_pending);
    assert(c->frames[slot].start_ms == start_ms);
    build_frame(c, slot);
    c->writer_pending = 1U;
    c->writer_slot = slot;
    c->writer_off = 0U;
    c->logical_bytes += FRAME_SIZE;
    issue3_frame_queue_claim_pending(&c->queue, slot);
}

static void advance_data_position(Capture *c)
{
    c->data_sector++;
    c->data_sectors_written++;
    uint32_t old = c->prefetch.current_cluster;
    uint32_t next = c->prefetch.next_cluster;
    int result = issue3_log_prefetch_advance_sector(&c->prefetch);
    if (result == ISSUE3_LOG_PREFETCH_IDLE) return;
    if (result == ISSUE3_LOG_PREFETCH_READY) {
        assert(c->data_sector == SPC);
        assert(next >= FIRST_CLUSTER && next < FIRST_CLUSTER + CLUSTERS);
        /* The prefetch state has already advanced to the successor, but the
         * old->new FAT link must remain pending until both mirror writes have
         * completed through Issue3SdTransfer. */
        start_runtime_link(c, old, next, 1U);
    } else {
        assert(result == ISSUE3_LOG_NEEDS_SUCCESSOR);
        assert(c->data_sector == SPC);
        c->data_boundary_pending = 1U;
    }
}

static void metadata_maybe_schedule(Capture *c)
{
    uint32_t complete = (c->durable_bytes / FRAME_SIZE) * FRAME_SIZE;
    if (complete > c->logical_bytes) complete = c->logical_bytes;
    if (complete > c->metadata_size) {
        c->metadata_target = complete;
        c->metadata_pending = 1U;
    }
}

static void service_metadata(Capture *c)
{
    if (!c->metadata_pending || c->sd.active) return;
    put32(&c->root[28], c->metadata_target);
    if (sd_start(c, ISSUE3_SD_OP_WRITE, SDK_ROOT, ROOT_LBA, 0, c->root) == 0)
        c->metadata_pending = 2U;
}

static void service_writer(Capture *c)
{
    if (c->sd.active) return;
    if (c->metadata_pending == 2U) return;
    if (c->metadata_pending == 1U) {
        service_metadata(c);
        return;
    }
    if (c->sector_fill == SECTOR) {
        assert(!c->data_boundary_pending);
        uint32_t lba = data_lba(c->data_cluster, c->data_sector);
        if (sd_start(c, ISSUE3_SD_OP_WRITE, SDK_DATA, lba, 0,
                     c->sector_buf) == 0)
            c->sector_fill = (uint16_t)(SECTOR + 1U);
        return;
    }
    if (!c->writer_pending) return;
    uint16_t left = (uint16_t)(FRAME_SIZE - c->writer_off);
    uint16_t room = (uint16_t)(SECTOR - c->sector_fill);
    uint16_t n = left < room ? left : room;
    memcpy(&c->sector_buf[c->sector_fill],
           &c->slots[c->writer_slot][c->writer_off], n);
    c->sector_fill = (uint16_t)(c->sector_fill + n);
    c->writer_off = (uint16_t)(c->writer_off + n);
    if (c->sector_fill == SECTOR) {
        uint32_t lba = data_lba(c->data_cluster, c->data_sector);
        assert(sd_start(c, ISSUE3_SD_OP_WRITE, SDK_DATA, lba, 0,
                        c->sector_buf) == 0);
        c->sector_fill = (uint16_t)(SECTOR + 1U);
    }
    if (c->writer_off == FRAME_SIZE) {
        issue3_frame_queue_release_pending(&c->queue, c->writer_slot);
        c->writer_pending = 0U;
    }
}

static void on_sd_finished(Capture *c)
{
    if (c->sd_kind == SDK_FAT || c->sd_kind == SDK_LINK) {
        runtime_fat_sd_finished(c);
        return;
    }
    if (c->sd_kind != SDK_DATA) return;
    if (c->sector_fill == SECTOR + 1U) {
        c->sector_fill = 0U;
        c->durable_bytes += SECTOR;
        advance_data_position(c);
        metadata_maybe_schedule(c);
    }
}

static void service_sd(Capture *c)
{
    if (c->sd.active) {
        (void)sd_step(c);
        if (!c->sd.active && c->sd.result_ready) {
            assert(c->sd.result == 0U);
            on_sd_finished(c);
            (void)sd_take(c);
            if (c->sd_kind == SDK_ROOT && c->metadata_pending == 2U) {
                c->metadata_size = c->metadata_target;
                c->metadata_pending = 0U;
            }
        }
        return;
    }

    /* A cluster link has ownership priority over the background allocator and
     * frame writer.  Its read, FAT0 write and FAT1 write each occupy their own
     * resumable SD transaction, exactly as in the production path. */
    if (c->link_pending) {
        service_runtime_link(c);
        return;
    }

    if (c->prefetch.active && c->data_boundary_pending) {
        if (!c->prefetch.next_ready) {
            int result = runtime_fat_prefetch_step(c);
            if (result == ISSUE3_LOG_PREFETCH_FULL)
                assert(!"runtime FAT allocation failed");
            if (result == ISSUE3_LOG_PREFETCH_IO)
                assert(!"runtime FAT allocation I/O failed");
            if (c->sd.active || c->fat_runtime_phase != RUNTIME_FAT_IDLE)
                return;
        }
        if (c->prefetch.next_ready) {
            uint32_t old = c->prefetch.current_cluster;
            uint32_t next = c->prefetch.next_cluster;
            assert(next >= FIRST_CLUSTER && next < FIRST_CLUSTER + CLUSTERS);
            start_runtime_link(c, old, next, 0U);
        }
        return;
    }
    if (c->prefetch.active && !c->prefetch.next_ready) {
        int result = runtime_fat_prefetch_step(c);
        assert(result == ISSUE3_LOG_PREFETCH_READY ||
               result == ISSUE3_LOG_PREFETCH_BUDGET ||
               result == ISSUE3_LOG_PREFETCH_IDLE);
        if (c->sd.active || c->fat_runtime_phase != RUNTIME_FAT_IDLE)
            return;
    }
    service_writer(c);
    if (!c->sd.active && c->metadata_pending == 0U)
        (void)issue3_frame_queue_commit_ready(&c->queue,
                                              (uint32_t)(c->now_us / 1000ULL),
                                              commit_frame, c);
}

static void qmi_push(Capture *c, uint32_t ts_ms, uint32_t id)
{
    QmiFifo *q = &c->qmi;
    if (q->count >= 128U) {
        q->overflow = 1U;
        c->qmi_overflow++;
        c->qmi_lost++;
        return;
    }
    uint16_t index = (uint16_t)((q->head + q->count) % 128U);
    q->ts_ms[index] = ts_ms;
    q->id[index] = id;
    q->count++;
    if (q->count > q->max_count) q->max_count = q->count;
    /* This long run deliberately leaves EXTI disconnected.  The production
     * fallback must therefore discover the same FIFO through the independent
     * TIM3 poll flag rather than through a synthetic edge. */
    if (c->qmi_exti_enabled && q->count >= QMI_WATERMARK)
        q->irq_pending = 1U;
    c->qmi_input++;
}

static void timer3_poll_update(Capture *c)
{
    if (!c->recording) return;
    uint32_t now_ms = (uint32_t)(c->now_us / 1000ULL);
    if (now_ms < c->next_timer_ms) return;
    c->qmi_poll_due = 1U;
    c->icp_poll_due = 1U;
    c->next_timer_ms = ((now_ms / TIMER_POLL_PERIOD_MS) + 1U) *
                       TIMER_POLL_PERIOD_MS;
}

static void charge_i2c(Capture *c, uint32_t us)
{
    c->now_us += us;
}

static void produce_qmi(Capture *c)
{
    while (c->recording && c->qmi_next_us <= (uint64_t)RUN_MS * 1000ULL &&
           c->qmi_next_us <= c->now_us) {
        qmi_push(c, (uint32_t)(c->qmi_next_us / 1000ULL), c->qmi_next_id++);
        c->qmi_next_us += ISSUE3_QMI_SAMPLE_PERIOD_US;
    }
}

static void service_qmi(Capture *c)
{
    QmiFifo *q = &c->qmi;
    /* Once during the long run, deliberately defer a watermark IRQ for four
     * 4.2 seconds.  This reaches the fixed 128-sample hardware capacity without
     * overflowing it and exercises per-sample admission after a delayed
     * batch under the same queue and SD ownership rules. */
    if (c->recording && !c->qmi_delay_started && c->qmi_input >= 1000U) {
        c->qmi_delay_started = 1U;
        c->qmi_hold_until_us = c->now_us + 4200000ULL;
    }
    if (c->qmi_delay_started && c->now_us < c->qmi_hold_until_us) return;
    if (q->count == 0U) return;
    c->qmi_service_calls++;
    charge_i2c(c, QMI_STATUS_I2C_COST_US);
    c->qmi_i2c_elapsed_us += QMI_STATUS_I2C_COST_US;
    if (q->overflow) {
        /* The model has no I²C fault injection in this normal-path test, but
         * retain the production reset boundary so an injected overflow is
         * visible rather than silently treated as a clean drain. */
        c->qmi_fifo_resets++;
        q->head = 0U;
        q->count = 0U;
        q->irq_pending = 0U;
        q->overflow = 0U;
        return;
    }
    uint16_t take = q->count;
    for (uint16_t i = 0; i < take; i++) {
        charge_i2c(c, QMI_SAMPLE_I2C_COST_US);
        c->qmi_i2c_elapsed_us += QMI_SAMPLE_I2C_COST_US;
        uint16_t index = (uint16_t)((q->head + i) % 128U);
        c->qmi_read++;
        /* Consume every raw FIFO sample.  The persistent decimator is applied
         * only after the read, and one rejected retained sample cannot stop
         * later samples in this same hardware batch. */
        if (issue3_decimation_keep_next(&c->qmi_decimation_phase))
            (void)add_qmi(c, q->ts_ms[index], q->id[index]);
    }
    q->head = (uint16_t)((q->head + take) % 128U);
    q->count = 0U;
    q->irq_pending = 0U;
}

static void produce_icp(Capture *c)
{
    while (c->recording && c->icp_next_ms <= RUN_MS &&
           c->icp_next_ms <= (uint32_t)(c->now_us / 1000ULL)) {
        if (c->icp_fifo_count == ICP_FIFO_CAP) c->icp_lost++;
        else {
            c->icp_fifo_ts[c->icp_fifo_count] = c->icp_next_ms;
            c->icp_fifo_id[c->icp_fifo_count] = c->icp_input;
            c->icp_fifo_count++;
        }
        c->icp_input++;
        c->icp_next_ms += 500U;
    }
}

static void service_icp(Capture *c)
{
    if (c->icp_fifo_count == 0U) return;
    c->icp_service_calls++;
    charge_i2c(c, ICP_POLL_I2C_COST_US);
    c->icp_i2c_elapsed_us += ICP_POLL_I2C_COST_US;
    uint8_t take = c->icp_fifo_count > ICP_N ? ICP_N : c->icp_fifo_count;
    uint8_t discard = 0U, admit = 0U;
    issue3_icp_batch_prefix_plan(&c->queue, c->icp_fifo_ts[0], take, take,
                                 &discard, &admit);
    if (discard == 0U && admit == 0U) return;
    for (uint8_t i = 0; i < (uint8_t)(discard + admit); i++) {
        if (i < discard) {
            c->icp_lost++;
        } else {
            add_icp(c, c->icp_fifo_ts[i], c->icp_fifo_id[i]);
            c->icp_persisted++;
        }
    }
    uint8_t consumed = (uint8_t)(discard + admit);
    if (consumed < c->icp_fifo_count) {
        memmove(c->icp_fifo_ts, c->icp_fifo_ts + consumed,
                (c->icp_fifo_count - consumed) * sizeof(c->icp_fifo_ts[0]));
        memmove(c->icp_fifo_id, c->icp_fifo_id + consumed,
                (c->icp_fifo_count - consumed) * sizeof(c->icp_fifo_id[0]));
    }
    c->icp_fifo_count = (uint8_t)(c->icp_fifo_count - consumed);
}

static void note_gzp(void *ctx, uint32_t count)
{
    Capture *c = (Capture *)ctx;
    if (count && c->gzp_missed == 0U)
        c->gzp_first_miss_ms = (uint32_t)(c->now_us / 1000ULL);
    c->gzp_note_calls++;
    c->gzp_missed += count;
}

static int start_gzp(void *ctx)
{
    Capture *c = (Capture *)ctx;
    c->gzp_pending = 1U;
    c->gzp_started_ms = (uint32_t)(c->now_us / 1000ULL);
    c->gzp_plan_ms = c->gzp.plan_ms;
    c->gzp_start_count++;
    return 0;
}

static void service_gzp(Capture *c)
{
    uint32_t now = (uint32_t)(c->now_us / 1000ULL);
    if (c->gzp_pending && now - c->gzp_started_ms >= 20U) {
        c->gzp_pending = 0U;
        if (issue3_decimation_keep_next(&c->gzp_decimation_phase)) {
            assert(issue3_gzp_result_queue_push(&c->gzp_results,
                                                c->gzp_plan_ms, 0x00302010U,
                                                0x0040));
        }
        c->gzp_ready = c->gzp_results.count != 0U;
    }
    if (c->gzp_results.count != 0U) {
        const Issue3GzpResult *result = issue3_gzp_result_queue_peek(
            &c->gzp_results);
        assert(result);
        if (issue3_frame_queue_timestamp_unrecoverable(
                &c->queue, result->timestamp_ms)) {
            c->gzp_missed++;
            issue3_gzp_result_queue_pop(&c->gzp_results);
            c->gzp_ready = c->gzp_results.count != 0U;
        } else if (add_gzp(c, result->timestamp_ms)) {
            issue3_gzp_result_queue_pop(&c->gzp_results);
            c->gzp_ready = c->gzp_results.count != 0U;
            c->gzp_persisted++;
        } else if (issue3_time_reached(
                       now, result->timestamp_ms +
                                (uint32_t)ISSUE3_FRAME_RETENTION_MS)) {
            c->gzp_missed++;
            issue3_gzp_result_queue_pop(&c->gzp_results);
            c->gzp_ready = c->gzp_results.count != 0U;
        }
    }
    if (c->recording && !c->gzp_pending &&
        c->gzp_results.count < ISSUE3_GZP_RESULT_QUEUE_CAPACITY &&
        now < RUN_MS && now >= c->gzp.next_ms) {
        uint32_t plan = 0U;
        int decision = issue3_gzp_dispatch(&c->gzp, now, start_gzp, c,
                                           note_gzp, c, &plan);
        if (decision == ISSUE3_GZP_START) {
            c->gzp_plan_count++;
            c->gzp_plan_ms = plan;
        }
    }
}

static void service_mts(Capture *c)
{
    uint32_t now = (uint32_t)(c->now_us / 1000ULL);
    if (c->recording && c->mts_next_ms <= RUN_MS && now >= c->mts_next_ms) {
        c->mts_ready = 1U;
        c->mts_ts = c->mts_next_ms;
        c->mts_input++;
        c->mts_next_ms += 1800000U;
    }
    if (c->mts_ready) {
        if (add_mts(c, c->mts_ts)) {
            c->mts_ready = 0U;
            c->mts_persisted++;
        }
    }
}

static void nf_service(Capture *c)
{
    uint32_t now = (uint32_t)(c->now_us / 1000ULL);
    if (!c->broadcast.active && !c->recording) return;
    if (!c->broadcast.active && c->recording && now < RUN_MS) {
        uint32_t sequence = 0U;
        if (issue3_heartbeat_startup(&c->heartbeat, &sequence) ||
            issue3_heartbeat_periodic_due(&c->heartbeat, now, &sequence))
            assert(issue3_nf_broadcast_begin(&c->broadcast, c->nf_device_id,
                                             sequence, now));
    }
    uint32_t sequence = 0U;
    uint8_t copy = 0U;
    if (!issue3_nf_broadcast_due(&c->broadcast, now, &sequence, &copy)) return;
    if (c->spi.owner != ISSUE3_SPI_OWNER_NONE) {
        c->nf_deferred++;
        if (c->sd.phase <= ISSUE3_SD_PHASE_WRITE_BUSY)
            c->nf_deferred_phase[c->sd.phase]++;
        return;
    }
    assert(issue3_spi_bus_try_acquire(&c->spi, ISSUE3_SPI_OWNER_NF));
    c->nf_cs_low = 1U;
    assert(!c->sd_cs_low);
    uint8_t payload[32];
    issue3_heartbeat_build_payload(payload, c->nf_device_id, 3U, 1U,
                                   sequence, (uint32_t)(c->now_us / 1000ULL),
                                   1U, 1U, 1U, 1U, 0U);
    assert(c->nf_copy_count < NF_MAX_COPIES);
    c->nf_seq[c->nf_copy_count] = sequence;
    c->nf_copy[c->nf_copy_count] = copy;
    c->nf_uptime[c->nf_copy_count] = get_be32(&payload[9]);
    c->nf_copy_count++;
    c->nf_cs_low = 0U;
    issue3_spi_bus_release(&c->spi, ISSUE3_SPI_OWNER_NF);
    issue3_nf_broadcast_complete(&c->broadcast);
    if (!c->broadcast.active) {
        c->nf_logical_count++;
        issue3_heartbeat_complete(&c->heartbeat);
    }
}

static void flush_partial_sector(Capture *c)
{
    if (c->sector_fill == 0U) return;
    while (c->sd.active) service_sd(c);
    memset(&c->sector_buf[c->sector_fill], 0, SECTOR - c->sector_fill);
    c->sector_fill = SECTOR;
    service_sd(c);
    while (c->sd.active) service_sd(c);
    assert(!c->sd.result_ready);
}

static void release_prefetch_tail(Capture *c)
{
    if (!c->prefetch.next_ready) return;
    /* This is end-of-capture validation cleanup, not the runtime allocator:
     * all production allocations/links above have already gone through the
     * asynchronous FAT mirror transactions. */
    assert(!c->recording && !c->sd.active && !c->link_pending);
    fat_put(c, fat_lba(c->prefetch.next_cluster),
            fat_off(c->prefetch.next_cluster), 0U);
    c->prefetch.next_ready = 0U;
    c->prefetch.next_cluster = 0U;
    c->fat_released++;
}

/* Build the same foreground/WFI decision that main.c makes.  In particular,
 * an active SD slice, a ready FIFO, a completed GZP conversion, or a due
 * frame commit keeps the simulated CPU running; only an empty foreground
 * with an interrupt-backed deadline advances directly to that wake event. */
static uint8_t capture_sd_work_pending(const Capture *c, uint32_t now_ms)
{
    if (c->sd.active || c->sd.result_ready || c->link_pending ||
        c->fat_runtime_phase != RUNTIME_FAT_IDLE || c->writer_pending ||
        c->sector_fill == SECTOR || c->metadata_pending ||
        c->data_boundary_pending)
        return 1U;
    if (c->prefetch.active && !c->prefetch.next_ready &&
        c->prefetch.allocation_pending && !c->prefetch.sd_full)
        return 1U;
    if (!c->queue.pending_valid && c->queue.count != 0U &&
        issue3_frame_commit_due(c->queue.start_ms[c->queue.head], now_ms))
        return 1U;
    return 0U;
}

static Issue3RuntimeWork capture_runtime_work(const Capture *c)
{
    uint32_t now_ms = (uint32_t)(c->now_us / 1000ULL);
    uint32_t nf_sequence = 0U;
    uint8_t nf_copy = 0U;
    uint8_t nf_ready = issue3_nf_broadcast_due(
        &c->broadcast, now_ms, &nf_sequence, &nf_copy);
    (void)nf_sequence;
    (void)nf_copy;
    uint8_t qmi_ready = (uint8_t)((c->qmi.irq_pending || c->qmi_poll_due) &&
                                  (!c->qmi_delay_started ||
                                   c->now_us >= c->qmi_hold_until_us));
    uint32_t wake_deadline = ((now_ms / 100U) + 1U) * 100U;
    if (c->gzp_pending) {
        uint32_t deadline = c->gzp_started_ms + 20U;
        if (issue3_time_reached(now_ms, deadline))
            wake_deadline = now_ms;
        else if (issue3_time_reached(wake_deadline, deadline))
            wake_deadline = deadline;
    }
    Issue3RuntimeWork work = {
        c->sd.active,
        capture_sd_work_pending(c, now_ms),
        qmi_ready,
        c->icp_poll_due,
        (uint8_t)(c->gzp_pending &&
                  issue3_time_reached(now_ms, c->gzp_started_ms + 20U)),
        (uint8_t)(c->gzp_results.count != 0U),
        0U,
        (uint8_t)c->mts_ready,
        nf_ready,
        0U,
        1U,
        now_ms,
        wake_deadline,
        0U /* host model has no peripheral pending bits at this point */
    };
    work.frame_ready = (!c->queue.pending_valid && c->queue.count != 0U &&
                        issue3_frame_commit_due(
                            c->queue.start_ms[c->queue.head], now_ms));
    return work;
}

static void advance_idle(Capture *c);

/* Drive the same atomic sleep entrance as main.c.  The callbacks model
 * PRIMASK and WFI; sensor/FAT production work remains in the surrounding
 * foreground loop, while an actual sleep advances only to the next event. */
static void capture_sleep_disable(void *ctx)
{
    Capture *c = (Capture *)ctx;
    assert(!c->sleep_irq_masked);
    c->sleep_irq_masked = 1U;
}

static void capture_sleep_enable(void *ctx)
{
    Capture *c = (Capture *)ctx;
    assert(c->sleep_irq_masked);
    c->sleep_irq_masked = 0U;
}

static void capture_sleep_refresh(void *ctx, Issue3RuntimeWork *work)
{
    Capture *c = (Capture *)ctx;
    assert(c->sleep_irq_masked);
    *work = capture_runtime_work(c);
}

static void capture_sleep_arm(void *ctx, uint32_t deadline_ms)
{
    Capture *c = (Capture *)ctx;
    assert(c->sleep_irq_masked);
    c->sleep_deadline_ms = deadline_ms;
}

static void capture_sleep_wfi(void *ctx)
{
    Capture *c = (Capture *)ctx;
    assert(c->sleep_irq_masked);
    if (c->sd.active) c->wfi_while_sd++;
    c->wfi_count++;
    advance_idle(c);
}

static const Issue3RuntimeSleepOps capture_sleep_ops = {
    capture_sleep_disable,
    capture_sleep_enable,
    capture_sleep_refresh,
    capture_sleep_arm,
    0,
    capture_sleep_wfi
};

static void advance_idle(Capture *c)
{
    if (c->sd.active) return;
    uint64_t limit = (uint64_t)RUN_MS * 1000ULL;
    uint64_t next = limit;
    if (c->recording && c->qmi_next_us > c->now_us && c->qmi_next_us < next)
        next = c->qmi_next_us;
    if (c->recording && (uint64_t)c->next_timer_ms * 1000ULL > c->now_us &&
        (uint64_t)c->next_timer_ms * 1000ULL < next)
        next = (uint64_t)c->next_timer_ms * 1000ULL;
    if (c->recording && (uint64_t)c->icp_next_ms * 1000ULL > c->now_us &&
        (uint64_t)c->icp_next_ms * 1000ULL < next)
        next = (uint64_t)c->icp_next_ms * 1000ULL;
    if (c->recording && (uint64_t)c->mts_next_ms * 1000ULL > c->now_us &&
        (uint64_t)c->mts_next_ms * 1000ULL < next)
        next = (uint64_t)c->mts_next_ms * 1000ULL;
    if (c->recording && (uint64_t)c->gzp.next_ms * 1000ULL > c->now_us &&
        (uint64_t)c->gzp.next_ms * 1000ULL < next)
        next = (uint64_t)c->gzp.next_ms * 1000ULL;
    if (c->gzp_pending &&
        (uint64_t)(c->gzp_started_ms + 20U) * 1000ULL > c->now_us &&
        (uint64_t)(c->gzp_started_ms + 20U) * 1000ULL < next)
        next = (uint64_t)(c->gzp_started_ms + 20U) * 1000ULL;
    if (c->recording && !c->gzp_pending && !c->gzp_ready &&
        (uint64_t)c->gzp.next_ms * 1000ULL <= c->now_us)
        next = c->now_us;
    /* A completed conversion is retained until its frame slot is available.
     * Keep servicing that retained result at the next bounded turn instead
     * of sleeping until the next QMI sample and turning queue pressure into a
     * false GZP schedule miss. */
    if (c->gzp_ready) next = c->now_us;
    if (next <= c->now_us) next = c->now_us + 1000ULL;
    c->now_us = next;
}

static void run_capture(Capture *c)
{
    c->recording = 1U;
    c->qmi_next_us = 0U;
    c->qmi_next_id = 0U;
    c->icp_next_ms = 0U;
    c->mts_next_ms = 1800000U;
    /* Match firmware runtime origin: the first GZP grid point is 0 ms. */
    c->gzp.next_ms = 0U;
    c->now_us = 0U;
    c->next_timer_ms = TIMER_POLL_PERIOD_MS;
    issue3_heartbeat_schedule_init(&c->heartbeat, 1800000U);
    for (;;) {
        timer3_poll_update(c);
        produce_qmi(c);
        produce_icp(c);
        service_gzp(c);
        service_mts(c);
        if (c->qmi_poll_due || c->qmi.irq_pending ||
            (!c->recording && c->qmi.count)) {
            c->qmi_poll_due = 0U;
            c->qmi.irq_pending = 0U;
            service_qmi(c);
        }
        if (c->icp_poll_due || (!c->recording && c->icp_fifo_count)) {
            c->icp_poll_due = 0U;
            service_icp(c);
        }
        nf_service(c);
        service_sd(c);
        if (c->recording) {
            Issue3RuntimeWork work;
            if (!issue3_runtime_sleep_entry(&work, &capture_sleep_ops, c)) {
                c->foreground_turns++;
            }
        }
        if (c->recording && c->now_us < (uint64_t)RUN_MS * 1000ULL)
            continue;
        if (c->recording) c->recording_stop_us = c->now_us;
        c->recording = 0U;
        if (c->qmi.count || c->icp_fifo_count || c->gzp_pending ||
            c->gzp_ready || c->mts_ready || c->queue.count ||
            c->writer_pending || c->sd.active || c->metadata_pending) {
            if (!c->sd.active) c->now_us += 1000U;
            continue;
        }
        break;
    }
    flush_partial_sector(c);
    metadata_maybe_schedule(c);
    while (c->metadata_pending) {
        service_sd(c);
        if (!c->sd.active && c->metadata_pending == 1U) service_metadata(c);
    }
    release_prefetch_tail(c);
}

static void verify_file(Capture *c)
{
    uint32_t length = get32(&c->root[28]);
    assert(length == RUN_MS / 1000U * FRAME_SIZE);
    assert(length == c->logical_bytes);
    uint8_t *seen_qmi = (uint8_t *)calloc(c->qmi_input + 1U, 1U);
    uint8_t *seen_gzp = (uint8_t *)calloc(RUN_MS / ISSUE3_GZP_PERIOD_MS, 1U);
    uint8_t *seen_icp = (uint8_t *)calloc(c->icp_input + 1U, 1U);
    assert(seen_qmi && seen_gzp && seen_icp);
    uint32_t sector_count = (length + SECTOR - 1U) / SECTOR;
    uint32_t needed_clusters = (sector_count + SPC - 1U) / SPC;
    assert(c->fat_allocations >= c->fat_released);
    assert(c->fat_allocations - c->fat_released == needed_clusters);
    assert(c->fat_links == (needed_clusters > 0U ? needed_clusters - 1U : 0U));
    uint32_t *cluster_for_index = (uint32_t *)calloc(needed_clusters,
                                                       sizeof(*cluster_for_index));
    assert(cluster_for_index);
    uint8_t *seen_cluster = (uint8_t *)calloc(CLUSTERS, 1U);
    assert(seen_cluster);
    uint32_t cl = FIRST_CLUSTER;
    for (uint32_t i = 0; i < needed_clusters; i++) {
        assert(cl >= FIRST_CLUSTER && cl < FIRST_CLUSTER + CLUSTERS);
        assert(!seen_cluster[cl - FIRST_CLUSTER]);
        seen_cluster[cl - FIRST_CLUSTER] = 1U;
        cluster_for_index[i] = cl;
        uint32_t lba = fat_lba(cl), off = fat_off(cl);
        uint32_t next = fat_get(c, lba, (uint16_t)off);
        assert(fat_get(c, lba + FAT_SECTORS, (uint16_t)off) == next);
        if (i + 1U < needed_clusters) {
            assert(next >= FIRST_CLUSTER && next < FIRST_CLUSTER + CLUSTERS);
            cl = next;
        } else {
            assert(next >= 0x0FFFFFF8U);
        }
    }
    uint32_t frames = length / FRAME_SIZE;
    uint32_t offset = 0U;
    uint32_t previous_start = UINT32_MAX, previous_seq = UINT32_MAX;
    uint32_t previous_qmi_ts = 0U, previous_gzp_ts = 0U;
    uint8_t have_qmi_ts = 0U, have_gzp_ts = 0U;
    for (uint32_t n = 0; n < frames; n++) {
        uint8_t frame[FRAME_SIZE];
        for (uint32_t i = 0; i < FRAME_SIZE; i++) {
            uint32_t absolute = offset + i;
            uint32_t sec_no = absolute / SECTOR;
            uint32_t in = absolute % SECTOR;
            cl = cluster_for_index[sec_no / SPC];
            frame[i] = disk_sector(c, data_lba(cl, (uint16_t)(sec_no % SPC)))[in];
        }
        assert(get32(&frame[0]) == FRAME_MAGIC);
        assert(get16(&frame[4]) == FRAME_VERSION);
        assert(get16(&frame[6]) == FRAME_HEADER);
        assert(crc32(frame, FRAME_CRC_OFF) == get32(&frame[FRAME_CRC_OFF]));
        uint32_t seq = get32(&frame[8]), start = get32(&frame[12]);
        assert(seq == n && start == n * 1000U);
        if (n) {
            assert(seq == previous_seq + 1U);
            assert(start == previous_start + 1000U);
        }
        previous_seq = seq;
        previous_start = start;
        for (uint8_t i = 0; i < frame[16]; i++) {
            const uint8_t *p = &frame[FRAME_QMI_OFF + i * 16U];
            uint32_t id = (uint32_t)get16(p + 4) |
                          ((uint32_t)get16(p + 6) << 16);
            assert(id < c->qmi_input && !seen_qmi[id]);
            seen_qmi[id] = 1U;
            uint32_t sample_ts = get32(p);
            assert(sample_ts >= start && sample_ts < start + 1000U);
            if (have_qmi_ts) assert(sample_ts > previous_qmi_ts &&
                                    sample_ts - previous_qmi_ts <= 200U);
            previous_qmi_ts = sample_ts;
            have_qmi_ts = 1U;
        }
        for (uint8_t i = 0; i < frame[17]; i++) {
            const uint8_t *p = &frame[FRAME_GZP_OFF + i * 10U];
            uint32_t ts = get32(p);
            uint32_t point = ts / ISSUE3_GZP_PERIOD_MS;
            assert(ts >= start && ts < start + 1000U);
            assert(ts % ISSUE3_GZP_PERIOD_MS == 0U);
            if (have_gzp_ts) assert(ts > previous_gzp_ts &&
                                    ts - previous_gzp_ts == 200U);
            previous_gzp_ts = ts;
            have_gzp_ts = 1U;
            assert(point < RUN_MS / ISSUE3_GZP_PERIOD_MS &&
                   !seen_gzp[point]);
            seen_gzp[point] = 1U;
        }
        for (uint8_t i = 0; i < frame[18]; i++) {
            const uint8_t *p = &frame[FRAME_ICP_OFF + i * 10U];
            uint32_t id = (uint32_t)p[4] | ((uint32_t)p[5] << 8) |
                          ((uint32_t)p[6] << 16);
            assert(get32(p) % 500U == 0U);
            assert(id < c->icp_input && !seen_icp[id]);
            seen_icp[id] = 1U;
        }
        if (frame[19]) {
            uint32_t mts_ms = get32(&frame[FRAME_MTS_OFF]);
            assert(mts_ms % 1800000U == 0U);
        }
        offset += FRAME_SIZE;
    }
    uint8_t qmi_phase = 0U;
    uint32_t expected_qmi_saved = 0U;
    for (uint32_t i = 0; i < c->qmi_input; i++) {
        uint8_t keep = issue3_decimation_keep_next(&qmi_phase);
        if (keep) {
            expected_qmi_saved++;
            assert(seen_qmi[i]);
        } else {
            assert(!seen_qmi[i]);
        }
    }
    assert(expected_qmi_saved != 0U);
    uint8_t gzp_phase = 0U;
    uint32_t expected_gzp_saved = 0U;
    for (uint32_t i = 0; i < RUN_MS / ISSUE3_GZP_PERIOD_MS; i++) {
        uint8_t keep = issue3_decimation_keep_next(&gzp_phase);
        if (keep) {
            expected_gzp_saved++;
            assert(seen_gzp[i]);
        } else {
            assert(!seen_gzp[i]);
        }
    }
    assert(expected_gzp_saved != 0U);
    for (uint32_t i = 0; i < c->icp_input; i++) assert(seen_icp[i]);
    free(seen_qmi);
    free(seen_gzp);
    free(seen_icp);
    free(seen_cluster);
    free(cluster_for_index);
    assert(expected_qmi_saved == (c->qmi_input + 1U) / 2U);
    assert(c->qmi_lost == 0U);
    assert(c->gzp_persisted == expected_gzp_saved);
}

static void verify_heartbeats(const Capture *c)
{
    assert(c->nf_copy_count == c->nf_logical_count * ISSUE3_NF_BROADCAST_COPIES);
    assert(c->nf_logical_count == 8U);
    for (uint32_t i = 0; i < c->nf_logical_count; i++) {
        uint32_t base = i * ISSUE3_NF_BROADCAST_COPIES;
        assert(c->nf_seq[base] == i && c->nf_seq[base + 1U] == i &&
               c->nf_seq[base + 2U] == i);
        assert(c->nf_copy[base] == 0U && c->nf_copy[base + 1U] == 1U &&
               c->nf_copy[base + 2U] == 2U);
        uint32_t start = c->nf_uptime[base];
        uint32_t copy2 = c->nf_uptime[base + 1U];
        uint32_t copy3 = c->nf_uptime[base + 2U];
        uint32_t offset2 = issue3_nf_copy_offset_ms(c->nf_device_id, i, 1U);
        uint32_t offset3 = issue3_nf_copy_offset_ms(c->nf_device_id, i, 2U);
        assert(copy2 >= start + offset2 && copy2 - start < offset2 + 1000U);
        assert(copy3 >= start + offset3 && copy3 - start < offset3 + 1000U);
        if (i == 0U) assert(start < 1000U);
        else {
            uint32_t grid = i * 1800000U;
            assert(start >= grid && start < grid + 1000U);
        }
    }
}

static void init_capture(Capture *c)
{
    memset(c, 0, sizeof(*c));
    c->disk = (uint8_t *)calloc((size_t)DISK_SECTORS, SECTOR);
    assert(c->disk);
    c->nf_device_id = 0x504342U;
    issue3_frame_queue_init(&c->queue);
    issue3_spi_bus_init(&c->spi);
    issue3_gzp_result_queue_init(&c->gzp_results);
    c->setup_mode = 1U;
    prepare_fat_chain(c);
    init_root(c);
    c->setup_mode = 0U;
    /* Production starts the absolute GZP grid at runtime origin (0 ms). */
    c->gzp.next_ms = 0U;
}

static void destroy_capture(Capture *c)
{
    free(c->disk);
    c->disk = 0;
}

/* Regression for the production failure seen on the V4 hardware run: the
 * oldest retained sample in a QMI FIFO batch can map to the physical frame
 * slot currently owned by the SD writer.  The FIFO must still be read through
 * to the later sample that belongs to an already-open safe frame. */
static void test_qmi_samplewise_pending(void)
{
    Capture c;
    init_capture(&c);
    c.recording = 1U;
    c.now_us = 8000000ULL;
    c.writer_pending = 1U;
    c.writer_slot = 0U;
    c.queue.count = ISSUE3_FRAME_QUEUE_CAPACITY;
    c.queue.head = 0U;
    for (uint8_t i = 0U; i < ISSUE3_FRAME_QUEUE_CAPACITY; i++) {
        c.queue.start_ms[i] = (uint32_t)i * 1000U;
        frame_init(&c.frames[i], c.queue.start_ms[i], i);
    }
    issue3_frame_queue_claim_pending(&c.queue, 0U);
    c.qmi.count = 4U;
    c.qmi.head = 0U;
    c.qmi.ts_ms[0] = 0U;
    c.qmi.ts_ms[1] = 1000U;
    c.qmi.ts_ms[2] = 2000U;
    c.qmi.ts_ms[3] = 3000U;
    c.qmi.id[0] = 0U;
    c.qmi.id[1] = 1U;
    c.qmi.id[2] = 2U;
    c.qmi.id[3] = 3U;
    c.qmi_decimation_phase = 0U;
    service_qmi(&c);
    assert(c.qmi.count == 0U);
    assert(c.qmi_read == 4U);
    assert(c.qmi_persisted == 1U);
    assert(c.qmi_admission_drops == 1U);
    assert(c.qmi_drop_owner == 1U);
    assert(c.qmi_drop_time == 0U && c.qmi_drop_queue == 0U);
    assert(c.qmi_fifo_resets == 0U);
    assert(c.frames[2].qmi_count == 1U && c.frames[2].qmi_ts[0] == 2000U);
    destroy_capture(&c);
}

int main(void)
{
    assert(ISSUE3_SD_RUNTIME_SPI_HZ == 2000000UL);
    assert(ISSUE3_SD_SPI_BYTE_US == 4ULL);
    assert(ISSUE3_SD_STEP_MAX_US == 64ULL);
    test_qmi_samplewise_pending();
    Capture c;
    init_capture(&c);
    run_capture(&c);
    verify_file(&c);
    verify_heartbeats(&c);
    assert(c.qmi_lost == 0U && c.qmi_overflow == 0U);
    assert(c.qmi_read == c.qmi_input);
    assert(c.qmi_admission_drops == 0U && c.qmi_fifo_resets == 0U);
    assert(c.qmi_exti_enabled == 0U);
    assert(c.qmi_input ==
           (uint32_t)(((uint64_t)RUN_MS * 1000ULL /
                       ISSUE3_QMI_SAMPLE_PERIOD_US) + 1ULL));
    assert(c.qmi_service_calls > 0U &&
           c.qmi_service_calls <= RUN_MS / TIMER_POLL_PERIOD_MS + 2U);
    assert(c.qmi.max_count >= 120U && c.qmi.max_count < 128U);
    assert(c.gzp_plan_count == RUN_MS / ISSUE3_GZP_PERIOD_MS);
    assert(c.gzp_start_count == c.gzp_plan_count &&
           c.gzp_plan_count == RUN_MS / ISSUE3_GZP_PERIOD_MS &&
           c.gzp_persisted == (c.gzp_plan_count + 1U) / 2U &&
           c.gzp_missed == 0U);
    assert(c.icp_input == RUN_MS / 500U);
    assert(c.icp_persisted == c.icp_input && c.icp_lost == 0U);
    assert(c.icp_service_calls > 0U &&
           c.icp_service_calls <= RUN_MS / TIMER_POLL_PERIOD_MS + 2U);
    assert(c.qmi_i2c_elapsed_us != 0U && c.icp_i2c_elapsed_us != 0U);
    /* Runtime allocation and linking are required to cross the same
     * asynchronous SD path as data sectors.  The startup FAT preparation
     * contributes seven reads; a long run must show additional runtime reads
     * and matching FAT0/FAT1 writes for every durable allocation/link. */
    assert(c.runtime_fat_reads > FAT_SECTORS);
    assert(c.runtime_fat0_writes == c.runtime_fat1_writes);
    assert(c.runtime_fat0_writes ==
           (c.fat_allocations - 1U) + c.fat_links);
    /* Runtime starts MTS4 at the first 30-minute boundary; the boundary at
     * exactly RUN_MS belongs to the next interval and is not part of this
     * four-hour capture window. */
    assert(c.mts_input == (RUN_MS - 1U) / 1800000U &&
           c.mts_persisted == c.mts_input);
    assert(c.wfi_count > 0U);
    assert(c.wfi_while_sd == 0U);
    assert(c.foreground_turns > c.sd_steps);
    /* Hardware SPI is intentionally much faster than the former bit-banged
     * model, so a four-hour run may finish before the ring is full often
     * enough to admit an existing frame while another physical slot is
     * pending.  That admission rule has dedicated targeted regressions in
     * test_issue3_logic; do not turn its absence in a faster run into a
     * false failure. */
    assert(c.sd_total_elapsed_us != 0U);
    assert(((uint64_t)c.sd_write_transactions * SECTOR * 1000000ULL) /
               c.sd_total_elapsed_us > FRAME_SIZE);
    printf("异步4小时生产路径通过：QMI读取/落盘=%u/%u（原始输入=%u，1/2相位），FIFO峰值=%u（EXTI=%u，轮询服务=%u，I2C=%lluus），GZP原始计划/启动/落盘=%u/%u/%u（1/2相位） missed=%u，ICP输入/落盘=%u/%u（轮询服务=%u，I2C=%lluus），MTS=%u/%u，V4帧=%u（%u字节），FAT分配/链接/释放=%u/%u/%u，运行期FAT读/FAT0写/FAT1写=%u/%u/%u，SD事务=%u（读%u/写%u），单事务最大=%lluus，NF逻辑/副本=%u/%u，NF延后=%u，满队列pending下已有帧追加=%u\n",
           c.qmi_read, c.qmi_persisted, c.qmi_input,
           c.qmi.max_count,
           c.qmi_exti_enabled ? 1U : 0U, c.qmi_service_calls,
           (unsigned long long)c.qmi_i2c_elapsed_us,
           c.gzp_plan_count, c.gzp_start_count, c.gzp_persisted, c.gzp_missed,
           c.icp_input, c.icp_persisted, c.icp_service_calls,
           (unsigned long long)c.icp_i2c_elapsed_us,
           c.mts_input, c.mts_persisted,
           get32(&c.root[28]) / FRAME_SIZE, FRAME_SIZE,
           c.fat_allocations, c.fat_links,
           c.fat_released, c.runtime_fat_reads, c.runtime_fat0_writes,
           c.runtime_fat1_writes, c.sd_transactions,
           c.sd_read_transactions, c.sd_write_transactions,
           (unsigned long long)c.sd_max_transaction_us,
           c.nf_logical_count, c.nf_copy_count, c.nf_deferred,
           c.existing_pending_admissions);
    printf("睡眠判定：WFI=%llu 次（SD事务期间=%llu），前台轮=%llu，SD写入平均传输速率=%llu B/s（非真实等待）\n",
           (unsigned long long)c.wfi_count,
           (unsigned long long)c.wfi_while_sd,
           (unsigned long long)c.foreground_turns,
           (unsigned long long)(((uint64_t)c.sd_write_transactions * SECTOR *
                                 1000000ULL) / c.sd_total_elapsed_us));
    destroy_capture(&c);
    return 0;
}
