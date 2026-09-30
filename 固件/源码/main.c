#include <stdint.h>
#include "stm32g030.h"
#include "issue3_logic.h"

extern uint32_t _estack;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _etext;

void Reset_Handler(void);
void Default_Handler(void);
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void);
void EXTI4_15_IRQHandler(void);
void TIM3_IRQHandler(void);

__attribute__((section(".isr_vector")))
void (*const vector_table[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,
    SVC_Handler,
    0, 0,
    PendSV_Handler,
    SysTick_Handler,
    /* External IRQ 0..7: WWDG..EXTI4_15. */
    0, 0, 0, 0, 0, 0, 0, EXTI4_15_IRQHandler,
    /* External IRQ 8..14: DMA..TIM2. */
    0, 0, 0, 0, 0, 0, 0,
    /* External IRQ 15: TIM2.  IRQ16 is TIM3 on STM32G030. */
    0,
    TIM3_IRQHandler,
    /* Remaining STM32G0 IRQ slots. */
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t TIMINGR;
    volatile uint32_t TIMEOUTR;
    volatile uint32_t ISR;
    volatile uint32_t ICR;
    volatile uint32_t PECR;
    volatile uint32_t RXDR;
    volatile uint32_t TXDR;
} I2C_TypeDef;

#define I2C1   ((I2C_TypeDef *)I2C1_BASE)

#define RCC_APBENR1_I2C1EN  (1U << 21)
#define RCC_APBRSTR1_I2C1RST (1U << 21)
#define RCC_CCIPR_I2C1SEL_Msk (3U << 12)
#define RCC_CCIPR_I2C1SEL_HSI16 (2U << 12)

#define I2C_ISR_TXE   (1U << 0)
#define I2C_ISR_TXIS  (1U << 1)
#define I2C_ISR_RXNE  (1U << 2)
#define I2C_ISR_NACKF (1U << 4)
#define I2C_ISR_STOPF (1U << 5)
#define I2C_ISR_TC    (1U << 6)
#define I2C_CR1_PE    (1U << 0)
#define I2C_CR2_START (1U << 13)
#define I2C_CR2_STOP  (1U << 14)
#define I2C_CR2_RD_WRN (1U << 10)
#define I2C_ICR_NACKCF  (1U << 4)
#define I2C_ICR_STOPCF  (1U << 5)

#define I2C_SCL_PIN (1U << 9)
#define I2C_SDA_PIN (1U << 10)

#define SD_CS_PIN   (1U << 4)
#define SD_SCK_PIN  (1U << 5)
#define SD_MISO_PIN (1U << 6)
#define SD_MOSI_PIN (1U << 7)
#define SD_PWR_PIN  (1U << 3)

#define NF_CS_PIN   (1U << 15)
#define NF_CE_PIN   (1U << 5)
#define NF_IRQ_PIN  (1U << 4)

#define OLED_SDA_PIN (1U << 13)
#define OLED_SCL_PIN (1U << 14)
#define OLED_ADDR_WRITE 0x78U
#define OLED_COLUMN_OFFSET 2U

#define USART_CR1_UE    (1U << 0)
#define USART_CR1_TE    (1U << 3)
#define USART_CR1_RE    (1U << 2)
#define USART_ISR_TXE   (1U << 7)
#define USART_ISR_TC    (1U << 6)

static volatile uint32_t r_magic;
static volatile uint32_t r_i2c_scan0, r_i2c_scan1, r_i2c_scan2, r_i2c_scan3;
static volatile uint32_t r_i2c_last_isr, r_i2c_last_cr2;
static volatile uint8_t r_i2c_last_addr;
static volatile int16_t r_ax, r_ay, r_az, r_gx, r_gy, r_gz;
static volatile int16_t r_mts4_temp;
static volatile uint8_t r_qmi_ok, r_mts4_ok, r_icp_ok;
static volatile uint8_t r_sd_ok, r_sd_init_ok, r_sd_cmd0, r_sd_cmd8, r_sd_acmd41, r_sd_cmd58;
static volatile uint32_t r_sd_ocr;
static volatile uint8_t r_spi_timeout;
/* Keep only the first eight bytes of the read-sector diagnostic; the full
 * sector is consumed transiently and is not needed after probing. */
static volatile uint8_t r_sd_read0, r_sd_sector0[8];
static volatile uint16_t r_sd_mbr_sig;
static volatile uint8_t r_log_ok;
static volatile uint16_t r_log_bytes;
static volatile uint32_t r_log_cluster;
static volatile int32_t r_icp_p_raw, r_icp_t_raw;
static volatile uint8_t r_gzp_ok, r_gzp_stage, r_gzp_data[6];
static volatile uint8_t r_gzp_addr_used;
static volatile uint32_t r_gzp_p_raw;
static volatile int16_t r_gzp_t_raw;
/* r_nf_ok reports the last physical radio transmission completion only;
 * with ACK disabled it never means that a receiver heard the packet. */
static volatile uint8_t r_nf_ok, r_nf_init_ok, r_nf_stage, r_nf_tx_cnt;
static volatile uint16_t r_log_num;
static volatile uint32_t r_log_written;
static volatile uint32_t r_frame_seq;
static volatile uint32_t r_system_ms;
static volatile uint32_t r_runtime_ms;
static volatile uint8_t r_qmi_irq_pending;
/* TIM3 provides a periodic FIFO/status fallback when the PB6 edge is absent. */
static volatile uint8_t r_qmi_poll_due;
/* ICP has no wired interrupt; limit its FIFO service to the 100 ms cadence. */
static volatile uint8_t r_icp_poll_due;
static volatile uint8_t r_qmi_fifo_overflow;
static volatile uint16_t r_qmi_fifo_overflow_count;
static volatile uint16_t r_qmi_late_samples;
static volatile uint16_t r_gzp_missed_count;
/* ICP samples discarded because their timestamp was already outside the
 * retained frame window or still belonged to an SD-owned physical slot.  The
 * V4 layout has no separate ICP counter; the per-frame ICP_FAIL flag carries
 * the diagnostic while this bounded counter keeps the loss explicit for
 * runtime diagnostics and tests. */
static volatile uint16_t r_icp_late_samples;
static volatile uint8_t r_mts_due;
static volatile uint8_t r_nf_due;
static volatile uint8_t r_short_timer_done;
static volatile uint8_t r_sd_full;
static volatile uint8_t r_sd_alloc_pending;
static uint8_t sd_buf[512];
static uint8_t log_meta_buf[512];

/* ST-Link/GDB-readable, RAM-only production diagnostics.  The magic is set
 * when the formal runtime begins, after startup probing, so startup I/O cannot
 * be mistaken for the measured acquisition path. */
volatile Issue3RuntimeDiagnostics r_issue3_diag;

#define FRAME_MAGIC ISSUE3_FRAME_MAGIC_LOG2 /* LOG2 container; V4 current */
#define FRAME_MAGIC_V1 0x4C4F4731u
#define FRAME_VERSION ISSUE3_FRAME_VERSION
#define FRAME_HEADER_SIZE ISSUE3_FRAME_HEADER_SIZE
#define QMI_N ISSUE3_FRAME_QMI_CAPACITY
#define GZP_N ISSUE3_FRAME_GZP_CAPACITY
#define ICP_N ISSUE3_FRAME_ICP_CAPACITY
#define MTS4_N 1
#define QMI_SENSOR_PERIOD_US ((uint32_t)ISSUE3_QMI_SAMPLE_PERIOD_US)
#define QMI_FRAME_RETENTION_MS ((uint32_t)ISSUE3_FRAME_RETENTION_MS)
#define FRAME_QUEUE_N ISSUE3_FRAME_QUEUE_CAPACITY /* oldest retained for derived SD bound */
#define FAT_SCAN_SECTOR_BUDGET 1u /* one bounded background step */
#define GZP_CONVERSION_WAIT_MS 20u /* 0xB4 OSR_P=8x, nominal 19 ms */
#define GZP_CONVERSION_TIMEOUT_MS 100u
#define GZP_RESULT_RETENTION_MS ((uint32_t)ISSUE3_FRAME_RETENTION_MS)
#define QMI_HEALTH_TIMEOUT_MS 500u
#define GZP_HEALTH_TIMEOUT_MS 500u
#define ICP_HEALTH_TIMEOUT_MS 1500u
#define MTS4_HEALTH_TIMEOUT_MS (MTS4_PERIOD_MS + 5000u)
#define MTS4_PERIOD_MS 1800000u
#define NF_HEARTBEAT_PERIOD_MS 1800000u
#define LOG_PROGRESS_TIMEOUT_MS (ISSUE3_FRAME_RETENTION_MS + 5000u)
#define NF_HEARTBEAT_PROTOCOL_VERSION 3U
#define NF_HEARTBEAT_MESSAGE_TYPE 1U
#define NF_STATUS_SD_INIT_OK (1U << 0)
#define NF_STATUS_LOG_ACTIVE (1U << 1)
#define NF_STATUS_SD_FULL (1U << 2)
#define FRAME_QMI_OFF ISSUE3_FRAME_QMI_OFFSET
#define FRAME_GZP_OFF ISSUE3_FRAME_GZP_OFFSET
#define FRAME_ICP_OFF ISSUE3_FRAME_ICP_OFFSET
#define FRAME_MTS_OFF ISSUE3_FRAME_MTS_OFFSET
#define FRAME_DATA_SIZE ISSUE3_FRAME_DATA_SIZE
#define FRAME_CRC_OFF ISSUE3_FRAME_CRC_OFFSET
#define FRAME_SIZE ISSUE3_FRAME_SIZE
#define FRAME_STATUS_QMI_FAIL (1u << 0)
#define FRAME_STATUS_QMI_FIFO_OVERFLOW (1u << 1)
#define FRAME_STATUS_GZP_FAIL (1u << 2)
#define FRAME_STATUS_ICP_FAIL (1u << 3)
#define FRAME_STATUS_MTS_FAIL (1u << 4)
#define FRAME_STATUS_SD_FULL (1u << 5)
#define FRAME_STATUS_GZP_SCHEDULE_OVERFLOW (1u << 6)
#define FRAME_STATUS_QMI_FIFO_CLEANUP (1u << 7)
#define FRAME_STATUS_QMI_REANCHORED (1u << 8)
#define FRAME_STATUS_QMI_LATE_SAMPLE (1u << 9)
/* V4 status bits 10..12 were unused.  They make a QMI stalled-stream
 * watchdog/reconfiguration visible in the persisted frame without changing
 * the frame layout or the meaning of the existing counters.  Bit 10 retains
 * the historical qmi_empty_recovery name for parser compatibility. */
#define FRAME_STATUS_QMI_EMPTY_RECOVERY ISSUE3_QMI_STATUS_EMPTY_RECOVERY
#define FRAME_STATUS_QMI_RECOVERY_OK ISSUE3_QMI_STATUS_RECOVERY_OK
#define FRAME_STATUS_QMI_RECOVERY_FAIL ISSUE3_QMI_STATUS_RECOVERY_FAIL
#define GZP_MEASURE_CMD 0xB4u
#define STATUS_OK 1U
#define STATUS_FAIL 0xFFU
#define STARTUP_DISPLAY_HOLD_MS 10000U

typedef struct {
    uint8_t data[FRAME_SIZE];
    uint32_t start_ms;
    uint8_t qmi_count, gzp_count, icp_count, mts_valid;
    uint16_t qmi_dropped;
    uint16_t gzp_missed;
    uint32_t status;
    uint8_t valid;
} FrameSlot;

static FrameSlot frame_slots[FRAME_QUEUE_N];
static Issue3FrameQueue frame_queue;
static volatile uint32_t frame_status;
static Issue3QmiClock qmi_clock;
/* Runtime state for the valid-zero FIFO watchdog.  This is intentionally a
 * named RAM object so Tiger can inspect the last raw sample and recovery
 * attempt while the device is running. */
static Issue3QmiRecovery qmi_recovery;
/* Read-side decimation phases persist across FIFO batches, frame boundaries,
 * re-anchors and scheduler turns.  Raw sensor reads and clock advancement are
 * never skipped; only the successful sample admitted to the smaller V4 frame
 * is selected by these phases. */
static uint8_t qmi_decimation_phase;
static uint8_t gzp_decimation_phase;
static uint8_t gzp_pending;
static uint8_t gzp_result_ready;
static Issue3GzpResultQueue gzp_results;
static uint32_t gzp_started_ms;
static uint32_t gzp_plan_ms;
static uint8_t mts_pending;
static uint8_t mts_result_ready;
static uint32_t mts_started_ms;
static uint32_t mts_result_ms;
static int16_t mts_result_temp;
static uint32_t next_mts_ms;
static Issue3HeartbeatSchedule nf_heartbeat_schedule;
static Issue3NfBroadcast nf_broadcast;
static uint32_t next_gzp_ms;
static uint8_t log_active;
static uint32_t log_clu, log_lba, total_written;
static uint16_t log_sec_off, log_num;
static uint16_t log_meta_frame_count;
static uint32_t g_data_clusters;
static Issue3FatScan g_fat_scan;
static int g_fat_last_result;
static uint32_t log_next_clu;
static uint8_t log_next_ready;
static Issue3LogPrefetch log_prefetch;
/* The finalized queue slot remains owned by the log writer until its bytes
 * are persisted.  Keeping an index avoids a second full V4 RAM frame. */
static uint8_t log_pending_slot;
static uint16_t log_pending_off;
static uint8_t log_pending_active;
static uint8_t log_metadata_pending;
static uint32_t log_metadata_size;
static uint32_t log_complete_size;
static uint8_t log_link_pending;
static uint8_t log_link_stage;
static uint32_t log_link_cluster, log_link_next, log_link_sector;
static Issue3SectorBuffer log_sector_buffer;

/* Runtime SD transactions are deliberately separate from the synchronous
 * startup/probe helpers above.  CS stays asserted for the complete SD
 * command, while each call to sd_async_step() performs at most one response,
 * token or busy poll, or ISSUE3_SD_STEP_BYTES data bytes.  This preserves the
 * card protocol without allowing a 512-byte hardware-SPI transfer to monopolize
 * the foreground sensor scheduler. */
typedef Issue3SdTransfer SdAsyncTxn;

static SdAsyncTxn sd_async;
/* NF-03 and SD share SCK/MOSI/MISO.  The owner remains held for the whole
 * SD transaction (including token/data/busy phases), or for the complete NF
 * register/send/wake/sleep sequence. */
static Issue3SpiBus shared_spi_bus;
static uint8_t log_sector_write_pending;
static uint8_t log_metadata_stage;
static uint8_t fat_runtime_phase;
static uint32_t fat_runtime_cluster;
static uint32_t fat_runtime_sector;
static uint8_t log_alloc_io;
static uint32_t log_last_progress_ms;
static uint8_t log_write_stalled;

/* Shared with host diagnostics tests; this replaces the former collection of
 * private phase/transaction variables without changing the fixed diagnostic
 * ABI or adding another persistent RAM block. */
static Issue3DiagRuntimeState diag_runtime;

/* A successful register read during startup is not evidence that the
 * sensor kept producing data during the long run.  These timestamps are
 * refreshed only by successful runtime samples and are folded into the
 * heartbeat status immediately before each physical broadcast. */
static uint8_t qmi_sample_seen, gzp_sample_seen, icp_sample_seen,
               mts4_sample_seen;
static uint32_t qmi_last_sample_ms, gzp_last_sample_ms, icp_last_sample_ms,
                mts4_last_sample_ms;

static uint32_t log_persisted_complete_size(void)
{
    uint32_t physical_complete = (total_written / FRAME_SIZE) * FRAME_SIZE;
    return log_complete_size < physical_complete ?
           log_complete_size : physical_complete;
}

static FrameSlot *frame_slot_for(uint32_t timestamp_ms,
                                 Issue3FrameOpenReason *open_reason);
static uint8_t frame_queue_commit_ready(uint32_t now_ms);
static uint8_t frame_add_qmi(uint32_t ts_ms, int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz);
static uint8_t frame_add_gzp(uint32_t ts_ms, uint32_t pressure_raw,
                             int16_t temperature_raw);
static uint8_t frame_add_icp(uint32_t ts_ms, int32_t p_raw, int32_t t_raw);
static uint8_t frame_add_mts(uint32_t ts_ms, int16_t temp_raw);

#define SYST_CSR (*(volatile uint32_t *)0xE000E010U)
#define SYST_RVR (*(volatile uint32_t *)0xE000E014U)
#define SYST_CVR (*(volatile uint32_t *)0xE000E018U)
#define SYST_CSR_ENABLE   (1U << 0)
#define SYST_CSR_TICKINT  (1U << 1)
#define SYST_CSR_CLKSOURCE (1U << 2)
#define NVIC_ISER (*(volatile uint32_t *)0xE000E100U)
#define NVIC_EXTI4_15_IRQ STM32G030_EXTI4_15_IRQ_BIT
#define NVIC_TIM3_IRQ (1U << 16)
#define EXTI_QMI_PIN STM32G030_QMI_EXTI_PIN
#define EXTI_QMI_EXTICR_INDEX STM32G030_QMI_EXTICR_INDEX
#define EXTI_QMI_EXTICR_SHIFT STM32G030_QMI_EXTICR_SHIFT

#ifndef PCB32_DEVICE_ID0
/* Development default only; deployment builds must override all three. */
#define PCB32_DEVICE_ID0 0x50U
#define PCB32_DEVICE_ID1 0x43U
#define PCB32_DEVICE_ID2 0x42U
#endif

static void systick_init(void)
{
    /* HSI16 is the reset clock; this tick is used only during startup check. */
    SYST_RVR = 16000U - 1U;
    SYST_CVR = 0;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
}

static void systick_stop(void)
{
    SYST_CSR = 0;
}

void SysTick_Handler(void)
{
    r_system_ms++;
}

static inline void cpu_sleep(void)
{
    /* DSB guarantees all timer/EXTI programming is visible before the
     * architectural sleep instruction.  The runtime entry masks interrupts
     * around its final check; short startup waits also benefit from the same
     * ordering. */
    __asm volatile("dsb" ::: "memory");
    __asm volatile("wfi" ::: "memory");
}

static uint32_t runtime_now_ms(void)
{
    uint32_t base_a = r_runtime_ms;
    uint32_t counter = TIM3->CNT;
    uint32_t base_b = r_runtime_ms;
    if (base_a != base_b) {
        base_a = base_b;
        counter = TIM3->CNT;
    }
    return base_a + counter;
}

static int time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static uint8_t issue3_diag_active(void)
{
    return (uint8_t)(r_issue3_diag.magic == ISSUE3_RUNTIME_DIAG_MAGIC &&
                     r_issue3_diag.version == ISSUE3_RUNTIME_DIAG_VERSION);
}

static void issue3_diag_runtime_init(void)
{
    /* .bss has already been cleared by Reset_Handler.  Only the identity is
     * written here; all counters therefore start at zero without a memset or
     * a libc dependency in the freestanding image. */
    r_issue3_diag.magic = ISSUE3_RUNTIME_DIAG_MAGIC;
    r_issue3_diag.version = ISSUE3_RUNTIME_DIAG_VERSION;
    diag_runtime.sd_phase = 0xffU;
}

static void issue3_diag_observe_depths(void)
{
    if (!issue3_diag_active()) return;
    if (frame_queue.count > r_issue3_diag.frame_queue_max_depth)
        r_issue3_diag.frame_queue_max_depth = frame_queue.count;
    if (gzp_results.count > r_issue3_diag.gzp_queue_max_depth)
        r_issue3_diag.gzp_queue_max_depth = gzp_results.count;
}

static void issue3_diag_note_drop(uint8_t sensor,
                                  Issue3FrameOpenReason reason)
{
    if (!issue3_diag_active()) return;
    issue3_diag_note_frame_drop(&r_issue3_diag, sensor, reason);
}

static void issue3_diag_note_old_icp(uint32_t first_ms, uint32_t count)
{
    if (!issue3_diag_active()) return;
    for (uint32_t i = 0U; i < count; i++)
        issue3_diag_note_drop(
            ISSUE3_DIAG_SENSOR_ICP,
            issue3_frame_queue_timestamp_reason(
                &frame_queue, first_ms + i * 500U));
}

static void issue3_diag_sd_transaction_begin(uint8_t op, uint32_t now_ms)
{
    if (!issue3_diag_active()) return;
    issue3_diag_runtime_sd_begin(&r_issue3_diag, &diag_runtime, op, now_ms);
}

static void issue3_diag_sd_transaction_finish(uint32_t now_ms)
{
    if (!issue3_diag_active()) return;
    issue3_diag_runtime_sd_finish(&r_issue3_diag, &diag_runtime, now_ms);
}

static void issue3_diag_operation_begin(volatile Issue3DiagStage *stage,
                                        uint8_t *active, uint32_t *start_ms)
{
    if (!issue3_diag_active()) return;
    issue3_diag_runtime_operation_begin(&r_issue3_diag, stage, active,
                                        start_ms, runtime_now_ms());
}

static void issue3_diag_operation_finish(volatile Issue3DiagStage *stage,
                                         uint8_t *active, uint32_t start_ms,
                                         uint32_t now_ms)
{
    if (!issue3_diag_active()) return;
    issue3_diag_runtime_operation_finish(&r_issue3_diag, stage, active,
                                         start_ms, now_ms);
}

static void issue3_diag_sector_complete(uint32_t now_ms)
{
    if (!issue3_diag_active()) return;
    issue3_diag_runtime_sector_complete(
        &r_issue3_diag, &diag_runtime, now_ms, total_written,
        total_written / FRAME_SIZE);
}

static void gzp_note_missed(uint32_t count)
{
    if ((uint32_t)r_gzp_missed_count + count > 0xFFFFU) {
        r_gzp_missed_count = 0xFFFFU;
        frame_status |= FRAME_STATUS_GZP_SCHEDULE_OVERFLOW;
    } else {
        r_gzp_missed_count = (uint16_t)(r_gzp_missed_count + count);
    }
}

typedef struct {
    uint16_t milliseconds;
    uint32_t compare;
    uint8_t armed;
} ShortWaitContext;

static void runtime_irq_disable(void *ctx);
static void runtime_irq_enable(void *ctx);

static uint8_t short_wait_ready(void *ctx)
{
    (void)ctx;
    return r_short_timer_done;
}

static uint8_t short_wait_pending(void *ctx)
{
    (void)ctx;
    uint32_t status = TIM3->SR;
    if (status & TIM_SR_CC1IF) {
        /* The start callback cleared the previous interval's flag.  A CC1IF
         * seen here is therefore a new compare event (possibly arriving in
         * the final check/WFI window); make it serviceable without clearing
         * the pending source. */
        TIM3->DIER |= TIM_DIER_CC1IE;
        return 1U;
    }
    return (status & TIM_SR_UIF) != 0U;
}

static void short_wait_start(void *ctx)
{
    ShortWaitContext *wait = (ShortWaitContext *)ctx;
    uint32_t duration = wait->milliseconds;

    /* This callback runs with PRIMASK set.  Disable the old compare source,
     * remove only the stale CC1 flag, and clear the completion state before
     * taking the CNT sample for this interval. */
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    TIM3->SR &= ~TIM_SR_CC1IF;
    r_short_timer_done = 0U;
    wait->armed = 0U;

    /* CNT can advance while the MMIO writes above are in flight.  If it has
     * already crossed the requested interval, discard that candidate and
     * establish a fresh target from the newer sample.  The second CNT check
     * closes the same boundary immediately before CC1IE is enabled. */
    for (;;) {
        uint32_t start = TIM3->CNT % ISSUE3_SHORT_TIMER_PERIOD_MS;
        uint32_t target = (start + duration) % ISSUE3_SHORT_TIMER_PERIOD_MS;
        TIM3->CCR1 = target;
        uint32_t after_program = TIM3->CNT % ISSUE3_SHORT_TIMER_PERIOD_MS;
        if (!issue3_short_timer_target(start, after_program,
                                       (uint16_t)duration, &target)) {
            TIM3->SR &= ~TIM_SR_CC1IF;
            continue;
        }

        /* Any flag raised before the new compare target was reached belongs
         * to the previous CCR1 value.  It is safe to clear it only while the
         * target is still in the future. */
        TIM3->SR &= ~TIM_SR_CC1IF;
        uint32_t confirmed = TIM3->CNT % ISSUE3_SHORT_TIMER_PERIOD_MS;
        if (!issue3_short_timer_target(start, confirmed,
                                       (uint16_t)duration, &target)) {
            TIM3->SR &= ~TIM_SR_CC1IF;
            continue;
        }

        wait->compare = target;
        wait->armed = 1U;
        TIM3->DIER |= TIM_DIER_CC1IE;
        return;
    }
}

static void short_wait_arm(void *ctx)
{
    ShortWaitContext *wait = (ShortWaitContext *)ctx;
    /* The interval was already established atomically by short_wait_start.
     * A spurious update wake must not restart it and extend the delay. */
    if (!wait->armed) {
        wait->compare = (TIM3->CNT + wait->milliseconds) %
                        ISSUE3_SHORT_TIMER_PERIOD_MS;
        TIM3->CCR1 = wait->compare;
        wait->armed = 1U;
    }
    TIM3->DIER |= TIM_DIER_CC1IE;
}

static void short_wait_wfi(void *ctx)
{
    (void)ctx;
    cpu_sleep();
}

static const Issue3AtomicWaitOps short_wait_ops = {
    runtime_irq_disable,
    runtime_irq_enable,
    short_wait_ready,
    short_wait_pending,
    short_wait_arm,
    0,
    short_wait_wfi,
    short_wait_start
};

static void short_sleep_ms(uint16_t milliseconds)
{
    if (milliseconds == 0) return;
    if (milliseconds > 20) milliseconds = 20;
    ShortWaitContext wait = {milliseconds, 0U, 0U};
    /* Start and arm this interval under the same masked region.  An old CC1IF
     * is cleared before the new CCR1 is selected; the atomic wait loop then
     * handles only the new target or a genuinely new pending event. */
    if (!issue3_atomic_wait_begin(&short_wait_ops, &wait)) return;
    for (;;) {
        if (!issue3_atomic_wait_step(&short_wait_ops, &wait)) return;
    }
}

void TIM3_IRQHandler(void)
{
    uint32_t sr = TIM3->SR;
    if (sr & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF;
        r_runtime_ms += 100U;
        r_qmi_poll_due = 1U;
        r_icp_poll_due = 1U;
        if (time_reached(r_runtime_ms, next_mts_ms)) r_mts_due = 1;
        if (time_reached(r_runtime_ms, nf_heartbeat_schedule.next_ms)) r_nf_due = 1;
    }
    if (sr & TIM_SR_CC1IF) {
        TIM3->SR &= ~TIM_SR_CC1IF;
        TIM3->DIER &= ~TIM_DIER_CC1IE;
        r_short_timer_done = 1;
    }
}

void EXTI4_15_IRQHandler(void)
{
    if (EXTI->RPR1 & EXTI_QMI_PIN) {
        EXTI->RPR1 = EXTI_QMI_PIN;
        r_qmi_irq_pending = 1;
    }
}

static void runtime_timer_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    TIM3->CR1 = 0;
    TIM3->PSC = 15999U; /* HSI16 / 16000 = 1 kHz */
    TIM3->ARR = 99U;     /* 100 ms update, not a high-rate SysTick */
    TIM3->CNT = 0;
    TIM3->CCR1 = 0;
    TIM3->SR = 0;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    next_mts_ms = MTS4_PERIOD_MS;
    issue3_heartbeat_schedule_init(&nf_heartbeat_schedule,
                                   NF_HEARTBEAT_PERIOD_MS);
    next_gzp_ms = 0;
    NVIC_ISER |= NVIC_TIM3_IRQ;
    TIM3->CR1 = TIM_CR1_CEN;
}

/* TIM3 update is the coarse (100 ms) wake source.  Conversion completion and
 * MTS4 completion are earlier deadlines, so arm the compare channel for the
 * nearest one when the foreground has no work.  The compare is deliberately
 * used only for a sub-update interval; the normal update interrupt remains
 * the reliable wake source for all later deadlines. */
static void runtime_arm_wake_deadline(uint32_t now_ms,
                                      uint32_t deadline_ms)
{
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    if (time_reached(now_ms, deadline_ms)) return;
    uint32_t delta = deadline_ms - now_ms;
    if (delta >= 100U) return;
    uint32_t compare = (TIM3->CNT + (delta == 0U ? 1U : delta)) % 100U;
    TIM3->CCR1 = compare;
    /* Do not clear CC1IF here.  This function runs from the atomic sleep
     * entrance with PRIMASK set; an event can become pending between the
     * final refresh and this programming step.  Clearing the flag with a
     * read-modify-write would erase that wakeup and allow WFI to oversleep.
     * TIM3_IRQHandler clears stale/handled compare flags instead. */
    TIM3->DIER |= TIM_DIER_CC1IE;
}

static uint32_t runtime_next_wake_deadline(uint32_t now_ms)
{
    uint32_t next = ((now_ms / 100U) + 1U) * 100U;
    if (gzp_pending) {
        uint32_t deadline = gzp_started_ms + GZP_CONVERSION_WAIT_MS;
        if (time_reached(now_ms, deadline)) return now_ms;
        if (time_reached(next, deadline)) next = deadline;
    }
    if (mts_pending) {
        uint32_t deadline = mts_started_ms + 50U;
        if (time_reached(now_ms, deadline)) return now_ms;
        if (time_reached(next, deadline)) next = deadline;
    }
    return next;
}

static void qmi_irq_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
    GPIOB->MODER &= ~(3U << 12); /* PB6 / IMU_INT1 input */
    GPIOB->PUPDR &= ~(3U << 12);

    /* STM32G030 routes GPIOs through EXTI_EXTICR, not SYSCFG_EXTICR.
     * EXTI6 is field 16..18 of EXTICR2 (array index 1); value 1 selects
     * GPIOB.  The explicit masks avoid touching adjacent line mappings. */
    EXTI->EXTICR[EXTI_QMI_EXTICR_INDEX] &=
        ~(0x7U << EXTI_QMI_EXTICR_SHIFT);
    EXTI->EXTICR[EXTI_QMI_EXTICR_INDEX] |=
        (STM32G030_GPIOB_EXTI_SELECT << EXTI_QMI_EXTICR_SHIFT);
    EXTI->RPR1 = EXTI_QMI_PIN;
    EXTI->FPR1 = EXTI_QMI_PIN;
    EXTI->RTSR1 |= EXTI_QMI_PIN;
    EXTI->FTSR1 &= ~EXTI_QMI_PIN;
    EXTI->IMR1 |= EXTI_QMI_PIN;
    NVIC_ISER |= NVIC_EXTI4_15_IRQ;
}

static void delay_ms(volatile uint32_t ms)
{
    for (volatile uint32_t i = 0; i < ms * 800; i++);
}

static void i2c_record_error(uint8_t dev_addr)
{
    r_i2c_last_addr = dev_addr;
    r_i2c_last_isr = I2C1->ISR;
    r_i2c_last_cr2 = I2C1->CR2;
}

static void bb_i2c_delay(void)
{
    for (volatile uint32_t i = 0; i < 80; i++);
}

static void bb_scl(int high)
{
    if (high) GPIOA->BSRR = I2C_SCL_PIN;
    else GPIOA->BRR = I2C_SCL_PIN;
    bb_i2c_delay();
}

static void bb_sda(int high)
{
    if (high) GPIOA->BSRR = I2C_SDA_PIN;
    else GPIOA->BRR = I2C_SDA_PIN;
    bb_i2c_delay();
}

static void bb_i2c_start(void)
{
    bb_sda(1);
    bb_scl(1);
    bb_sda(0);
    bb_scl(0);
}

static void bb_i2c_stop(void)
{
    bb_sda(0);
    bb_scl(1);
    bb_sda(1);
}

static void bb_i2c_recover(void)
{
    bb_sda(1);
    for (uint8_t i = 0; i < 9; i++) {
        bb_scl(1);
        bb_scl(0);
    }
    bb_i2c_stop();
}

static int bb_i2c_write_byte(uint8_t byte)
{
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        bb_sda(byte & mask);
        bb_scl(1);
        bb_scl(0);
    }

    bb_sda(1);
    bb_scl(1);
    int ack = !(GPIOA->IDR & I2C_SDA_PIN);
    bb_scl(0);
    return ack;
}

static uint8_t bb_i2c_read_byte(int ack)
{
    uint8_t byte = 0;
    bb_sda(1);
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        bb_scl(1);
        if (GPIOA->IDR & I2C_SDA_PIN) byte |= 1;
        bb_scl(0);
    }

    bb_sda(!ack);
    bb_scl(1);
    bb_scl(0);
    bb_sda(1);
    return byte;
}

static void bb_i2c_prepare(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    GPIOA->MODER &= ~((3U << 18) | (3U << 20));
    GPIOA->MODER |= (1U << 18) | (1U << 20);
    GPIOA->OTYPER |= I2C_SCL_PIN | I2C_SDA_PIN;
    GPIOA->PUPDR &= ~((3U << 18) | (3U << 20));
    GPIOA->PUPDR |= (1U << 18) | (1U << 20);
    bb_i2c_stop();
}

static void bb_i2c_scan_bus(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    GPIOA->MODER &= ~((3U << 18) | (3U << 20));
    GPIOA->MODER |= (1U << 18) | (1U << 20);
    GPIOA->OTYPER |= I2C_SCL_PIN | I2C_SDA_PIN;
    GPIOA->PUPDR &= ~((3U << 18) | (3U << 20));
    GPIOA->PUPDR |= (1U << 18) | (1U << 20);

    r_i2c_scan0 = r_i2c_scan1 = r_i2c_scan2 = r_i2c_scan3 = 0;
    bb_i2c_recover();
    for (uint8_t addr = 0x08; addr <= 0x78; addr++) {
        bb_i2c_start();
        if (bb_i2c_write_byte(addr << 1)) {
            if (addr < 32) r_i2c_scan0 |= 1UL << addr;
            else if (addr < 64) r_i2c_scan1 |= 1UL << (addr - 32);
            else if (addr < 96) r_i2c_scan2 |= 1UL << (addr - 64);
            else r_i2c_scan3 |= 1UL << (addr - 96);
        }
        bb_i2c_stop();
        delay_ms(1);
    }
}

static int i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t len)
{
    bb_i2c_start();
    if (!bb_i2c_write_byte(dev_addr << 1)) {
        bb_i2c_stop();
        i2c_record_error(dev_addr);
        return -1;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (!bb_i2c_write_byte(data[i])) {
            bb_i2c_stop();
            i2c_record_error(dev_addr);
            return -1;
        }
    }
    bb_i2c_stop();
    return 0;
}

static int i2c_read(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    bb_i2c_start();
    if (!bb_i2c_write_byte(dev_addr << 1) || !bb_i2c_write_byte(reg)) {
        bb_i2c_stop();
        i2c_record_error(dev_addr);
        return -1;
    }

    bb_i2c_start();
    if (!bb_i2c_write_byte((dev_addr << 1) | 1U)) {
        bb_i2c_stop();
        i2c_record_error(dev_addr);
        return -1;
    }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = bb_i2c_read_byte(i + 1 < len);
    }
    bb_i2c_stop();
    return 0;
}

static int i2c_read_raw(uint8_t dev_addr, uint8_t *buf, uint8_t len)
{
    bb_i2c_start();
    if (!bb_i2c_write_byte((dev_addr << 1) | 1U)) {
        bb_i2c_stop();
        i2c_record_error(dev_addr);
        return -1;
    }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = bb_i2c_read_byte(i + 1 < len);
    }
    bb_i2c_stop();
    return 0;
}

static int16_t to_signed16(uint8_t hi, uint8_t lo)
{
    int16_t v = (int16_t)((uint16_t)hi << 8 | lo);
    return v;
}

static int qmi8658_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_write(0x6A, data, 2);
}

#define QMI_CTRL9_CMD 0x0AU
#define QMI_STATUSINT 0x2DU
#define QMI_CMD_DONE 0x80U
#define QMI_CMD_ACK 0x00U
#define QMI_CMD_RST_FIFO ISSUE3_QMI_CMD_RST_FIFO
#define QMI_CMD_REQ_FIFO 0x05U
#define QMI_CTRL9_TIMEOUT_POLLS 16U

static int qmi8658_ctrl9(uint8_t command)
{
    uint8_t status = 0;
    if (qmi8658_write_reg(QMI_CTRL9_CMD, command) < 0) return -1;
    for (uint8_t i = 0; i < QMI_CTRL9_TIMEOUT_POLLS; i++) {
        if (i2c_read(0x6A, QMI_STATUSINT, &status, 1) < 0) return -1;
        if (status & QMI_CMD_DONE) break;
        if (i + 1U == QMI_CTRL9_TIMEOUT_POLLS) return -2;
    }
    if (!(status & QMI_CMD_DONE)) return -2;
    if (qmi8658_write_reg(QMI_CTRL9_CMD, QMI_CMD_ACK) < 0) return -3;
    for (uint8_t i = 0; i < QMI_CTRL9_TIMEOUT_POLLS; i++) {
        if (i2c_read(0x6A, QMI_STATUSINT, &status, 1) < 0) return -4;
        if (!(status & QMI_CMD_DONE)) return 0;
    }
    return -5;
}

/* Formal QMI8658A stream configuration.  Keep the register/value list in one
 * place so startup and a watchdog recovery cannot drift apart.  The values
 * are the existing 28.025 Hz, six-axis, four-sample-watermark configuration;
 * recovery only reapplies it and verifies the readable bits. */
#define QMI_REG_CTRL1 ISSUE3_QMI_REG_CTRL1
#define QMI_REG_CTRL2 ISSUE3_QMI_REG_CTRL2
#define QMI_REG_CTRL3 ISSUE3_QMI_REG_CTRL3
#define QMI_REG_CTRL6 ISSUE3_QMI_REG_CTRL6
#define QMI_REG_CTRL7 ISSUE3_QMI_REG_CTRL7
#define QMI_REG_CTRL8 ISSUE3_QMI_REG_CTRL8
#define QMI_REG_FIFO_WTM ISSUE3_QMI_REG_FIFO_WTM
#define QMI_REG_FIFO_CTRL ISSUE3_QMI_REG_FIFO_CTRL
#define QMI_CTRL1_CONFIG ISSUE3_QMI_CTRL1_CONFIG
#define QMI_CTRL2_CONFIG ISSUE3_QMI_CTRL2_CONFIG
#define QMI_CTRL3_CONFIG ISSUE3_QMI_CTRL3_CONFIG
#define QMI_CTRL6_CONFIG ISSUE3_QMI_CTRL6_CONFIG
#define QMI_CTRL7_CONFIG ISSUE3_QMI_CTRL7_CONFIG
#define QMI_CTRL8_CONFIG ISSUE3_QMI_CTRL8_CONFIG
#define QMI_FIFO_WTM_CONFIG ISSUE3_QMI_FIFO_WTM_CONFIG
#define QMI_FIFO_CTRL_CONFIG ISSUE3_QMI_FIFO_CTRL_CONFIG

static int qmi8658_write_verify(uint8_t reg, uint8_t value, uint8_t mask)
{
    uint8_t actual = 0U;
    if (qmi8658_write_reg(reg, value) < 0) return -1;
    if (i2c_read(0x6A, reg, &actual, 1) < 0) return -1;
    return ((actual & mask) == (value & mask)) ? 0 : -1;
}

static int qmi8658_apply_stream_config(void)
{
    if (qmi8658_write_verify(QMI_REG_CTRL1, QMI_CTRL1_CONFIG, 0xFFU) < 0)
        return -1;
    if (qmi8658_write_verify(QMI_REG_CTRL2, QMI_CTRL2_CONFIG, 0xFFU) < 0)
        return -1;
    if (qmi8658_write_verify(QMI_REG_CTRL3, QMI_CTRL3_CONFIG, 0xFFU) < 0)
        return -1;
    if (qmi8658_write_verify(QMI_REG_CTRL6, QMI_CTRL6_CONFIG, 0xFFU) < 0)
        return -1;
    if (qmi8658_write_verify(QMI_REG_CTRL7, QMI_CTRL7_CONFIG, 0xFFU) < 0)
        return -1;
    if (qmi8658_write_verify(QMI_REG_CTRL8, QMI_CTRL8_CONFIG, 0xFFU) < 0)
        return -1;
    if (qmi8658_write_verify(QMI_REG_FIFO_WTM, QMI_FIFO_WTM_CONFIG, 0xFFU) < 0)
        return -1;
    /* FIFO_CTRL bit7 is the transient FIFO_RD_MODE bit.  The low stream and
     * capacity bits are the formal configuration and are the bits verified. */
    if (qmi8658_write_verify(QMI_REG_FIFO_CTRL, QMI_FIFO_CTRL_CONFIG, 0x0FU) < 0)
        return -1;
    return 0;
}

static int qmi8658_init(void)
{
    uint8_t who;
    if (i2c_read(0x6A, 0x00, &who, 1) < 0) return -1;
    if (who != 0x05) return -1;

    /* The QMI8658A has no exact 25 Hz 6DOF ODR.  Use its native legal
     * 28.025 Hz ODR and preserve every FIFO sample in acquisition order. */
    if (qmi8658_apply_stream_config() < 0) return -1;
    qmi_irq_init();

    return 0;
}

static int qmi8658_read(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[12];
    if (i2c_read(0x6A, 0x35, buf, 12) < 0) return -1;
    *ax = to_signed16(buf[1], buf[0]);
    *ay = to_signed16(buf[3], buf[2]);
    *az = to_signed16(buf[5], buf[4]);
    *gx = to_signed16(buf[7], buf[6]);
    *gy = to_signed16(buf[9], buf[8]);
    *gz = to_signed16(buf[11], buf[10]);
    return 0;
}

static int mts4_start(void)
{
    uint8_t cmd[] = {0x04, 0x03};
    return i2c_write(0x41, cmd, 2);
}

/* Returns 0 when complete, 1 while the conversion is still running, -1 on
 * an I2C or status error.  The caller is responsible for scheduling another
 * check; this function never busy-waits. */
static int mts4_finish(float *temp)
{
    uint8_t status;
    if (i2c_read(0x41, 0x03, &status, 1) < 0) return -1;
    if (status & 0x20U) return 1;

    uint8_t data[2];
    if (i2c_read(0x41, 0x00, data, 2) < 0) return -1;
    int16_t raw = (int16_t)((uint16_t)data[1] << 8 | data[0]);
    *temp = (float)raw / 256.0f + 25.0f;
    return 0;
}

static void icp_wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_write(0x63, buf, 2);
}

static uint8_t icp_rd(uint8_t reg)
{
    uint8_t v = 0;
    i2c_read(0x63, reg, &v, 1);
    return v;
}

static uint8_t icp_otp_read(uint8_t otp_addr)
{
    icp_wr(0xB5, otp_addr);
    icp_wr(0xB6, 0x10 | (otp_addr >> 4));
    for (uint8_t i = 0; i < 100; i++) {
        if (!(icp_rd(0xB9) & 0x01)) break;
        delay_ms(1);
    }
    return icp_rd(0xB8);
}

static int icp20100_init(void)
{
    uint8_t who;
    if (i2c_read(0x63, 0x0C, &who, 1) < 0) return -1;
    if (who != 0x63) return -1;

    uint8_t dummy[2] = {0x00, 0x00};
    i2c_write(0x63, dummy, 2);
    i2c_write(0x63, dummy, 2);

    uint8_t boot_status = icp_rd(0xBF);
    if (!(boot_status & 0x01)) {
        icp_wr(0xC0, 0x01);
        delay_ms(4);
        icp_wr(0xBE, 0x1F);
        icp_wr(0xAC, 0x03);
        delay_ms(1);
        icp_wr(0xBC, 0x80);
        delay_ms(1);
        icp_wr(0xBC, 0x00);
        delay_ms(1);
        icp_wr(0xAF, 0x04);
        icp_wr(0xB0, 0x04);
        icp_wr(0xB1, 0x21);
        icp_wr(0xB2, 0x20);
        icp_wr(0xAD, 0x10);
        icp_wr(0xAE, 0x80);

        uint8_t offset = icp_otp_read(0xF8);
        uint8_t gain = icp_otp_read(0xF9);
        uint8_t hfosc = icp_otp_read(0xFA);

        icp_wr(0xAC, 0x00);
        delay_ms(1);
        icp_wr(0x05, offset & 0x3F);
        uint8_t trim2 = icp_rd(0x07);
        trim2 = (trim2 & 0x8F) | ((gain & 0x07) << 4);
        icp_wr(0x07, trim2);
        icp_wr(0x06, hfosc);
        icp_wr(0xBE, 0x00);
        icp_wr(0xC0, 0x00);
        icp_wr(0xBF, 0x01);
    }

    icp_wr(0xC4, 0x80);
    delay_ms(1);
    return 0;
}

static int icp20100_read(float *pressure_kpa, float *temp)
{
    for (uint8_t i = 0; i < 100; i++) {
        uint8_t ds = icp_rd(0xCD);
        if (ds & 0x01) break;
        delay_ms(1);
    }

    icp_wr(0xC0, 0x90);

    uint8_t fifo = 0;
    for (uint16_t i = 0; i < 300; i++) {
        fifo = icp_rd(0xC4);
        if (fifo & 0x1F) break;
        delay_ms(1);
    }
    if (!(fifo & 0x1F)) return -1;

    uint8_t data[6];
    if (i2c_read(0x63, 0xFA, data, 6) < 0) return -1;

    int32_t p_raw = ((int32_t)(data[2] & 0x0F) << 16) | ((int32_t)data[1] << 8) | data[0];
    int32_t t_raw = ((int32_t)(data[5] & 0x0F) << 16) | ((int32_t)data[4] << 8) | data[3];
    if (p_raw & 0x80000) p_raw |= (int32_t)0xFFF00000;
    if (t_raw & 0x80000) t_raw |= (int32_t)0xFFF00000;

    r_icp_p_raw = p_raw;
    r_icp_t_raw = t_raw;
    *pressure_kpa = ((float)p_raw / 131072.0f) * 40.0f + 70.0f;
    *temp = ((float)t_raw / 262144.0f) * 65.0f + 25.0f;

    icp_wr(0xC0, 0x00);
    uint8_t dummy = 0;
    i2c_read(0x63, 0x00, &dummy, 1);
    return 0;
}

static void icp20100_start_mode3(void)
{
    /* MODE_SELECT: MEAS_CONFIG=011 (bits7:5), continuous (bit3), normal
     * power (bit2=0), pressure-first FIFO output (bits1:0=00).  0x68 is
     * therefore Mode 3 at the datasheet's approximately 2 Hz ODR. */
    icp_wr(0xC2, 0xFF); /* mask all interrupt sources; INT is not wired */
    icp_wr(0xC3, 0x00); /* no watermark interrupt required for polling */
    icp_wr(0xC4, 0x80); /* flush stale startup data */
    icp_wr(0xC0, 0x68);
}

static void icp20100_reset_fifo_after_loss(void)
{
    /* C4 bit7 is the FIFO flush command.  Re-apply the exact Mode 3 stream
     * value afterwards; otherwise a full/failed FIFO can remain in a state
     * where polling never observes another sample. */
    icp_wr(0xC4, 0x80);
    icp_wr(0xC0, 0x68);
    frame_status |= FRAME_STATUS_ICP_FAIL;
    r_icp_ok = STATUS_FAIL;
}

static void icp20100_service(void)
{
    uint8_t fill = icp_rd(0xC4);
    uint8_t level = fill & 0x1FU;
    if (level == 0 || (fill & 0x40U)) return;
    if (fill & 0x20U) { /* FIFO full: unread history is no longer complete. */
        icp20100_reset_fifo_after_loss();
        return;
    }
    if (level > 16U) {
        icp20100_reset_fifo_after_loss();
        return;
    }

    /* Drain a bounded prefix when the queue cannot yet represent the whole
     * backlog.  An old prefix that can never be appended is explicitly
     * consumed and counted; a future sample blocked only by temporary queue
     * pressure remains in hardware for the next turn. */
    uint8_t take = level > 4U ? 4U : level;
    uint32_t oldest_ms = runtime_now_ms();
    uint32_t full_age = (uint32_t)(level - 1U) * 500U;
    if (oldest_ms >= full_age) oldest_ms -= full_age;
    else oldest_ms = 0U;
    uint8_t discard = 0U, admit = 0U;
    issue3_icp_batch_prefix_plan(&frame_queue, oldest_ms, level, take,
                                 &discard, &admit);
    if (discard == 0U && admit == 0U) return;
    if (discard != 0U) {
        frame_status |= FRAME_STATUS_ICP_FAIL;
        if ((uint32_t)r_icp_late_samples + discard > 0xFFFFU)
            r_icp_late_samples = 0xFFFFU;
        else
            r_icp_late_samples = (uint16_t)(r_icp_late_samples + discard);
        issue3_diag_note_old_icp(oldest_ms, discard);
    }

    uint8_t consume = (uint8_t)(discard + admit);
    for (uint8_t i = 0; i < consume; i++) {
        uint8_t data[6];
        if (i2c_read(0x63, 0xFA, data, 6) < 0) {
            icp20100_reset_fifo_after_loss();
            return;
        }
        if (issue3_diag_active())
            issue3_diag_sat_inc(&r_issue3_diag.icp_received);
        if (i < discard) continue;
        int32_t p_raw = ((int32_t)(data[2] & 0x0FU) << 16) |
                        ((int32_t)data[1] << 8) | data[0];
        int32_t t_raw = ((int32_t)(data[5] & 0x0FU) << 16) |
                        ((int32_t)data[4] << 8) | data[3];
        if (p_raw & 0x80000) p_raw |= (int32_t)0xFFF00000;
        if (t_raw & 0x80000) t_raw |= (int32_t)0xFFF00000;
        r_icp_p_raw = p_raw;
        r_icp_t_raw = t_raw;
        r_icp_ok = STATUS_OK;
        icp_sample_seen = 1U;
        icp_last_sample_ms = runtime_now_ms();
        /* The FIFO is read as a burst, so the timestamp is the actual host
         * read time plus the known 2 Hz sample spacing. */
        (void)frame_add_icp(oldest_ms + (uint32_t)i * 500U, p_raw, t_raw);
    }
}

static int gzp6816d_start_addr(uint8_t addr)
{
    r_gzp_stage = 1;
    bb_i2c_recover();
    uint8_t cmd = GZP_MEASURE_CMD;
    if (i2c_write(addr, &cmd, 1) < 0) { r_gzp_stage = 2; return -1; }
    r_gzp_addr_used = addr;
    return 0;
}

static int gzp6816d_finish_addr(uint8_t addr)
{
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    r_gzp_stage = 3;
    if (i2c_read_raw(addr, data, 6) < 0) { r_gzp_stage = 4; return -1; }

    if ((data[0] | data[1] | data[2] | data[3] | data[4] | data[5]) == 0) { r_gzp_stage = 5; return -1; }
    if (data[0] & 0x20U) { r_gzp_stage = 6; return 1; }

    for (uint8_t i = 0; i < 6; i++) r_gzp_data[i] = data[i];

    r_gzp_p_raw = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
    r_gzp_t_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    r_gzp_stage = 0;
    return 0;
}

static int gzp6816d_start(void)
{
    return gzp6816d_start_addr(0x78);
}

static int gzp6816d_finish(void)
{
    return gzp6816d_finish_addr(0x78);
}

static int gzp_start_callback(void *ctx)
{
    (void)ctx;
    return gzp6816d_start();
}

static void gzp_note_callback(void *ctx, uint32_t count)
{
    (void)ctx;
    gzp_note_missed(count);
    if (issue3_diag_active())
        issue3_diag_sat_add(&r_issue3_diag.gzp_drop_time, count);
}

static uint8_t gzp_result_can_enter_frame(uint32_t timestamp_ms)
{
    return issue3_frame_queue_span_fits(&frame_queue, timestamp_ms, timestamp_ms);
}

static void oled_delay(void)
{
    for (volatile uint32_t i = 0; i < 100; i++);
}

static void oled_sda(int high)
{
    if (high) GPIOA->BSRR = OLED_SDA_PIN;
    else GPIOA->BRR = OLED_SDA_PIN;
    oled_delay();
}

static void oled_scl(int high)
{
    if (high) GPIOA->BSRR = OLED_SCL_PIN;
    else GPIOA->BRR = OLED_SCL_PIN;
    oled_delay();
}

static void oled_i2c_start(void)
{
    oled_sda(1);
    oled_scl(1);
    oled_sda(0);
    oled_scl(0);
}

static void oled_i2c_stop(void)
{
    oled_sda(0);
    oled_scl(1);
    oled_sda(1);
}

static int oled_i2c_write(uint8_t value)
{
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        oled_sda(value & mask);
        oled_scl(1);
        oled_scl(0);
    }
    oled_sda(1);
    oled_scl(1);
    int ack = !(GPIOA->IDR & OLED_SDA_PIN);
    oled_scl(0);
    return ack;
}

static void oled_command(uint8_t command)
{
    oled_i2c_start();
    (void)oled_i2c_write(OLED_ADDR_WRITE);
    (void)oled_i2c_write(0x00);
    (void)oled_i2c_write(command);
    oled_i2c_stop();
}

static const uint8_t oled_font_upper[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static const uint8_t oled_font_digit[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
};

static void oled_glyph(char c, uint8_t out[5])
{
    for (uint8_t i = 0; i < 5; i++) out[i] = 0;
    if (c >= 'A' && c <= 'Z') {
        for (uint8_t i = 0; i < 5; i++) out[i] = oled_font_upper[(uint8_t)(c - 'A')][i];
    } else if (c >= '0' && c <= '9') {
        for (uint8_t i = 0; i < 5; i++) out[i] = oled_font_digit[(uint8_t)(c - '0')][i];
    } else if (c == '-') {
        out[0]=0x08; out[1]=0x08; out[2]=0x08; out[3]=0x08; out[4]=0x08;
    } else if (c == '.') {
        out[1]=0x60; out[2]=0x60;
    } else if (c == ':') {
        out[1]=0x36; out[2]=0x36;
    } else if (c == '/') {
        out[0]=0x20; out[1]=0x10; out[2]=0x08; out[3]=0x04; out[4]=0x02;
    }
}

static void oled_write_line(uint8_t page, const char line[21])
{
    oled_command((uint8_t)(0xB0U | (page & 7U)));
    oled_command((uint8_t)(OLED_COLUMN_OFFSET & 0x0FU));
    oled_command(0x10);

    oled_i2c_start();
    (void)oled_i2c_write(OLED_ADDR_WRITE);
    (void)oled_i2c_write(0x40);
    for (uint8_t c = 0; c < 21; c++) {
        uint8_t glyph[5];
        oled_glyph(line[c], glyph);
        for (uint8_t i = 0; i < 5; i++) (void)oled_i2c_write(glyph[i]);
        (void)oled_i2c_write(0x00);
    }
    (void)oled_i2c_write(0x00);
    (void)oled_i2c_write(0x00);
    oled_i2c_stop();
}

static void oled_line_clear(char line[21])
{
    for (uint8_t i = 0; i < 21; i++) line[i] = ' ';
}

static uint8_t oled_line_text(char line[21], uint8_t pos, const char *text)
{
    while (*text && pos < 21) line[pos++] = *text++;
    return pos;
}

static uint8_t oled_line_int(char line[21], uint8_t pos, int32_t value, uint8_t width)
{
    char temp[11];
    uint8_t count = 0;
    uint8_t negative = value < 0;
    uint32_t magnitude = negative ? (uint32_t)(-value) : (uint32_t)value;
    do {
        temp[count++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude && count < sizeof(temp));

    uint8_t needed = (uint8_t)(count + negative);
    while (width > needed && pos < 21) { line[pos++] = ' '; width--; }
    if (negative && pos < 21) line[pos++] = '-';
    while (count && pos < 21) line[pos++] = temp[--count];
    return pos;
}

static uint8_t oled_line_fixed1(char line[21], uint8_t pos, int32_t value_x10)
{
    if (value_x10 < 0) {
        if (pos < 21) line[pos++] = '-';
        value_x10 = -value_x10;
    }
    pos = oled_line_int(line, pos, value_x10 / 10, 1);
    if (pos < 21) line[pos++] = '.';
    if (pos < 21) line[pos++] = (char)('0' + value_x10 % 10);
    return pos;
}

static uint8_t oled_line_status(char line[21], uint8_t pos, int ok)
{
    return oled_line_text(line, pos, ok ? "OK" : "FAIL");
}

static uint8_t oled_line_tf(char line[21], uint8_t pos, int ok)
{
    if (pos < 21) line[pos++] = ok ? 'T' : 'F';
    return pos;
}

static void oled_prepare(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER &= ~((3U << 26) | (3U << 28));
    GPIOA->MODER |= (1U << 26) | (1U << 28);
    GPIOA->OTYPER |= OLED_SDA_PIN | OLED_SCL_PIN;
    GPIOA->PUPDR &= ~((3U << 26) | (3U << 28));
    GPIOA->PUPDR |= (1U << 26) | (1U << 28);
    oled_sda(1);
    oled_scl(1);
    delay_ms(20);
}

static void oled_init(void)
{
    oled_prepare();
    const uint8_t commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    for (uint8_t i = 0; i < sizeof(commands); i++) oled_command(commands[i]);

    char blank[21];
    oled_line_clear(blank);
    for (uint8_t page = 0; page < 8; page++) oled_write_line(page, blank);
}

static void oled_release_swd_pins(void)
{
    const uint32_t oled_mode_mask = (3U << 26) | (3U << 28);
    const uint32_t oled_afr_mask = (0xFU << 20) | (0xFU << 24);

    /* OLED shares PA13/PA14 with SWD; restore AF0 after startup inspection. */
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->AFRH &= ~oled_afr_mask;
    GPIOA->OSPEEDR |= oled_mode_mask;
    GPIOA->OTYPER &= ~(OLED_SDA_PIN | OLED_SCL_PIN);
    GPIOA->PUPDR &= ~oled_mode_mask;
    GPIOA->PUPDR |= (1U << 26) | (2U << 28);
    GPIOA->MODER &= ~oled_mode_mask;
    GPIOA->MODER |= (2U << 26) | (2U << 28);
}

static int32_t icp_pressure_hpa_x10(void)
{
    return 7000 + (r_icp_p_raw * 4000) / 131072;
}

static int32_t gzp_pressure_hpa_x10(void)
{
    const int32_t dmin = 1677722;
    const int32_t span = 15099494 - 1677722;
    int64_t scaled = ((int64_t)(int32_t)r_gzp_p_raw - dmin) * 8000;
    return 3000 + (int32_t)(scaled / span);
}

static void oled_show_sensors(void)
{
    char line[21];
    uint8_t pos;

    oled_line_clear(line); pos = oled_line_text(line, 0, "IMU ");
    (void)oled_line_status(line, pos, r_qmi_ok == STATUS_OK); oled_write_line(0, line);

    oled_line_clear(line); pos = oled_line_text(line, 0, "A");
    pos = oled_line_int(line, pos, r_ax, 6); pos = oled_line_int(line, pos, r_ay, 6);
    (void)oled_line_int(line, pos, r_az, 6); oled_write_line(1, line);

    oled_line_clear(line); pos = oled_line_text(line, 0, "G");
    pos = oled_line_int(line, pos, r_gx, 6); pos = oled_line_int(line, pos, r_gy, 6);
    (void)oled_line_int(line, pos, r_gz, 6); oled_write_line(2, line);

    oled_line_clear(line); pos = oled_line_text(line, 0, "MTS ");
    pos = oled_line_status(line, pos, r_mts4_ok == STATUS_OK); pos = oled_line_text(line, pos, " T ");
    pos = oled_line_fixed1(line, pos, (int32_t)r_mts4_temp * 10 / 256);
    (void)oled_line_text(line, pos, "C"); oled_write_line(3, line);

    oled_line_clear(line); pos = oled_line_text(line, 0, "ICP ");
    pos = oled_line_status(line, pos, r_icp_ok == STATUS_OK); pos = oled_line_text(line, pos, " P ");
    pos = oled_line_fixed1(line, pos, icp_pressure_hpa_x10());
    (void)oled_line_text(line, pos, "HPA"); oled_write_line(4, line);

    oled_line_clear(line); pos = oled_line_text(line, 0, "GZP ");
    pos = oled_line_status(line, pos, r_gzp_ok == STATUS_OK); pos = oled_line_text(line, pos, " P ");
    pos = oled_line_fixed1(line, pos, gzp_pressure_hpa_x10());
    (void)oled_line_text(line, pos, "HPA"); oled_write_line(5, line);

    oled_line_clear(line); pos = oled_line_text(line, 0, "NF ");
    (void)oled_line_status(line, pos, r_nf_ok == STATUS_OK); oled_write_line(6, line);
}

static void oled_show_sd_debug(void)
{
    char line[21];
    uint8_t pos;

    oled_line_clear(line); pos = oled_line_text(line, 0, "SD ");
    pos = oled_line_tf(line, pos, r_sd_cmd0 == 1);
    pos = oled_line_tf(line, pos, r_sd_cmd8 == 1);
    pos = oled_line_tf(line, pos, r_sd_acmd41 == 0);
    pos = oled_line_tf(line, pos, r_sd_cmd58 == 0);
    (void)oled_line_tf(line, pos, r_sd_read0 == 0);
    oled_write_line(7, line);
}

static void oled_startup_check(int imu_initialized, int mts4_initialized, int icp_initialized)
{
    oled_init();
    uint32_t start_ms = r_system_ms;
    uint8_t mts_started = 0;
    uint8_t gzp_started = 0;
    uint32_t mts_start_ms = 0;
    uint32_t gzp_start_ms = 0;

    /* Keep the display live for manual movement and pressure checks only. */
    do {
        int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        if (imu_initialized == 0 && qmi8658_read(&ax, &ay, &az, &gx, &gy, &gz) == 0) {
            r_qmi_ok = STATUS_OK;
            qmi_sample_seen = 1U;
            qmi_last_sample_ms = r_system_ms;
            r_ax=ax; r_ay=ay; r_az=az; r_gx=gx; r_gy=gy; r_gz=gz;
        } else {
            r_qmi_ok = STATUS_FAIL;
        }

        if (mts4_initialized == 0) {
            if (!mts_started) {
                mts_started = (mts4_start() == 0);
                mts_start_ms = r_system_ms;
            } else if ((uint32_t)(r_system_ms - mts_start_ms) >= 50U) {
                float body_temp = 0.0f;
                int result = mts4_finish(&body_temp);
                if (result == 0) {
                    r_mts4_ok = STATUS_OK;
                    mts4_sample_seen = 1U;
                    mts4_last_sample_ms = r_system_ms;
                    r_mts4_temp = (int16_t)(body_temp * 256.0f);
                    mts_started = 0;
                } else if (result < 0) {
                    r_mts4_ok = STATUS_FAIL;
                    mts_started = 0;
                }
            }
        }

        float pressure = 0.0f, air_temp = 0.0f;
        if (icp_initialized == 0 && icp20100_read(&pressure, &air_temp) == 0) {
            r_icp_ok = STATUS_OK;
            icp_sample_seen = 1U;
            icp_last_sample_ms = r_system_ms;
        } else {
            r_icp_ok = STATUS_FAIL;
        }

        if (!gzp_started) {
            gzp_started = (gzp6816d_start() == 0);
            gzp_start_ms = r_system_ms;
        } else if ((uint32_t)(r_system_ms - gzp_start_ms) >= GZP_CONVERSION_WAIT_MS) {
            int result = gzp6816d_finish();
            if (result == 0) {
                r_gzp_ok = STATUS_OK;
                gzp_sample_seen = 1U;
                gzp_last_sample_ms = r_system_ms;
                gzp_started = 0;
            } else if (result < 0 || (uint32_t)(r_system_ms - gzp_start_ms) >= GZP_CONVERSION_TIMEOUT_MS) {
                r_gzp_ok = STATUS_FAIL;
                gzp_started = 0;
            }
        }
        oled_show_sensors();
        oled_show_sd_debug();
        delay_ms(50);
    } while ((uint32_t)(r_system_ms - start_ms) < STARTUP_DISPLAY_HOLD_MS);

    oled_command(0xAE);
    oled_release_swd_pins();
    systick_stop();
}

static void sd_cs(int high);
static uint8_t sd_spi_xfer(uint8_t out);
static uint8_t sd_read_sector0(void);
static int sd_async_start_read(uint32_t lba, uint8_t *buf);
static int sd_async_start_write(uint32_t lba, const uint8_t *buf);
static int sd_async_step(void);
static int sd_async_take_result(void);

/* NF-03 embeds a Si24R1.  Heartbeats are deliberately one-way broadcasts:
 * ACK is disabled because a reverse ACK is not a useful reliability signal
 * for this low-duty-cycle telemetry link.  Each logical heartbeat is sent as
 * three separately assembled copies by the non-blocking scheduler below. */
#define NF_REG_CONFIG       0x00U
#define NF_REG_EN_AA        0x01U
#define NF_REG_EN_RXADDR    0x02U
#define NF_REG_SETUP_AW     0x03U
#define NF_REG_SETUP_RETR   0x04U
#define NF_REG_RF_CH        0x05U
#define NF_REG_RF_SETUP     0x06U
#define NF_REG_STATUS       0x07U
#define NF_REG_RX_PW_P0     0x11U
#define NF_REG_TX_ADDR      0x10U
#define NF_REG_DYNPD        0x1CU
#define NF_CMD_NOP          0xFFU
#define NF_CMD_FLUSH_TX     0xE1U
#define NF_CMD_FLUSH_RX     0xE2U
#define NF_CONFIG_EN_CRC    0x08U
#define NF_CONFIG_PWR_UP   0x02U
#define NF_CONFIG_PRIM_RX  0x01U
#define NF_CONFIG_TX       (NF_CONFIG_EN_CRC | NF_CONFIG_PWR_UP)
#define NF_CONFIG_RX       (NF_CONFIG_TX | NF_CONFIG_PRIM_RX)
#define NF_EN_AA_DISABLED  0x00U
#define NF_EN_RXADDR_DISABLED 0x00U
#define NF_SETUP_RETR_DISABLED 0x00U
#define NF_STATUS_RX_DR    0x40U
#define NF_STATUS_TX_DS    0x20U
#define NF_STATUS_MAX_RT   0x10U
#define NF_TX_POLL_LIMIT_MS 20U

static void nf_cs(int high)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF)) return;
    if (high) GPIOA->BSRR = NF_CS_PIN;
    else GPIOA->BRR = NF_CS_PIN;
    __asm volatile("nop" ::: "memory");
}

static void nf_ce(int high)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF)) return;
    if (high) GPIOB->BSRR = NF_CE_PIN;
    else GPIOB->BRR = NF_CE_PIN;
    __asm volatile("nop" ::: "memory");
}

static uint8_t nf_spi_xfer(uint8_t out)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF))
        return 0xFFU;
    return sd_spi_xfer(out);
}

static uint8_t nf_cmd(uint8_t cmd)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF))
        return 0xFFU;
    nf_cs(0);
    uint8_t s = nf_spi_xfer(cmd);
    nf_cs(1);
    return s;
}

static uint8_t nf_read_reg(uint8_t reg)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF))
        return 0xFFU;
    nf_cs(0);
    nf_spi_xfer(0x00 | (reg & 0x1F));
    uint8_t v = nf_spi_xfer(0xFF);
    nf_cs(1);
    return v;
}

static void nf_write_reg(uint8_t reg, uint8_t val)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF)) return;
    nf_ce(0);
    nf_cs(0);
    nf_spi_xfer(0x20 | (reg & 0x1F));
    nf_spi_xfer(val);
    nf_cs(1);
}

static void nf_write_addr(uint8_t reg, const uint8_t *addr, uint8_t len)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF)) return;
    nf_ce(0);
    nf_cs(0);
    nf_spi_xfer(0x20 | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) nf_spi_xfer(addr[i]);
    nf_cs(1);
}

static void nf_gpio_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    GPIOA->MODER &= ~(3U << 30);
    GPIOA->MODER |= (1U << 30);

    GPIOB->MODER &= ~((3U << 8) | (3U << 10));
    GPIOB->MODER |= (1U << 8) | (1U << 10);
    GPIOB->PUPDR &= ~((3U << 8) | (3U << 10) | (3U << 16));
    GPIOB->PUPDR |= (1U << 16);

    nf_ce(0);
    nf_cs(1);
}

static void nf_flush_rx(void)
{
    nf_cmd(NF_CMD_FLUSH_RX);
}

static void nf_flush_tx(void)
{
    nf_cmd(NF_CMD_FLUSH_TX);
}

static int nf03_init(void)
{
    if (!issue3_spi_bus_try_acquire(&shared_spi_bus,
                                    ISSUE3_SPI_OWNER_NF)) return -1;
    int result = -1;
    r_nf_stage = 1;
    nf_gpio_init();
    sd_cs(1);
    delay_ms(5);

    nf_ce(0);

    nf_write_reg(NF_REG_STATUS,
                 NF_STATUS_RX_DR | NF_STATUS_TX_DS | NF_STATUS_MAX_RT);
    nf_flush_tx();
    nf_flush_rx();
    nf_write_reg(NF_REG_CONFIG, NF_CONFIG_TX);
    nf_write_reg(NF_REG_EN_AA, NF_EN_AA_DISABLED);
    nf_write_reg(NF_REG_EN_RXADDR, NF_EN_RXADDR_DISABLED);
    nf_write_reg(NF_REG_SETUP_AW, 0x03U);
    nf_write_reg(NF_REG_SETUP_RETR, NF_SETUP_RETR_DISABLED);
    nf_write_reg(NF_REG_RF_CH, 0x4CU);
    nf_write_reg(NF_REG_RF_SETUP, 0x26U);
    nf_write_reg(NF_REG_STATUS,
                 NF_STATUS_RX_DR | NF_STATUS_TX_DS | NF_STATUS_MAX_RT);
    nf_write_reg(NF_REG_RX_PW_P0, 0x20U);
    nf_write_reg(NF_REG_DYNPD, 0x00U);

    uint8_t setup = nf_read_reg(NF_REG_CONFIG);
    uint8_t en_aa = nf_read_reg(NF_REG_EN_AA);
    uint8_t retr = nf_read_reg(NF_REG_SETUP_RETR);
    uint8_t rf = nf_read_reg(NF_REG_RF_SETUP);
    if (setup != NF_CONFIG_TX || en_aa != NF_EN_AA_DISABLED ||
        retr != NF_SETUP_RETR_DISABLED || (rf & 0x0FU) != 0x06U) {
        r_nf_stage = 2;
        r_nf_ok = STATUS_FAIL;
        r_nf_init_ok = STATUS_FAIL;
        goto nf_init_done;
    }

    /* All devices share one RF network address.  The build-time device ID is
     * carried only in the heartbeat payload, so one receiver can hear all
     * deployed devices. */
    static const uint8_t addr_pcb32[5] = {'P', 'C', 'B', '3', '2'};
    nf_write_addr(NF_REG_TX_ADDR, addr_pcb32, 5);

    r_nf_stage = 0;
    r_nf_ok = STATUS_OK;
    r_nf_init_ok = STATUS_OK;
    result = 0;

nf_init_done:
    issue3_spi_bus_release(&shared_spi_bus, ISSUE3_SPI_OWNER_NF);
    return result;
}

static int nf03_wake(void)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF))
        return -1;
    nf_ce(0);
    nf_write_reg(NF_REG_CONFIG, NF_CONFIG_TX);
    short_sleep_ms(2);
    return 0;
}

static void nf03_sleep(void)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF)) return;
    nf_ce(0);
    uint8_t config = nf_read_reg(NF_REG_CONFIG);
    nf_write_reg(NF_REG_CONFIG, (uint8_t)(config & ~NF_CONFIG_PWR_UP));
}

static int nf03_send(const uint8_t *data, uint8_t len)
{
    if (!issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF))
        return -1;
    if (len > 32) len = 32;

    sd_cs(1);
    nf_ce(0);
    nf_write_reg(NF_REG_CONFIG, NF_CONFIG_TX);
    /* Always start from an empty FIFO.  With EN_AA/SETUP_RETR disabled the
     * radio performs one transmission and never turns a failed copy into a
     * hidden retry or MAX_RT transaction. */
    nf_flush_tx();
    nf_write_reg(NF_REG_STATUS,
                 NF_STATUS_RX_DR | NF_STATUS_TX_DS | NF_STATUS_MAX_RT);

    nf_cs(0);
    nf_spi_xfer(0xA0);
    for (uint8_t i = 0; i < len; i++) nf_spi_xfer(data[i]);
    while (len < 32) { nf_spi_xfer(0x00); len++; }
    nf_cs(1);

    nf_ce(1);
    uint8_t status = 0;
    uint8_t terminal = 0;
    for (uint8_t elapsed = 0; elapsed < NF_TX_POLL_LIMIT_MS; elapsed++) {
        short_sleep_ms(1);
        status = nf_cmd(NF_CMD_NOP);
        if (status & NF_STATUS_TX_DS) {
            terminal = 1;
            break;
        }
    }
    nf_ce(0);

    /* TX_DS means only that this copy left the radio; without ACK it says
     * nothing about whether a receiver heard it. */
    uint8_t sent = terminal && (status & NF_STATUS_TX_DS);
    if (sent) {
        /* r_nf_tx_cnt counts copies completed by the RF transmitter, not
         * acknowledged packets or logical heartbeat records. */
        r_nf_tx_cnt++;
        r_nf_ok = STATUS_OK;
    } else {
        r_nf_ok = STATUS_FAIL;
        /* A failed copy must not block later copies or the next absolute
         * heartbeat. */
        nf_write_reg(NF_REG_STATUS,
                     NF_STATUS_RX_DR | NF_STATUS_TX_DS | NF_STATUS_MAX_RT);
        nf_flush_tx();
        return -1;
    }
    nf_write_reg(NF_REG_STATUS, NF_STATUS_TX_DS | NF_STATUS_MAX_RT);
    return 0;
}

static uint32_t nf03_device_id_value(void)
{
    return ((uint32_t)PCB32_DEVICE_ID0 << 16) |
           ((uint32_t)PCB32_DEVICE_ID1 << 8) | PCB32_DEVICE_ID2;
}

static void sensor_health_refresh(uint32_t now_ms)
{
    r_qmi_ok = (qmi_sample_seen &&
                (uint32_t)(now_ms - qmi_last_sample_ms) <=
                QMI_HEALTH_TIMEOUT_MS) ? STATUS_OK : STATUS_FAIL;
    r_gzp_ok = (gzp_sample_seen &&
                (uint32_t)(now_ms - gzp_last_sample_ms) <=
                GZP_HEALTH_TIMEOUT_MS) ? STATUS_OK : STATUS_FAIL;
    r_icp_ok = (icp_sample_seen &&
                (uint32_t)(now_ms - icp_last_sample_ms) <=
                ICP_HEALTH_TIMEOUT_MS) ? STATUS_OK : STATUS_FAIL;
    /* MTS4 is intentionally a 30-minute measurement.  Give it one whole
     * period plus a small conversion margin before declaring it stale. */
    r_mts4_ok = (mts4_sample_seen &&
                 (uint32_t)(now_ms - mts4_last_sample_ms) <=
                 MTS4_HEALTH_TIMEOUT_MS) ? STATUS_OK : STATUS_FAIL;
}

/* Assemble immediately before every physical copy.  Thus uptime and all
 * sensor/SD status fields describe that copy's actual send attempt rather
 * than a payload cached at the beginning of the 30-second broadcast window. */
static int nf03_send_heartbeat(uint32_t sequence)
{
    if (r_nf_init_ok != STATUS_OK ||
        !issue3_spi_bus_owned(&shared_spi_bus, ISSUE3_SPI_OWNER_NF)) return -1;

    uint32_t uptime_ms = runtime_now_ms();
    sensor_health_refresh(uptime_ms);
    uint8_t msg[32];
    issue3_heartbeat_build_payload(
        msg, nf03_device_id_value(), NF_HEARTBEAT_PROTOCOL_VERSION,
        NF_HEARTBEAT_MESSAGE_TYPE, sequence, uptime_ms,
        (uint8_t)(r_qmi_ok == STATUS_OK ? 0xA1 : 0xA0),
        (uint8_t)(r_icp_ok == STATUS_OK ? 0xB1 : 0xB0),
        (uint8_t)(r_gzp_ok == STATUS_OK ? 0xC1 : 0xC0),
        (uint8_t)(r_mts4_ok == STATUS_OK ? 0xD1 : 0xD0),
        (uint8_t)((r_sd_init_ok == STATUS_OK ? NF_STATUS_SD_INIT_OK : 0U) |
                  (log_active ? NF_STATUS_LOG_ACTIVE : 0U) |
                  (r_sd_full ? NF_STATUS_SD_FULL : 0U)));

    (void)nf03_wake();
    int result = nf03_send(msg, sizeof(msg));
    nf03_sleep();
    return result;
}

static void nf03_broadcast_service(uint32_t now_ms)
{
    uint32_t sequence = 0;
    uint8_t copy_index = 0;
    if (!issue3_nf_broadcast_due(&nf_broadcast, now_ms, &sequence,
                                 &copy_index)) return;

    /* If an SD transaction owns the shared pins, leave next_copy and the
     * logical heartbeat schedule untouched.  The next loop retries this same
     * copy after SD has released CS; no sequence is allocated twice. */
    if (!issue3_spi_bus_try_acquire(&shared_spi_bus,
                                    ISSUE3_SPI_OWNER_NF)) return;

    /* A failed TX_DS does not abort the logical heartbeat.  Completion here
     * advances to the next deterministic copy regardless of send result. */
    (void)copy_index;
    (void)nf03_send_heartbeat(sequence);
    issue3_nf_broadcast_complete(&nf_broadcast);
    if (!nf_broadcast.active)
        issue3_heartbeat_complete(&nf_heartbeat_schedule);
    issue3_spi_bus_release(&shared_spi_bus, ISSUE3_SPI_OWNER_NF);
}

static void sd_cs(int high)
{
    /* Startup SD probing is ownerless, but at runtime an NF owner must never
     * be able to see SD CS asserted on the shared hardware SPI1 pins. */
    if (!high && shared_spi_bus.owner == ISSUE3_SPI_OWNER_NF) return;
    if (high) GPIOA->BSRR = SD_CS_PIN;
    else GPIOA->BRR = SD_CS_PIN;
    __asm volatile("nop" ::: "memory");
}

static void spi_hw_timeout(void)
{
    r_spi_timeout = 1U;
    if (issue3_diag_active())
        issue3_diag_sat_inc(&r_issue3_diag.spi_timeout_count);
    if (shared_spi_bus.owner == ISSUE3_SPI_OWNER_SD)
        r_sd_ok = STATUS_FAIL;
    else if (shared_spi_bus.owner == ISSUE3_SPI_OWNER_NF)
        r_nf_ok = STATUS_FAIL;
    /* Stop the peripheral after a bounded wait.  The current SD/NF
     * transaction will fail closed; no caller can spin forever on TXE/RXNE or
     * BSY, and a later transaction cannot reuse a wedged peripheral. */
    SPI1->CR1 &= ~SPI_CR1_SPE;
}

static uint8_t spi_hw_xfer(uint8_t out)
{
    uint32_t guard;
    uint8_t in;

    if (r_spi_timeout || !(SPI1->CR1 & SPI_CR1_SPE)) return 0xFFU;

    guard = ISSUE3_SD_SPI_WAIT_LIMIT_LOOPS;
    while ((SPI1->SR & SPI_SR_TXE) == 0U) {
        if (--guard == 0U) {
            spi_hw_timeout();
            return 0xFFU;
        }
    }
    *((volatile uint8_t *)&SPI1->DR) = out;

    guard = ISSUE3_SD_SPI_WAIT_LIMIT_LOOPS;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
        if (--guard == 0U) {
            spi_hw_timeout();
            return 0xFFU;
        }
    }
    in = *((volatile uint8_t *)&SPI1->DR);

    /* Do not release CS or hand the pins to the other peripheral while the
     * final bit is still on the wire. */
    guard = ISSUE3_SD_SPI_WAIT_LIMIT_LOOPS;
    while (SPI1->SR & SPI_SR_BSY) {
        if (--guard == 0U) {
            spi_hw_timeout();
            return 0xFFU;
        }
    }
    return in;
}

static void spi_hw_set_rate(uint8_t br)
{
    /* Rate changes are only made during startup or with the shared owner
     * released.  CS lines are driven high by the caller before this point. */
    if (shared_spi_bus.owner != ISSUE3_SPI_OWNER_NONE) return;
    /* Make the CS precondition explicit at the hardware boundary as well as
     * in the arbiter: no divider/configuration write can occur while either
     * external device is selected. */
    GPIOA->BSRR = SD_CS_PIN | NF_CS_PIN;
    __asm volatile("nop" ::: "memory");
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                (((uint32_t)br << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk);
    SPI1->CR2 = (7U << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;
    SPI1->CR1 |= SPI_CR1_SPE;
}

static void sd_spi_init(void)
{
    RCC->APBENR2 |= RCC_APBENR2_SPI1EN;
    SPI1->CR1 = 0U;
    SPI1->CR2 = (7U << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;
    spi_hw_set_rate(ISSUE3_SD_SPI_RUNTIME_BR);
}

static uint8_t sd_spi_xfer(uint8_t out)
{
    return spi_hw_xfer(out);
}

static void sd_spi_clocks(uint8_t n)
{
    while (n--) sd_spi_xfer(0xFF);
}

static void spi_shared_gpio_init(void)
{
    issue3_spi_bus_init(&shared_spi_bus);
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    /* PA5/PA6/PA7 are SPI1_SCK/MISO/MOSI on AF0 for STM32G030K8T6;
     * PA4 remains a normal GPIO SD_CS.  NF_CS is PA15 and is configured by
     * nf_gpio_init(). */
    GPIOA->MODER &= ~((3U << 8) | (3U << 10) | (3U << 12) | (3U << 14));
    GPIOA->MODER |= (1U << 8) | (2U << 10) | (2U << 12) | (2U << 14);
    GPIOA->AFRL &= ~((0xFU << 20) | (0xFU << 24) | (0xFU << 28));
    GPIOA->OTYPER &= ~(SD_CS_PIN | SD_SCK_PIN | SD_MISO_PIN | SD_MOSI_PIN);
    GPIOA->OSPEEDR |= (3U << 8) | (3U << 10) | (3U << 12) | (3U << 14);
    GPIOA->PUPDR &= ~((3U << 8) | (3U << 10) | (3U << 12) | (3U << 14));
    GPIOA->PUPDR |= (1U << 12); /* MISO/SD pull-up while CS is high. */

    GPIOA->BRR = SD_SCK_PIN;
    GPIOA->BSRR = SD_MOSI_PIN;
    sd_cs(1);
    sd_spi_init();
}

static void sd_card_prepare(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    GPIOB->MODER &= ~(3U << 6);
    GPIOB->MODER |= (1U << 6);
    GPIOB->BSRR = SD_PWR_PIN;

    spi_shared_gpio_init();
    /* SD SPI identification must stay <=400 kHz.  Runtime switches to 2 MHz
     * only after sd_probe() has completed the card's first read. */
    spi_hw_set_rate(ISSUE3_SD_SPI_INIT_BR);
    delay_ms(100);
    sd_spi_clocks(10);
}

static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t *extra, uint8_t extra_len)
{
    sd_cs(1);
    sd_spi_xfer(0xFF);
    sd_cs(0);
    sd_spi_xfer(0xFF);
    sd_spi_xfer(0x40 | cmd);
    sd_spi_xfer((uint8_t)(arg >> 24));
    sd_spi_xfer((uint8_t)(arg >> 16));
    sd_spi_xfer((uint8_t)(arg >> 8));
    sd_spi_xfer((uint8_t)arg);
    sd_spi_xfer(crc);

    uint8_t r = 0xFF;
    for (uint8_t i = 0; i < 10; i++) {
        r = sd_spi_xfer(0xFF);
        if ((r & 0x80) == 0) break;
    }

    for (uint8_t i = 0; i < extra_len; i++) extra[i] = sd_spi_xfer(0xFF);
    sd_cs(1);
    sd_spi_xfer(0xFF);
    return r;
}

static void sd_probe(void)
{
    uint8_t extra[4] = {0, 0, 0, 0};

    r_sd_cmd0 = sd_cmd(0, 0, 0x95, 0, 0);
    r_sd_cmd8 = sd_cmd(8, 0x000001AA, 0x87, extra, 4);

    r_sd_acmd41 = 0xFF;
    for (uint16_t i = 0; i < 200; i++) {
        (void)sd_cmd(55, 0, 0x01, 0, 0);
        r_sd_acmd41 = sd_cmd(41, 0x40000000, 0x01, 0, 0);
        if (r_sd_acmd41 == 0) break;
        delay_ms(5);
    }

    r_sd_cmd58 = sd_cmd(58, 0, 0x01, extra, 4);
    r_sd_ocr = ((uint32_t)extra[0] << 24) | ((uint32_t)extra[1] << 16) |
               ((uint32_t)extra[2] << 8) | extra[3];
    r_sd_read0 = sd_read_sector0();
    r_sd_ok = (r_sd_cmd0 == 1 && r_sd_cmd8 == 1 && r_sd_acmd41 == 0 &&
               r_sd_cmd58 == 0 && r_sd_read0 == 0) ? STATUS_OK : STATUS_FAIL;
    r_sd_init_ok = r_sd_ok;
    /* The low-rate identification phase is complete.  All runtime SD and NF
     * traffic now uses the same SPI1 peripheral at 2 MHz. */
    spi_hw_set_rate(ISSUE3_SD_SPI_RUNTIME_BR);
}

static uint8_t sd_read_sector0(void)
{
    sd_cs(1);
    sd_spi_xfer(0xFF);
    sd_cs(0);
    sd_spi_xfer(0xFF);
    sd_spi_xfer(0x40 | 17);
    sd_spi_xfer(0);
    sd_spi_xfer(0);
    sd_spi_xfer(0);
    sd_spi_xfer(0);
    sd_spi_xfer(0x01);

    uint8_t r = 0xFF;
    for (uint8_t i = 0; i < 10; i++) {
        r = sd_spi_xfer(0xFF);
        if ((r & 0x80) == 0) break;
    }
    if (r != 0) {
        sd_cs(1);
        sd_spi_xfer(0xFF);
        return r;
    }

    uint8_t token = 0xFF;
    for (uint16_t i = 0; i < ISSUE3_SD_READ_TOKEN_POLLS; i++) {
        token = sd_spi_xfer(0xFF);
        if (token != 0xFF) break;
    }
    if (token != 0xFE) {
        sd_cs(1);
        sd_spi_xfer(0xFF);
        return token;
    }

    uint8_t b510 = 0, b511 = 0;
    for (uint16_t i = 0; i < 512; i++) {
        uint8_t b = sd_spi_xfer(0xFF);
        if (i < sizeof(r_sd_sector0)) r_sd_sector0[i] = b;
        if (i == 510) b510 = b;
        if (i == 511) b511 = b;
    }
    (void)sd_spi_xfer(0xFF);
    (void)sd_spi_xfer(0xFF);
    sd_cs(1);
    sd_spi_xfer(0xFF);

    r_sd_mbr_sig = ((uint16_t)b511 << 8) | b510;
    return 0;
}

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

/* Synchronous bootstrap read.  It is used only before runtime_timer_init()
 * while acquisition is stopped; all runtime reads use sd_async_step(). */
static uint8_t sd_read_sector(uint32_t lba, uint8_t *buf)
{
    sd_cs(1);
    sd_spi_xfer(0xFF);
    sd_cs(0);
    sd_spi_xfer(0xFF);
    sd_spi_xfer(0x40 | 17);
    sd_spi_xfer((uint8_t)(lba >> 24));
    sd_spi_xfer((uint8_t)(lba >> 16));
    sd_spi_xfer((uint8_t)(lba >> 8));
    sd_spi_xfer((uint8_t)lba);
    sd_spi_xfer(0x01);

    uint8_t r = 0xFF;
    for (uint8_t i = 0; i < 10; i++) {
        r = sd_spi_xfer(0xFF);
        if ((r & 0x80) == 0) break;
    }
    if (r != 0) {
        sd_cs(1);
        sd_spi_xfer(0xFF);
        return r;
    }

    uint8_t token = 0xFF;
    for (uint16_t i = 0; i < ISSUE3_SD_READ_TOKEN_POLLS; i++) {
        token = sd_spi_xfer(0xFF);
        if (token != 0xFF) break;
    }
    if (token != 0xFE) {
        sd_cs(1);
        sd_spi_xfer(0xFF);
        return token;
    }

    for (uint16_t i = 0; i < 512; i++) buf[i] = sd_spi_xfer(0xFF);
    (void)sd_spi_xfer(0xFF);
    (void)sd_spi_xfer(0xFF);
    sd_cs(1);
    sd_spi_xfer(0xFF);
    return 0;
}

/* Synchronous bootstrap write.  Runtime frame/FAT/metadata writes never call
 * this function; they use the resumable transaction below. */
static uint8_t sd_write_sector(uint32_t lba, const uint8_t *buf)
{
    sd_cs(1);
    sd_spi_xfer(0xFF);
    sd_cs(0);
    sd_spi_xfer(0xFF);
    sd_spi_xfer(0x40 | 24);
    sd_spi_xfer((uint8_t)(lba >> 24));
    sd_spi_xfer((uint8_t)(lba >> 16));
    sd_spi_xfer((uint8_t)(lba >> 8));
    sd_spi_xfer((uint8_t)lba);
    sd_spi_xfer(0x01);

    uint8_t r = 0xFF;
    for (uint8_t i = 0; i < 10; i++) {
        r = sd_spi_xfer(0xFF);
        if ((r & 0x80) == 0) break;
    }
    if (r != 0) {
        sd_cs(1);
        sd_spi_xfer(0xFF);
        r_sd_ok = STATUS_FAIL;
        return r;
    }

    sd_spi_xfer(0xFE);
    for (uint16_t i = 0; i < 512; i++) sd_spi_xfer(buf[i]);
    sd_spi_xfer(0xFF);
    sd_spi_xfer(0xFF);

    uint8_t resp = sd_spi_xfer(0xFF);
    if ((resp & 0x1F) != 0x05) {
        sd_cs(1);
        sd_spi_xfer(0xFF);
        r_sd_ok = STATUS_FAIL;
        return resp;
    }

    uint8_t ready = 0;
    for (uint16_t i = 0; i < ISSUE3_SD_WRITE_BUSY_POLLS; i++) {
        if (sd_spi_xfer(0xFF) == 0xFF) {
            ready = 1;
            break;
        }
    }
    sd_cs(1);
    sd_spi_xfer(0xFF);
    if (!ready) {
        r_sd_ok = STATUS_FAIL;
        return 0xFF;
    }
    return 0;
}

static void sd_async_cs_cb(void *ctx, uint8_t high)
{
    (void)ctx;
    sd_cs(high);
}

static uint8_t sd_async_xfer_cb(void *ctx, uint8_t out)
{
    (void)ctx;
    if (issue3_diag_active())
        issue3_diag_sat_inc(&r_issue3_diag.spi_xfer_bytes);
    return sd_spi_xfer(out);
}

static int sd_async_start_common(uint8_t op, uint32_t lba,
                                 uint8_t *read_buf,
                                 const uint8_t *write_buf)
{
    if (!issue3_spi_bus_try_acquire(&shared_spi_bus,
                                    ISSUE3_SPI_OWNER_SD)) return -1;
    uint32_t start_ms = runtime_now_ms();
    int result = issue3_sd_transfer_start(&sd_async, op, lba, read_buf,
                                          write_buf, sd_async_cs_cb,
                                          sd_async_xfer_cb, 0);
    if (result != 0) {
        issue3_spi_bus_release(&shared_spi_bus, ISSUE3_SPI_OWNER_SD);
    } else {
        issue3_diag_sd_transaction_begin(op, start_ms);
    }
    return result;
}

static int sd_async_start_read(uint32_t lba, uint8_t *buf)
{
    return sd_async_start_common(ISSUE3_SD_OP_READ, lba, buf, 0);
}

static int sd_async_start_write(uint32_t lba, const uint8_t *buf)
{
    return sd_async_start_common(ISSUE3_SD_OP_WRITE, lba, 0, buf);
}

/* Advance one bounded SD transaction step.  The command bytes and each
 * response/token/busy poll are individual steps.  Data is transferred in
 * chunks of at most ISSUE3_SD_STEP_BYTES, so no path below can spend a full
 * 512-byte sector (or the full busy timeout) before QMI/GZP/ICP run again. */
static int sd_async_step(void)
{
    uint32_t before_ms = runtime_now_ms();
    uint8_t was_active = sd_async.active;
    uint8_t before_phase = sd_async.phase;
    if (issue3_diag_active() && was_active)
        issue3_diag_runtime_sd_step(&r_issue3_diag, &diag_runtime,
                                    before_ms, before_phase);
    int result = issue3_sd_transfer_step(&sd_async, sd_async_cs_cb,
                                         sd_async_xfer_cb, 0);
    uint32_t after_ms = runtime_now_ms();
    if (issue3_diag_active() && was_active) {
        if (!sd_async.active) {
            issue3_diag_sd_transaction_finish(after_ms);
        } else if (sd_async.phase != before_phase) {
            issue3_diag_runtime_sd_phase_change(
                &r_issue3_diag, &diag_runtime, before_phase,
                sd_async.phase, after_ms);
        }
    }
    /* issue3_sd_transfer_finish() releases SD CS before setting
     * result_ready.  Only then may the NF service acquire the bus. */
    if (!sd_async.active && sd_async.result_ready)
        issue3_spi_bus_release(&shared_spi_bus, ISSUE3_SPI_OWNER_SD);
    return result;
}

static int sd_async_take_result(void)
{
    int result = issue3_sd_transfer_take_result(&sd_async);
    if (!sd_async.active && !sd_async.result_ready)
        issue3_spi_bus_release(&shared_spi_bus, ISSUE3_SPI_OWNER_SD);
    return result;
}

static uint8_t name_match_bin(const uint8_t *d)
{
    if (d[0] != 'L' || d[1] != 'O' || d[2] != 'G' || d[3] < '0' || d[3] > '9') return 0;
    if (d[4] < '0' || d[4] > '9' || d[5] < '0' || d[5] > '9' || d[6] < '0' || d[6] > '9') return 0;
    if (d[7] != ' ' || d[8] != 'B' || d[9] != 'I' || d[10] != 'N') return 0;
    return 1;
}

static uint16_t parse_log_num(const uint8_t *d)
{
    return ((d[3]-'0')*1000 + (d[4]-'0')*100 + (d[5]-'0')*10 + (d[6]-'0'));
}

static void make_log_name(uint16_t num, uint8_t *name11)
{
    name11[0]='L'; name11[1]='O'; name11[2]='G';
    name11[3] = '0' + (num / 1000) % 10;
    name11[4] = '0' + (num / 100) % 10;
    name11[5] = '0' + (num / 10) % 10;
    name11[6] = '0' + num % 10;
    name11[7]=' '; name11[8]='B'; name11[9]='I'; name11[10]='N';
}

static uint32_t g_fat0_lba, g_data_lba, g_root_lba;
static uint8_t g_spc, g_fats;
static uint32_t g_fat_size, g_root_cluster;

static uint8_t fat_init(void)
{
    uint32_t part_lba = 0;
    if (sd_read_sector(0, sd_buf) != 0) return 0xE1;
    if (sd_buf[510] != 0x55 || sd_buf[511] != 0xAA) return 0xE2;
    if (!(sd_buf[0] == 0xEB || sd_buf[0] == 0xE9)) part_lba = rd32(&sd_buf[454]);

    if (sd_read_sector(part_lba, sd_buf) != 0) return 0xE3;
    if (rd16(&sd_buf[11]) != 512) return 0xE4;

    g_spc = sd_buf[13];
    uint16_t reserved = rd16(&sd_buf[14]);
    g_fats = sd_buf[16];
    g_fat_size = rd32(&sd_buf[36]);
    g_root_cluster = rd32(&sd_buf[44]);
    if (g_spc == 0 || g_fats == 0 || g_fat_size == 0 || g_root_cluster < 2) return 0xE5;

    g_fat0_lba = part_lba + reserved;
    g_data_lba = g_fat0_lba + (uint32_t)g_fats * g_fat_size;
    g_root_lba = g_data_lba + (g_root_cluster - 2U) * g_spc;
    uint32_t total_sectors = rd32(&sd_buf[32]);
    if (total_sectors <= (g_data_lba - part_lba)) return 0xE6;
    g_data_clusters = (total_sectors - (g_data_lba - part_lba)) / g_spc;
    if (g_data_clusters == 0) return 0xE7;
    g_fat_scan.cursor = 2U;
    g_fat_scan.scanned_clusters = 0;
    g_fat_scan.cached_lba = 0U;
    g_fat_scan.complete = 0;
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    g_fat_last_result = ISSUE3_FAT_BUDGET;
    return 0;
}

static uint8_t fat_write(uint32_t cluster, uint32_t val)
{
    /* fat_write uses sd_buf rather than the scan scratch sector.  Any FAT
     * entry write may target the sector currently cached in log_meta_buf, so
     * never leave that cache live across this operation. */
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    uint32_t sec = g_fat0_lba + (cluster * 4U) / 512U;
    uint16_t off = (uint16_t)((cluster * 4U) & 511U);
    if (sd_read_sector(sec, sd_buf) != 0) return 1;
    wr32(&sd_buf[off], val);
    if (sd_write_sector(sec, sd_buf) != 0) return 1;
    if (g_fats > 1 && sd_write_sector(sec + g_fat_size, sd_buf) != 0) return 1;
    return 0;
}

static int fat_scan_read(void *ctx, uint32_t lba, uint8_t *buf)
{
    (void)ctx;
    return sd_read_sector(lba, buf);
}

static uint8_t fat_mark_cached(uint32_t cluster)
{
    uint32_t sec = g_fat0_lba + (cluster * 4U) / 512U;
    uint16_t off = (uint16_t)((cluster * 4U) & 511U);
    wr32(&log_meta_buf[off], 0x0FFFFFFFU);
    if (sd_write_sector(sec, log_meta_buf) != 0) {
        issue3_fat_scan_cache_invalidate(&g_fat_scan);
        return 1;
    }
    if (g_fats > 1 && sd_write_sector(sec + g_fat_size, log_meta_buf) != 0) {
        issue3_fat_scan_cache_invalidate(&g_fat_scan);
        return 1;
    }
    return 0;
}

static uint32_t fat_alloc(void)
{
    uint32_t cluster = 0;
    g_fat_last_result = issue3_fat_scan_step(
        &g_fat_scan, 2U, g_data_clusters, g_fat0_lba,
        FAT_SCAN_SECTOR_BUDGET, fat_scan_read, 0, log_meta_buf, &cluster);
    if (g_fat_last_result != ISSUE3_FAT_FOUND) return 0;
    if (fat_mark_cached(cluster) != 0) {
        g_fat_scan.cursor = cluster;
        g_fat_scan.scanned_clusters = 0;
        g_fat_scan.cache_valid = 0U;
        g_fat_last_result = ISSUE3_FAT_IO;
        return 0;
    }
    return cluster;
}

static int fat_alloc_callback(void *ctx, uint32_t *cluster)
{
    (void)ctx;
    *cluster = fat_alloc();
    return g_fat_last_result;
}

enum {
    FAT_RUNTIME_IDLE = 0,
    FAT_RUNTIME_READ = 1,
    FAT_RUNTIME_MARK_PRIMARY = 2,
    FAT_RUNTIME_MARK_SECONDARY = 3
};

/* Runtime allocator counterpart to fat_alloc_callback().  The startup path
 * may still use the synchronous helper before acquisition starts; once the
 * log is active, every FAT sector read/mark is issued through sd_async and
 * resumed on a later main-loop turn.  The in-memory scan of one 512-byte FAT
 * sector is bounded and does not perform any SPI transfer. */
static int log_runtime_prefetch_step(void)
{
    if (!log_prefetch.active || log_prefetch.next_ready)
        return ISSUE3_LOG_PREFETCH_IDLE;
    if (sd_async.active) return ISSUE3_LOG_PREFETCH_BUDGET;

    if (fat_runtime_phase != FAT_RUNTIME_IDLE && sd_async.result_ready) {
        if (sd_async_take_result() != 0) {
            issue3_fat_scan_cache_invalidate(&g_fat_scan);
            fat_runtime_phase = FAT_RUNTIME_IDLE;
            g_fat_last_result = ISSUE3_FAT_IO;
            log_prefetch.allocation_pending = 1U;
            return ISSUE3_LOG_PREFETCH_IO;
        }
        if (fat_runtime_phase == FAT_RUNTIME_READ) {
            g_fat_scan.cached_lba = fat_runtime_sector;
            g_fat_scan.cache_valid = 1U;
            fat_runtime_phase = FAT_RUNTIME_IDLE;
        } else if (fat_runtime_phase == FAT_RUNTIME_MARK_PRIMARY) {
            if (g_fats > 1U) {
                if (sd_async_start_write(fat_runtime_sector + g_fat_size,
                                         log_meta_buf) != 0)
                    return ISSUE3_LOG_PREFETCH_BUDGET;
                fat_runtime_phase = FAT_RUNTIME_MARK_SECONDARY;
                return ISSUE3_LOG_PREFETCH_BUDGET;
            }
            g_fat_last_result = ISSUE3_FAT_FOUND;
            log_prefetch.next_cluster = fat_runtime_cluster;
            log_prefetch.next_ready = 1U;
            log_prefetch.allocation_pending = 0U;
            fat_runtime_phase = FAT_RUNTIME_IDLE;
            return ISSUE3_LOG_PREFETCH_READY;
        } else if (fat_runtime_phase == FAT_RUNTIME_MARK_SECONDARY) {
            g_fat_last_result = ISSUE3_FAT_FOUND;
            log_prefetch.next_cluster = fat_runtime_cluster;
            log_prefetch.next_ready = 1U;
            log_prefetch.allocation_pending = 0U;
            fat_runtime_phase = FAT_RUNTIME_IDLE;
            return ISSUE3_LOG_PREFETCH_READY;
        }
    }
    if (fat_runtime_phase != FAT_RUNTIME_IDLE) return ISSUE3_LOG_PREFETCH_BUDGET;

    if (g_data_clusters == 0U) {
        g_fat_last_result = ISSUE3_FAT_FULL;
        log_prefetch.sd_full = 1U;
        return ISSUE3_LOG_PREFETCH_FULL;
    }
    uint32_t limit = 2U + g_data_clusters;
    if (g_fat_scan.cursor < 2U || g_fat_scan.cursor >= limit)
        g_fat_scan.cursor = 2U;
    uint32_t lba = g_fat0_lba + (g_fat_scan.cursor * 4U) / 512U;
    uint32_t sector_first = ((g_fat_scan.cursor * 4U) / 512U) * 128U;
    uint32_t begin = g_fat_scan.cursor > sector_first ?
                     g_fat_scan.cursor : sector_first;
    uint32_t end = sector_first + 128U;
    if (end > limit) end = limit;
    if (begin >= end) {
        g_fat_scan.cursor = 2U;
        g_fat_scan.scanned_clusters = 0U;
        return ISSUE3_LOG_PREFETCH_BUDGET;
    }
    if (!g_fat_scan.cache_valid || g_fat_scan.cached_lba != lba) {
        if (sd_async_start_read(lba, log_meta_buf) != 0)
            return ISSUE3_LOG_PREFETCH_BUDGET;
        fat_runtime_sector = lba;
        fat_runtime_phase = FAT_RUNTIME_READ;
        log_prefetch.allocation_pending = 1U;
        return ISSUE3_LOG_PREFETCH_BUDGET;
    }

    for (uint32_t cluster = begin; cluster < end; cluster++) {
        uint16_t off = (uint16_t)((cluster * 4U) & 511U);
        uint32_t value = rd32(&log_meta_buf[off]) & 0x0FFFFFFFU;
        if (value == 0U) {
            wr32(&log_meta_buf[off], 0x0FFFFFFFU);
            fat_runtime_cluster = cluster;
            fat_runtime_sector = lba;
            g_fat_scan.cursor = (cluster + 1U < limit) ?
                                 cluster + 1U : 2U;
            g_fat_scan.scanned_clusters = 0U;
            if (sd_async_start_write(lba, log_meta_buf) != 0)
                return ISSUE3_LOG_PREFETCH_BUDGET;
            fat_runtime_phase = FAT_RUNTIME_MARK_PRIMARY;
            log_prefetch.allocation_pending = 1U;
            return ISSUE3_LOG_PREFETCH_BUDGET;
        }
    }
    g_fat_scan.scanned_clusters += end - begin;
    g_fat_scan.cursor = end < limit ? end : 2U;
    if (g_fat_scan.scanned_clusters >= g_data_clusters) {
        g_fat_scan.complete = 1U;
        g_fat_last_result = ISSUE3_FAT_FULL;
        log_prefetch.sd_full = 1U;
        return ISSUE3_LOG_PREFETCH_FULL;
    }
    return ISSUE3_LOG_PREFETCH_BUDGET;
}

static int fat_alloc_startup(uint32_t *cluster)
{
    return issue3_fat_alloc_until_terminal(fat_alloc_callback, 0, cluster);
}

static uint8_t root_find_or_create_entry(const uint8_t *name11, uint16_t *out_ent,
                                         uint8_t *out_new)
{
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    if (sd_read_sector(g_root_lba, log_meta_buf) != 0) return 1;
    uint16_t ent = 0xFFFF;
    for (uint16_t off = 0; off < 512; off += 32) {
        uint8_t first = log_meta_buf[off];
        uint8_t match = 1;
        for (uint8_t i = 0; i < 11; i++) {
            if (log_meta_buf[off + i] != name11[i]) { match = 0; break; }
        }
        if (first != 0x00 && first != 0xE5 && match) {
            *out_ent = off;
            *out_new = 0;
            return 0;
        }
        if ((first == 0x00 || first == 0xE5) && ent == 0xFFFF) ent = off;
    }
    if (ent == 0xFFFF) return 1;
    for (uint8_t i = 0; i < 32; i++) log_meta_buf[ent + i] = 0;
    for (uint8_t i = 0; i < 11; i++) log_meta_buf[ent + i] = name11[i];
    log_meta_buf[ent + 11] = 0x20;
    if (sd_write_sector(g_root_lba, log_meta_buf) != 0) {
        issue3_fat_scan_cache_invalidate(&g_fat_scan);
        return 1;
    }
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    *out_ent = ent;
    *out_new = 1;
    return 0;
}

static uint16_t scan_max_log_num(void)
{
    if (sd_read_sector(g_root_lba, sd_buf) != 0) return 0;
    uint16_t maxnum = 0;
    for (uint16_t off = 0; off < 512; off += 32) {
        if (sd_buf[off] == 0x00) break;
        if (sd_buf[off] == 0xE5) continue;
        if (name_match_bin(&sd_buf[off])) {
            uint16_t n = parse_log_num(&sd_buf[off]);
            if (n > maxnum) maxnum = n;
        }
    }
    return maxnum;
}

static uint8_t log_create(uint16_t num, uint32_t *out_cluster)
{
    uint32_t clu = 0;
    int alloc_result = fat_alloc_startup(&clu);
    if (alloc_result != ISSUE3_FAT_FOUND) {
        if (alloc_result == ISSUE3_FAT_FULL) r_sd_full = 1;
        else if (alloc_result == ISSUE3_FAT_IO) r_sd_ok = STATUS_FAIL;
        return 1;
    }

    uint8_t name11[11];
    make_log_name(num, name11);
    uint16_t ent;
    uint8_t entry_new;
    if (root_find_or_create_entry(name11, &ent, &entry_new) != 0) {
        (void)fat_write(clu, 0U);
        return 1;
    }
    (void)entry_new;

    if (sd_read_sector(g_root_lba, log_meta_buf) != 0) {
        (void)fat_write(clu, 0U);
        return 1;
    }
    wr16(&log_meta_buf[ent + 20], (uint16_t)(clu >> 16));
    wr16(&log_meta_buf[ent + 26], (uint16_t)clu);
    wr32(&log_meta_buf[ent + 28], 0);
    if (sd_write_sector(g_root_lba, log_meta_buf) != 0) {
        (void)fat_write(clu, 0U);
        if (entry_new && sd_read_sector(g_root_lba, log_meta_buf) == 0) {
            for (uint8_t i = 0; i < 32U; i++) log_meta_buf[ent + i] = 0U;
            (void)sd_write_sector(g_root_lba, log_meta_buf);
        }
        return 1;
    }

    *out_cluster = clu;
    return 0;
}

static int fat_read_entry(uint32_t cluster, uint32_t *value)
{
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    uint32_t sec = g_fat0_lba + (cluster * 4U) / 512U;
    uint16_t off = (uint16_t)((cluster * 4U) & 511U);
    if (sd_read_sector(sec, log_meta_buf) != 0) return -1;
    *value = rd32(&log_meta_buf[off]) & 0x0FFFFFFFU;
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    return 0;
}

static int fat_free_chain(uint32_t first_cluster)
{
    uint32_t cluster = first_cluster;
    /* README is tiny, but bound traversal defensively so a corrupt FAT
     * cannot turn startup rollback into an unbounded loop. */
    for (uint32_t i = 0; i < 16U; i++) {
        if (cluster < 2U || cluster >= 2U + g_data_clusters) return 0;
        uint32_t next = 0;
        if (fat_read_entry(cluster, &next) != 0) return -1;
        if (fat_write(cluster, 0U) != 0) return -1;
        if (next < 2U || next >= 0x0FFFFFF8U) return 0;
        cluster = next;
    }
    return -1;
}

static int readme_read_root(void *ctx, uint8_t *sector)
{
    (void)ctx;
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    return sd_read_sector(g_root_lba, sector);
}

static int readme_write_root(void *ctx, const uint8_t *sector)
{
    (void)ctx;
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    int result = sd_write_sector(g_root_lba, sector);
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    return result;
}

static int readme_set_fat(void *ctx, uint32_t cluster, uint32_t value)
{
    (void)ctx;
    return fat_write(cluster, value);
}

static int readme_free_chain(void *ctx, uint32_t first_cluster)
{
    (void)ctx;
    return fat_free_chain(first_cluster);
}

static int readme_write_data(void *ctx, uint32_t cluster,
                             uint32_t sector_in_cluster,
                             const uint8_t *sector)
{
    (void)ctx;
    uint32_t lba = g_data_lba + (cluster - 2U) * g_spc + sector_in_cluster;
    return sd_write_sector(lba, sector);
}

static uint8_t write_readme(void)
{
    Issue3ReadmeStorage storage = {
        readme_read_root, readme_write_root, fat_alloc_callback,
        readme_set_fat, readme_free_chain, readme_write_data, 0,
        2U, g_data_clusters, g_spc
    };
    int result = issue3_readme_write_transaction(&storage, log_meta_buf,
                                                  sd_buf);
    if (result == ISSUE3_README_FULL) r_sd_full = 1U;
    else if (result == ISSUE3_README_IO ||
             result == ISSUE3_README_ROLLBACK_IO ||
             result == ISSUE3_README_COMMITTED_CLEANUP_IO)
        r_sd_ok = STATUS_FAIL;
    return result == ISSUE3_README_OK ? 0U : 1U;
}

static void wr16_buf(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(uint16_t)v;
    p[1] = (uint8_t)((uint16_t)v >> 8);
}

static void wr24_buf(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

static uint32_t frame_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(crc & 1U));
    }
    return ~crc;
}

/* Prepare a successor while the current cluster still has one sector left.
 * The scan itself is resumable and capped by FAT_SCAN_SECTOR_BUDGET; a
 * budget result is not an SD-full result and is retried on a later service. */
static void log_prepare_next_cluster(void)
{
    if (log_prefetch.sd_full || log_alloc_io) return;
    issue3_diag_operation_begin(&r_issue3_diag.fat_prefetch,
                                &diag_runtime.fat_active,
                                &diag_runtime.fat_start_ms);
    int result = log_runtime_prefetch_step();
    log_next_clu = log_prefetch.next_cluster;
    log_next_ready = log_prefetch.next_ready;
    if (result == ISSUE3_LOG_PREFETCH_FULL)
        g_fat_last_result = ISSUE3_FAT_FULL;
    else if (result == ISSUE3_LOG_PREFETCH_IO) {
        g_fat_last_result = ISSUE3_FAT_IO;
        log_alloc_io = 1U;
    }
    if (log_prefetch.next_ready || log_prefetch.sd_full || log_alloc_io)
        issue3_diag_operation_finish(&r_issue3_diag.fat_prefetch,
                                     &diag_runtime.fat_active,
                                     diag_runtime.fat_start_ms,
                                     runtime_now_ms());
}

static void log_stop_for_allocation(void)
{
    if (g_fat_last_result == ISSUE3_FAT_FULL) {
        /* A complete scan, unlike a budget result, is definitive. */
        /* Runtime allocation reaches this function immediately after a
         * completed sector/cluster boundary, so log_sec_off is normally zero.
         * Never re-enter the synchronous tail writer here: if a future path
         * leaves a partial sector, it remains private and the directory size
         * is capped at log_complete_size below. */
        r_sd_full = 1;
        log_active = 0;
        if (log_pending_active) {
            issue3_frame_queue_release_pending(&frame_queue, log_pending_slot);
            log_pending_active = 0;
        }
        if (!log_metadata_pending) {
            log_metadata_size = log_complete_size;
            log_metadata_pending = 1;
        }
    } else if (g_fat_last_result == ISSUE3_FAT_IO) {
        r_sd_ok = STATUS_FAIL;
        log_active = 0;
        if (log_pending_active) {
            issue3_frame_queue_release_pending(&frame_queue, log_pending_slot);
            log_pending_active = 0;
        }
        log_metadata_size = log_persisted_complete_size();
        log_metadata_pending = 1;
    } else {
        /* Keep the cursor and the active log alive while another bounded
         * allocator step runs.  This is not SD-full or an I/O failure. */
        r_sd_alloc_pending = 1;
    }
}

static void log_resume_after_allocation(void)
{
    if (!r_sd_alloc_pending || r_sd_full) return;
    /* The scan cursor is retained by issue3_fat_scan_step().  Retry one
     * bounded step per loop; a later free cluster resumes the same file
     * rather than leaving r_sd_alloc_pending as a permanent dead end. */
    log_prefetch.active = 1;
    log_prepare_next_cluster();
    if (!log_prefetch.next_ready) {
        if (g_fat_last_result == ISSUE3_FAT_FULL) {
            r_sd_alloc_pending = 0;
            log_stop_for_allocation();
        }
        else if (g_fat_last_result == ISSUE3_FAT_IO) {
            r_sd_alloc_pending = 0;
            r_sd_ok = STATUS_FAIL;
            log_active = 0;
            if (log_pending_active) {
                issue3_frame_queue_release_pending(&frame_queue,
                                                   log_pending_slot);
                log_pending_active = 0;
            }
            log_metadata_size = log_persisted_complete_size();
            log_metadata_pending = 1;
        }
        return;
    }
    log_link_pending = 1;
    log_link_stage = 0;
    log_link_cluster = log_clu;
    log_link_next = log_prefetch.next_cluster;
    log_link_sector = g_fat0_lba + (log_link_cluster * 4U) / 512U;
}

static void log_service_pending(void)
{
    if (!log_active || (!log_pending_active && !log_sector_write_pending) ||
        log_link_pending) return;

    if (log_sector_write_pending) {
        if (sd_async.active || !sd_async.result_ready) return;
        if (sd_async_take_result() != 0) {
            log_active = 0;
            issue3_frame_queue_release_pending(&frame_queue, log_pending_slot);
            log_pending_active = 0;
            log_sector_write_pending = 0U;
            r_sd_ok = STATUS_FAIL;
            log_metadata_size = log_persisted_complete_size();
            log_metadata_pending = 1U;
            return;
        }
        log_sector_write_pending = 0U;
        log_sector_buffer.offset = 0U;
        log_sec_off = 0U;
        total_written = log_sector_buffer.total_written + ISSUE3_SECTOR_BYTES;
        log_sector_buffer.total_written = total_written;
        log_last_progress_ms = runtime_now_ms();
        issue3_diag_sector_complete(log_last_progress_ms);
        log_lba++;

        int advance = issue3_log_prefetch_advance_sector(&log_prefetch);
        /* A frame can finish on the same physical sector that reaches a
         * cluster boundary.  Account for that complete frame before a FULL/
         * IO terminal path snapshots directory metadata. */
        if (log_pending_off >= FRAME_SIZE && log_pending_active) {
            issue3_frame_queue_release_pending(&frame_queue, log_pending_slot);
            log_pending_active = 0U;
            log_meta_frame_count++;
            log_complete_size = total_written;
            if (log_meta_frame_count >= 8U) {
                log_metadata_pending = 1U;
                log_metadata_size = log_persisted_complete_size();
                log_meta_frame_count = 0U;
            }
        }
        if (advance == ISSUE3_LOG_NEEDS_SUCCESSOR) {
            /* The background prefetch service gets the next turn; do not add
             * another SD operation to this write turn. */
            log_stop_for_allocation();
            return;
        }
        log_next_clu = log_prefetch.next_cluster;
        log_next_ready = log_prefetch.next_ready;
        if (log_prefetch.current_cluster != log_clu) {
            log_link_pending = 1U;
            log_link_stage = 0U;
            log_link_cluster = log_clu;
            log_link_next = log_prefetch.current_cluster;
            log_link_sector = g_fat0_lba + (log_link_cluster * 4U) / 512U;
        }
        r_log_written = total_written;
        return;
    }

    if (sd_async.active || sd_async.result_ready) return;
    uint16_t room = (uint16_t)(ISSUE3_SECTOR_BYTES -
                               log_sector_buffer.offset);
    uint16_t chunk = (uint16_t)(FRAME_SIZE - log_pending_off);
    if (chunk > room) chunk = room;
    for (uint16_t i = 0U; i < chunk; i++)
        sd_buf[log_sector_buffer.offset + i] =
            frame_slots[log_pending_slot].data[log_pending_off + i];
    log_sector_buffer.offset = (uint16_t)(log_sector_buffer.offset + chunk);
    log_sec_off = log_sector_buffer.offset;
    log_pending_off = (uint16_t)(log_pending_off + chunk);

    if (log_pending_off >= FRAME_SIZE && log_pending_active) {
        /* The source slot is released only after all V4 frame bytes have been
         * copied into the persistent sector buffer.  The SD transaction may
         * still be outstanding, but no producer can overwrite its bytes. */
        issue3_frame_queue_release_pending(&frame_queue, log_pending_slot);
        log_pending_active = 0U;
        log_meta_frame_count++;
        log_complete_size = total_written + log_sector_buffer.offset;
        if (log_meta_frame_count >= 8U) {
            log_metadata_pending = 1U;
            log_metadata_size = log_persisted_complete_size();
            log_meta_frame_count = 0U;
        }
    }
    if (log_sector_buffer.offset == ISSUE3_SECTOR_BYTES) {
        if (sd_async_start_write(log_lba, sd_buf) != 0) return;
        log_sector_write_pending = 1U;
    }
    r_log_written = total_written;
}

static void log_service_link(void)
{
    if (!log_link_pending) return;
    issue3_diag_operation_begin(&r_issue3_diag.fat_link,
                                &diag_runtime.link_active,
                                &diag_runtime.link_start_ms);
    /* The link operation uses sd_buf, not the FAT scan scratch buffer.  A
     * link can target the sector cached by the allocator, so force the next
     * allocator step to reread it. */
    issue3_fat_scan_cache_invalidate(&g_fat_scan);
    if (sd_async.active) return;
    if (sd_async.result_ready) {
        if (sd_async_take_result() != 0) {
            r_sd_ok = STATUS_FAIL;
            log_active = 0;
            if (log_pending_active) {
                issue3_frame_queue_release_pending(&frame_queue,
                                                   log_pending_slot);
                log_pending_active = 0;
            }
            log_link_pending = 0U;
            log_metadata_size = log_persisted_complete_size();
            log_metadata_pending = 1U;
            issue3_diag_operation_finish(&r_issue3_diag.fat_link,
                                         &diag_runtime.link_active,
                                         diag_runtime.link_start_ms,
                                         runtime_now_ms());
            return;
        }
        if (log_link_stage == 0U) {
            uint16_t off = (uint16_t)((log_link_cluster * 4U) & 511U);
            wr32(&sd_buf[off], log_link_next);
            log_link_stage = 1U;
        } else if (log_link_stage == 1U) {
            log_link_stage = 2U;
        } else {
            log_link_stage = 3U;
        }
    }
    if (log_link_stage == 0U) {
        if (sd_async_start_read(log_link_sector, sd_buf) != 0) return;
        return;
    }
    if (log_link_stage == 1U) {
        if (sd_async_start_write(log_link_sector, sd_buf) != 0) return;
        return;
    }
    if (log_link_stage == 2U && g_fats > 1U) {
        if (sd_async_start_write(log_link_sector + g_fat_size, sd_buf) != 0)
            return;
        return;
    }
    if (r_sd_alloc_pending) {
        (void)issue3_log_prefetch_link_ready(&log_prefetch);
        log_clu = log_prefetch.current_cluster;
        r_sd_alloc_pending = 0;
        log_active = 1;
    } else {
        log_clu = log_link_next;
        log_prefetch.current_cluster = log_clu;
        log_prefetch.next_cluster = 0;
        log_prefetch.next_ready = 0;
        log_prefetch.sector_index = 0;
    }
    log_lba = g_data_lba + (log_clu - 2U) * g_spc;
    log_next_clu = 0;
    log_next_ready = 0;
    log_link_pending = 0;
    log_link_stage = 0U;
    issue3_diag_operation_finish(&r_issue3_diag.fat_link,
                                 &diag_runtime.link_active,
                                 diag_runtime.link_start_ms,
                                 runtime_now_ms());
}

static void log_service_metadata(void)
{
    if (!log_metadata_pending) return;
    issue3_diag_operation_begin(&r_issue3_diag.metadata,
                                &diag_runtime.metadata_active,
                                &diag_runtime.metadata_start_ms);
    if (sd_async.active) return;
    if (sd_async.result_ready) {
        if (sd_async_take_result() != 0) {
            r_sd_ok = STATUS_FAIL;
            log_active = 0;
            log_metadata_pending = 0U;
            issue3_diag_operation_finish(&r_issue3_diag.metadata,
                                         &diag_runtime.metadata_active,
                                         diag_runtime.metadata_start_ms,
                                         runtime_now_ms());
            return;
        }
        if (log_metadata_stage == 0U) {
            uint8_t name11[11];
            make_log_name(log_num, name11);
            for (uint16_t off = 0U; off < 512U; off += 32U) {
                uint8_t match = 1U;
                if (log_meta_buf[off] == 0x00U) break;
                for (uint8_t i = 0U; i < 11U; i++)
                    if (log_meta_buf[off + i] != name11[i]) match = 0U;
                if (match) {
                    wr32(&log_meta_buf[off + 28U], log_metadata_size);
                    log_metadata_stage = 1U;
                    break;
                }
            }
            if (log_metadata_stage == 0U) {
                r_sd_ok = STATUS_FAIL;
                log_active = 0U;
                log_metadata_pending = 0U;
                issue3_diag_operation_finish(&r_issue3_diag.metadata,
                                             &diag_runtime.metadata_active,
                                             diag_runtime.metadata_start_ms,
                                             runtime_now_ms());
                return;
            }
        } else {
            log_metadata_pending = 0U;
            log_metadata_stage = 0U;
            log_metadata_size = 0U;
            issue3_diag_operation_finish(&r_issue3_diag.metadata,
                                         &diag_runtime.metadata_active,
                                         diag_runtime.metadata_start_ms,
                                         runtime_now_ms());
            return;
        }
    }
    if (log_metadata_stage == 0U) {
        issue3_fat_scan_cache_invalidate(&g_fat_scan);
        if (sd_async_start_read(g_root_lba, log_meta_buf) != 0) return;
        return;
    }
    if (sd_async_start_write(g_root_lba, log_meta_buf) != 0) return;
}

/* Return whether the next main-loop turn has a bounded SD/FAT/log step to
 * execute.  This is intentionally the same predicate used by the WFI
 * decision below: an active transaction, a completed result awaiting its
 * phase transition, or a resumable allocator/writer step keeps the CPU
 * running so the 8-byte SD slices are drained without waiting for TIM3. */
static uint8_t runtime_sd_work_pending(uint32_t now_ms)
{
    if (sd_async.active || sd_async.result_ready || log_link_pending ||
        r_sd_alloc_pending || fat_runtime_phase != FAT_RUNTIME_IDLE)
        return 1U;
    if (log_prefetch.active && !log_prefetch.next_ready &&
        log_prefetch.allocation_pending && !log_prefetch.sd_full &&
        !log_alloc_io)
        return 1U;
    if (log_active && (log_pending_active || log_sector_write_pending))
        return 1U;
    if (log_metadata_pending) return 1U;
    if (!frame_queue.pending_valid && frame_queue.count != 0U &&
        issue3_frame_commit_due(frame_queue.start_ms[frame_queue.head],
                                now_ms))
        return 1U;
    return 0U;
}

/* Rebuild every foreground and deadline condition while the sleep gate has
 * interrupts masked.  In addition to software flags, sample the peripheral
 * pending bits themselves: an ISR may have been delayed until after the
 * previous foreground turn, and those bits must never be cleared by the
 * compare-arm path before WFI. */
static void runtime_collect_work(void *ctx, Issue3RuntimeWork *work)
{
    (void)ctx;
    uint32_t now = runtime_now_ms();
    uint32_t nf_sequence = 0U;
    uint8_t nf_copy = 0U;
    uint8_t nf_ready = (uint8_t)(r_nf_due ||
        (!nf_heartbeat_schedule.in_flight &&
         time_reached(now, nf_heartbeat_schedule.next_ms)) ||
        issue3_nf_broadcast_due(&nf_broadcast, now, &nf_sequence, &nf_copy));
    uint8_t gzp_due = (uint8_t)(!gzp_pending &&
        gzp_results.count < ISSUE3_GZP_RESULT_QUEUE_CAPACITY &&
        time_reached(now, next_gzp_ms));
    uint8_t gzp_ready = (uint8_t)(gzp_due ||
        (gzp_pending && time_reached(now, gzp_started_ms +
                                     GZP_CONVERSION_WAIT_MS)) ||
        gzp_result_ready);
    uint8_t mts_ready = (uint8_t)(r_mts_due || time_reached(now, next_mts_ms) ||
        (mts_pending && time_reached(now, mts_started_ms + 50U)) ||
        mts_result_ready);
    uint8_t pending = 0U;
    uint32_t timer_status = TIM3->SR;
    if (timer_status & TIM_SR_CC1IF)
        pending |= ISSUE3_WAKE_PENDING_TIM3_COMPARE;
    if (timer_status & TIM_SR_UIF)
        pending |= ISSUE3_WAKE_PENDING_TIM3_UPDATE;
    if (EXTI->RPR1 & EXTI_QMI_PIN)
        pending |= ISSUE3_WAKE_PENDING_QMI_EXTI;

    *work = (Issue3RuntimeWork){
        sd_async.active,
        runtime_sd_work_pending(now),
        (uint8_t)(r_qmi_irq_pending || r_qmi_poll_due ||
                  (pending & ISSUE3_WAKE_PENDING_QMI_EXTI)),
        r_icp_poll_due,
        gzp_ready,
        gzp_result_ready,
        mts_ready,
        (uint8_t)(mts_result_ready ||
                  (mts_pending && time_reached(now, mts_started_ms + 50U))),
        nf_ready,
        0U,
        (uint8_t)((TIM3->DIER & TIM_DIER_UIE) != 0U),
        now,
        runtime_next_wake_deadline(now),
        pending
    };
}

static void runtime_irq_disable(void *ctx)
{
    (void)ctx;
    __asm volatile("cpsid i" ::: "memory");
}

static void runtime_irq_enable(void *ctx)
{
    (void)ctx;
    __asm volatile("cpsie i" ::: "memory");
    __asm volatile("isb" ::: "memory");
}

static void runtime_arm_wake_callback(void *ctx, uint32_t deadline_ms)
{
    (void)ctx;
    runtime_arm_wake_deadline(runtime_now_ms(), deadline_ms);
}

static void runtime_wfi_callback(void *ctx)
{
    (void)ctx;
    cpu_sleep();
}

static const Issue3RuntimeSleepOps runtime_sleep_ops = {
    runtime_irq_disable,
    runtime_irq_enable,
    runtime_collect_work,
    runtime_arm_wake_callback,
    0,
    runtime_wfi_callback
};

/* A healthy log must make durable progress, not merely keep the recording
 * flag set.  If a pending frame/FAT/metadata operation has made no complete
 * sector progress for several seconds, expose the failure through the
 * existing log_active heartbeat bit and stop accepting new log data.  The
 * timeout is well above the bounded SD transaction and FAT/link steps; it is
 * intended to catch a genuine scheduler/card stall, not normal bus latency. */
static void runtime_log_health_service(uint32_t now_ms)
{
    if (!log_active || log_write_stalled) return;
    /* Let the currently owned SD transaction reach its explicit completion
     * phase before changing log_active; otherwise the next metadata step
     * could mistake a data result for a root-directory result. */
    if (sd_async.active || sd_async.result_ready) return;
    if ((uint32_t)(now_ms - log_last_progress_ms) <=
        LOG_PROGRESS_TIMEOUT_MS)
        return;
    if (!log_pending_active && !log_sector_write_pending &&
        !log_link_pending && !log_metadata_pending &&
        !sd_async.active && !sd_async.result_ready &&
        frame_queue.count == 0U)
        return;
    log_write_stalled = 1U;
    log_active = 0U;
    r_sd_ok = STATUS_FAIL;
    log_metadata_size = log_persisted_complete_size();
    if (!log_metadata_pending) log_metadata_pending = 1U;
}

static void frame_slot_reset(FrameSlot *slot, uint32_t start_ms,
                             uint32_t sequence)
{
    for (uint16_t i = 0; i < FRAME_SIZE; i++) slot->data[i] = 0;
    slot->start_ms = start_ms;
    slot->qmi_count = 0;
    slot->gzp_count = 0;
    slot->icp_count = 0;
    slot->mts_valid = 0;
    slot->qmi_dropped = r_qmi_late_samples;
    slot->gzp_missed = r_gzp_missed_count;
    slot->status = frame_status;
    if (r_qmi_fifo_overflow) slot->status |= FRAME_STATUS_QMI_FIFO_OVERFLOW;
    /* Diagnostics are ownership-transferred to this newly opened frame.
     * Clearing the pending latch here prevents one old failure from marking
     * every later frame forever. */
    frame_status = 0;
    r_qmi_fifo_overflow = 0;
    r_qmi_late_samples = 0U;
    /* gzp_missed is a per-frame diagnostic, just like the other transferred
     * status fields.  Keeping the running count here made one old queue
     * stall permanently contaminate every later frame and eventually made
     * all frames report 0xFFFF. */
    r_gzp_missed_count = 0U;
    slot->valid = 1;
    wr32(&slot->data[0], FRAME_MAGIC);
    wr16(&slot->data[4], FRAME_VERSION);
    wr16(&slot->data[6], FRAME_HEADER_SIZE);
    wr32(&slot->data[8], sequence);
    wr32(&slot->data[12], start_ms);
}

static void frame_finalize_slot(FrameSlot *slot)
{
    if (!slot->valid) return;
    /* Reassert the container header at the ownership hand-off.  This also
     * protects a newly back-filled empty second from stale bytes in a
     * physical ring slot after wraparound. */
    wr32(&slot->data[0], FRAME_MAGIC);
    wr16(&slot->data[4], FRAME_VERSION);
    wr16(&slot->data[6], FRAME_HEADER_SIZE);
    if (slot->qmi_count == 0) slot->status |= FRAME_STATUS_QMI_FAIL;
    if (slot->gzp_count != GZP_N) slot->status |= FRAME_STATUS_GZP_FAIL;
    if (r_sd_full) slot->status |= FRAME_STATUS_SD_FULL;
    slot->data[16] = slot->qmi_count;
    slot->data[17] = slot->gzp_count;
    slot->data[18] = slot->icp_count;
    slot->data[19] = slot->mts_valid;
    wr32(&slot->data[20], slot->status);
    wr16(&slot->data[24], slot->qmi_dropped);
    wr16(&slot->data[26], r_qmi_fifo_overflow_count);
    wr16(&slot->data[28], qmi_clock.reanchor_count);
    wr16(&slot->data[30], slot->gzp_missed);
    wr32(&slot->data[FRAME_CRC_OFF], frame_crc32(slot->data, FRAME_DATA_SIZE));
    if (log_active && !log_pending_active && !log_link_pending &&
        !log_metadata_pending) {
        log_pending_slot = (uint8_t)(slot - frame_slots);
        log_pending_off = 0;
        log_pending_active = 1;
        issue3_frame_queue_claim_pending(&frame_queue, log_pending_slot);
    } else if (log_active) {
        /* The queue commit service is guarded so this is diagnostic-only;
         * it prevents silently overwriting a frame if that invariant fails. */
        frame_status |= FRAME_STATUS_QMI_FAIL;
    }
    slot->valid = 0;
}

static void frame_commit_callback(void *ctx, uint8_t slot_index,
                                  uint32_t frame_start_ms)
{
    (void)ctx;
    FrameSlot *slot = &frame_slots[slot_index];
    /* The queue owns the start timestamp; the slot is finalized exactly once
     * before the queue advances its head. */
    slot->start_ms = frame_start_ms;
    frame_finalize_slot(slot);
}

static uint8_t frame_queue_sd_work_pending(void)
{
    return (uint8_t)(sd_async.active || sd_async.result_ready ||
                     log_pending_active || log_sector_write_pending ||
                     log_link_pending || log_metadata_pending ||
                     r_sd_alloc_pending || fat_runtime_phase != FAT_RUNTIME_IDLE ||
                     log_alloc_io ||
                     (log_prefetch.active && !log_prefetch.next_ready &&
                      log_prefetch.allocation_pending && !log_prefetch.sd_full));
}

static FrameSlot *frame_slot_for(uint32_t timestamp_ms,
                                 Issue3FrameOpenReason *open_reason)
{
    uint32_t old_start[FRAME_QUEUE_N];
    uint8_t old_count = frame_queue.count;
    for (uint8_t i = 0; i < old_count; i++)
        old_start[i] = frame_queue.start_ms[
            issue3_frame_queue_index(&frame_queue, i)];

    uint8_t slot_index = 0, is_new = 0;
    int result = issue3_frame_queue_open_runtime_ex(
        &frame_queue, timestamp_ms, runtime_now_ms(),
        frame_queue_sd_work_pending(), frame_commit_callback, 0,
        &slot_index, &is_new, open_reason);
    if (result == ISSUE3_FRAME_OPEN_LATE ||
        result == ISSUE3_FRAME_OPEN_BLOCKED) return 0;
    if (is_new) {
        /* issue3_frame_queue_open() may fill several missing seconds in one
         * call.  Initialize every newly inserted physical slot, not only the
         * terminal slot that happened to match this sample. */
        for (uint8_t i = 0; i < frame_queue.count; i++) {
            uint8_t index = issue3_frame_queue_index(&frame_queue, i);
            uint8_t existed = 0;
            for (uint8_t j = 0; j < old_count; j++) {
                if (old_start[j] == frame_queue.start_ms[index]) {
                    existed = 1;
                    break;
                }
            }
            if (!existed)
                frame_slot_reset(&frame_slots[index],
                                 frame_queue.start_ms[index],
                                 issue3_frame_sequence_take(&r_frame_seq));
        }
    }
    return &frame_slots[slot_index];
}

static uint8_t frame_queue_commit_ready(uint32_t now_ms)
{
    if (log_pending_active || log_link_pending || log_metadata_pending) return 0;
    /* issue3_frame_commit_due() retains the frame's full one-second span,
     * the ceil(128 * 35.682 ms) FIFO age, 1 ms timestamp rounding, and the
     * established 1.82 s card-internal busy/metadata guard (wire time is
     * measured separately): 1000+4568+1+1920 = 7489 ms. */
    return issue3_frame_queue_commit_ready(&frame_queue, now_ms,
                                           frame_commit_callback, 0);
}

static uint8_t frame_add_qmi(uint32_t ts_ms, int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz)
{
    Issue3FrameOpenReason reason = ISSUE3_FRAME_OPEN_REASON_NONE;
    FrameSlot *slot = frame_slot_for(ts_ms, &reason);
    if (!slot) {
        frame_status |= FRAME_STATUS_QMI_FAIL;
        /* Only a timestamp outside the retained window is late.  OWNER and
         * QUEUE are real, separately diagnosed admission losses and must not
         * be mislabeled as a time reordering fault. */
        if (reason == ISSUE3_FRAME_OPEN_REASON_TIME)
            frame_status |= FRAME_STATUS_QMI_LATE_SAMPLE;
        if (r_qmi_late_samples != 0xFFFFU) r_qmi_late_samples++;
        issue3_diag_note_drop(ISSUE3_DIAG_SENSOR_QMI, reason);
        return 0U;
    }
    if (slot->qmi_count >= QMI_N) {
        slot->qmi_dropped++;
        slot->status |= FRAME_STATUS_QMI_FAIL;
        issue3_diag_note_drop(ISSUE3_DIAG_SENSOR_QMI,
                              ISSUE3_FRAME_OPEN_REASON_QUEUE);
        return 0U;
    }
    uint8_t *p = &slot->data[FRAME_QMI_OFF + slot->qmi_count * 16U];
    wr32(p, ts_ms); p += 4;
    wr16_buf(p, ax); p += 2; wr16_buf(p, ay); p += 2; wr16_buf(p, az); p += 2;
    wr16_buf(p, gx); p += 2; wr16_buf(p, gy); p += 2; wr16_buf(p, gz);
    slot->qmi_count++;
    if (issue3_diag_active()) issue3_diag_sat_inc(&r_issue3_diag.qmi_admitted);
    issue3_diag_observe_depths();
    return 1U;
}

static uint8_t frame_add_gzp(uint32_t ts_ms, uint32_t pressure_raw,
                             int16_t temperature_raw)
{
    Issue3FrameOpenReason reason = ISSUE3_FRAME_OPEN_REASON_NONE;
    FrameSlot *slot = frame_slot_for(ts_ms, &reason);
    if (!slot) {
        frame_status |= FRAME_STATUS_GZP_FAIL;
        issue3_diag_note_drop(ISSUE3_DIAG_SENSOR_GZP, reason);
        return 0U;
    }
    if (slot->gzp_count >= GZP_N) {
        slot->status |= FRAME_STATUS_GZP_FAIL;
        issue3_diag_note_drop(ISSUE3_DIAG_SENSOR_GZP,
                              ISSUE3_FRAME_OPEN_REASON_QUEUE);
        return 0U;
    }
    uint8_t *p = &slot->data[FRAME_GZP_OFF + slot->gzp_count * 10U];
    wr32(p, ts_ms); p += 4;
    wr24_buf(p, (int32_t)pressure_raw); p += 3;
    wr16_buf(p, temperature_raw);
    slot->gzp_count++;
    if (issue3_diag_active()) issue3_diag_sat_inc(&r_issue3_diag.gzp_admitted);
    issue3_diag_observe_depths();
    return 1U;
}

static uint8_t frame_add_icp(uint32_t ts_ms, int32_t p_raw, int32_t t_raw)
{
    Issue3FrameOpenReason reason = ISSUE3_FRAME_OPEN_REASON_NONE;
    FrameSlot *slot = frame_slot_for(ts_ms, &reason);
    if (!slot) {
        frame_status |= FRAME_STATUS_ICP_FAIL;
        issue3_diag_note_drop(ISSUE3_DIAG_SENSOR_ICP, reason);
        return 0U;
    }
    if (slot->icp_count >= ICP_N) {
        slot->status |= FRAME_STATUS_ICP_FAIL;
        issue3_diag_note_drop(ISSUE3_DIAG_SENSOR_ICP,
                              ISSUE3_FRAME_OPEN_REASON_QUEUE);
        return 0U;
    }
    uint8_t *p = &slot->data[FRAME_ICP_OFF + slot->icp_count * 10U];
    wr32(p, ts_ms); p += 4;
    wr24_buf(p, p_raw); p += 3;
    wr24_buf(p, t_raw);
    slot->icp_count++;
    if (issue3_diag_active()) issue3_diag_sat_inc(&r_issue3_diag.icp_admitted);
    issue3_diag_observe_depths();
    return 1U;
}

static uint8_t frame_add_mts(uint32_t ts_ms, int16_t temp_raw)
{
    FrameSlot *slot = frame_slot_for(ts_ms, 0);
    if (!slot) return 0;
    wr32(&slot->data[FRAME_MTS_OFF], ts_ms);
    wr16_buf(&slot->data[FRAME_MTS_OFF + 4], temp_raw);
    slot->mts_valid = 1;
    return 1;
}

/* Bind the shared recovery executor to the production I2C implementation.
 * Host tests provide the same callbacks against a controlled register map. */
static int qmi8658_recovery_write_cb(void *ctx, uint8_t reg, uint8_t value)
{
    (void)ctx;
    return qmi8658_write_reg(reg, value);
}

static int qmi8658_recovery_read_cb(void *ctx, uint8_t reg, uint8_t *value)
{
    (void)ctx;
    return i2c_read(0x6A, reg, value, 1);
}

static int qmi8658_recovery_ctrl9_cb(void *ctx, uint8_t command)
{
    (void)ctx;
    return qmi8658_ctrl9(command);
}

static const Issue3QmiRecoveryIo qmi_recovery_io = {
    0,
    qmi8658_recovery_write_cb,
    qmi8658_recovery_read_cb,
    qmi8658_recovery_ctrl9_cb
};

static int qmi_fifo_cleanup(void)
{
    uint8_t fifo_ctrl = 0;
    if (qmi8658_write_reg(QMI_REG_FIFO_CTRL, QMI_FIFO_CTRL_CONFIG) == 0 &&
        i2c_read(0x6A, QMI_REG_FIFO_CTRL, &fifo_ctrl, 1) == 0 &&
        !(fifo_ctrl & ISSUE3_QMI_FIFO_CTRL_RD_MODE)) return 0;
    frame_status |= FRAME_STATUS_QMI_FIFO_CLEANUP;
    issue3_qmi_clock_invalidate(&qmi_clock);
    /* Do not turn one transient cleanup error into a reset on every 100 ms
     * poll.  The stalled-stream watchdog owns the complete bounded reset;
     * this best-effort mode clear only records the failure and lets the next
     * service turn retry the normal path. */
    return -1;
}

/* A FIFO_DATA transfer can fail after only part of a batch has been
 * consumed.  Clearing FIFO_RD_MODE alone is insufficient: the unread FIFO
 * level may keep the watermark pin high, while EXTI is rising-edge only.
 * Reset the FIFO, restore the stream/watermark configuration, and verify that
 * read mode is cleared so a later batch can generate a fresh edge. */
static int qmi_fifo_reset_stream_after_loss(void)
{
    frame_status |= FRAME_STATUS_QMI_FIFO_CLEANUP;
    issue3_qmi_clock_invalidate(&qmi_clock);
    /* The shared executor owns the documented command/configuration order and
     * verifies every readable stream register plus the full empty status. */
    return issue3_qmi_recovery_execute(&qmi_recovery_io);
}

static void qmi_fifo_record_recovery_result(int result)
{
    if (result == 0) {
        frame_status = issue3_qmi_recovery_status_or(
            frame_status, 1U, 0U);
    } else {
        frame_status = issue3_qmi_recovery_status_or(
            frame_status, 0U, 1U);
        frame_status |= FRAME_STATUS_QMI_FAIL;
    }
}

/* A stalled-stream attempt is diagnosed independently of the FIFO status
 * that happened to be observed on this turn.  The caller has already armed
 * the one-second retry gate in Issue3QmiRecovery. */
static int qmi_fifo_attempt_recovery(void)
{
    frame_status |= FRAME_STATUS_QMI_EMPTY_RECOVERY;
    r_qmi_ok = STATUS_FAIL;
    int result = qmi_fifo_reset_stream_after_loss();
    qmi_fifo_record_recovery_result(result);
    return result;
}

static void qmi_fifo_service(void)
{
    /* Commit only before CTRL9 enters FIFO read mode; SD I/O must not happen
     * while the sensor is in the mode where new samples are discarded. */
    frame_queue_commit_ready(runtime_now_ms());
    /* A finalized slot may be pending in SD while older FIFO samples still
     * belong to retained seconds.  FIFO admission is deliberately per raw
     * sample below: an old or SD-owned point is dropped with its exact frame
     * reason, while later samples in this same hardware batch continue to be
     * consumed.  A whole-batch writable preflight would let the FIFO age
     * through an SD write and cause repeated 128-sample resets. */
    uint32_t service_now_ms = runtime_now_ms();
    uint8_t recovery_due = issue3_qmi_recovery_service_tick(
        &qmi_recovery, service_now_ms);
    /* Once the no-sample deadline has expired, reset before interpreting a
     * possibly stale/non-empty FIFO.  This makes every stalled-stream cause
     * share the same bounded recovery and guarantees the old QMI clock is
     * invalid before the first post-gap batch is accepted. */
    if (recovery_due) {
        (void)qmi_fifo_attempt_recovery();
        return;
    }
    uint8_t count_lsb = 0, status = 0;
    if (i2c_read(0x6A, ISSUE3_QMI_REG_FIFO_COUNT_LSB, &count_lsb, 1) < 0 ||
        i2c_read(0x6A, ISSUE3_QMI_REG_FIFO_STATUS, &status, 1) < 0) {
        frame_status |= FRAME_STATUS_QMI_FAIL;
        issue3_qmi_clock_invalidate(&qmi_clock);
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        /* The caller has consumed this trigger.  TIM3 will issue the next
         * independent poll, so an I2C failure cannot turn the foreground
         * loop into an unbounded retry. */
        return;
    }
    Issue3QmiFifoObservation fifo_observation =
        issue3_qmi_recovery_observe_fifo(&qmi_recovery, count_lsb, status,
                                         service_now_ms, &recovery_due);
    if (fifo_observation == ISSUE3_QMI_FIFO_OVERFLOW) {
        r_qmi_fifo_overflow = 1;
        r_qmi_fifo_overflow_count++;
        frame_status |= FRAME_STATUS_QMI_FIFO_OVERFLOW;
        /* The FIFO contents are no longer a continuous batch.  Count the
         * unread native samples as dropped before the reset so V4 diagnostics
         * distinguish an overflow loss from a clean re-anchor. */
        uint16_t overflow_words = (uint16_t)(((status & 0x03U) << 8) |
                                             count_lsb);
        uint16_t overflow_samples = (uint16_t)(overflow_words / 6U);
        if (overflow_samples == 0U) overflow_samples = 128U;
        if ((uint32_t)r_qmi_late_samples + overflow_samples > 0xFFFFU)
            r_qmi_late_samples = 0xFFFFU;
        else
            r_qmi_late_samples = (uint16_t)(r_qmi_late_samples +
                                            overflow_samples);
        issue3_qmi_clock_invalidate(&qmi_clock);
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }
    if (fifo_observation == ISSUE3_QMI_FIFO_INCONSISTENT) {
        /* Count/status are independent reads.  A transient contradiction is
         * a diagnosed protocol observation, not a valid empty interval and
         * not a reason to reset a possibly healthy FIFO immediately. */
        frame_status |= FRAME_STATUS_QMI_FAIL;
        r_qmi_ok = STATUS_FAIL;
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }
    uint16_t words = issue3_qmi_fifo_words(count_lsb, status);
    uint16_t bytes = (uint16_t)(words * 2U);
    if ((bytes % 12U) != 0U) {
        frame_status |= FRAME_STATUS_QMI_FAIL;
        issue3_qmi_clock_invalidate(&qmi_clock);
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }
    uint16_t samples = (uint16_t)(bytes / 12U);
    if (samples == 0) {
        /* A legal empty state with no stalled-stream deadline simply waits
         * for the next 100 ms service turn. */
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }
    if (samples > 128U) {
        frame_status |= FRAME_STATUS_QMI_FAIL;
        issue3_qmi_clock_invalidate(&qmi_clock);
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }
    uint64_t batch_now_us = (uint64_t)runtime_now_ms() * 1000ULL;

    /* Rev D section 8.8: CTRL_CMD_REQ_FIFO must complete before FIFO_DATA
     * is read.  The command enables read mode; clearing FIFO_CTRL bit7 below
     * releases the FIFO and re-enables filling. */
    if (qmi8658_ctrl9(QMI_CMD_REQ_FIFO) < 0) {
        frame_status |= FRAME_STATUS_QMI_FAIL;
        issue3_qmi_clock_invalidate(&qmi_clock);
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }

    /* Correct a successfully readable but stale software clock before the
     * first sample is admitted.  This is independent of the 500 ms no-sample
     * watchdog: the hardware FIFO may keep returning valid data while the
     * old free-running clock falls behind the current batch window. */
    (void)issue3_qmi_clock_prepare_batch(&qmi_clock, batch_now_us, samples);
    uint8_t batch_complete = 1;
    for (uint16_t i = 0; i < samples; i++) {
        uint8_t raw[12];
        if (i2c_read(0x6A, 0x17, raw, sizeof(raw)) < 0) {
            frame_status |= FRAME_STATUS_QMI_FAIL;
            issue3_qmi_clock_invalidate(&qmi_clock);
            uint16_t lost = (uint16_t)(samples - i);
            if ((uint32_t)r_qmi_late_samples + lost > 0xFFFFU)
                r_qmi_late_samples = 0xFFFFU;
            else
                r_qmi_late_samples = (uint16_t)(r_qmi_late_samples + lost);
            batch_complete = 0;
            break;
        }
        /* qmi_received counts only raw FIFO samples that were actually read.
         * A frame admission failure is handled below on this one sample and
         * must never prevent the rest of the batch from being consumed. */
        if (issue3_diag_active())
            issue3_diag_sat_inc(&r_issue3_diag.qmi_received);
        uint32_t sample_ms = issue3_qmi_clock_sample_ms(&qmi_clock);
        int16_t ax = to_signed16(raw[1], raw[0]);
        int16_t ay = to_signed16(raw[3], raw[2]);
        int16_t az = to_signed16(raw[5], raw[4]);
        int16_t gx = to_signed16(raw[7], raw[6]);
        int16_t gy = to_signed16(raw[9], raw[8]);
        int16_t gz = to_signed16(raw[11], raw[10]);
        r_ax = ax; r_ay = ay; r_az = az; r_gx = gx; r_gy = gy; r_gz = gz;
        r_qmi_ok = STATUS_OK;
        qmi_sample_seen = 1U;
        qmi_last_sample_ms = runtime_now_ms();
        issue3_qmi_recovery_note_sample(&qmi_recovery, qmi_last_sample_ms);

        /* The FIFO sample has already been read and the native clock has
         * advanced above.  Keep every other successful raw sample for V4;
         * the phase is deliberately not reset at FIFO, frame or re-anchor
         * boundaries.  An intentional half-rate omission is not a failure or
         * a diagnostic drop. */
        if (issue3_decimation_keep_next(&qmi_decimation_phase))
            frame_add_qmi(sample_ms, ax, ay, az, gx, gy, gz);
    }
    if (!batch_complete) {
        /* Do not merely clear read mode: reset the unread FIFO so a high
         * watermark cannot strand the edge-triggered EXTI path.  Samples
         * already committed above stay exactly once; the reset discards only
         * the unread suffix and leaves gap_pending visible for re-anchoring. */
        /* Exit read mode if possible, but defer the complete reset until the
         * shared 500 ms no-sample watchdog is due.  This bounds repeated
         * FIFO_DATA failures instead of resetting on every 100 ms poll. */
        (void)qmi_fifo_cleanup();
        if (recovery_due) (void)qmi_fifo_attempt_recovery();
        return;
    }
    if (qmi_clock.gap_pending) frame_status |= FRAME_STATUS_QMI_REANCHORED;
    if (qmi_fifo_cleanup() < 0) {
        frame_status |= FRAME_STATUS_QMI_FAIL;
        issue3_qmi_clock_invalidate(&qmi_clock);
    } else if (qmi_clock.gap_pending) {
        /* The complete batch is now represented; subsequent clean batches
         * continue from this anchor without re-anchoring. */
        issue3_qmi_clock_complete_batch(&qmi_clock);
    }
}

void Reset_Handler(void)
{
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) { *dst++ = *src++; }
    dst = &_sbss;
    while (dst < &_ebss) { *dst++ = 0; }

    systick_init();
    delay_ms(10);

    bb_i2c_prepare();
    delay_ms(10);

    int imu_ok = qmi8658_init();
    r_qmi_ok = (imu_ok == 0) ? STATUS_OK : STATUS_FAIL;
    uint8_t mts4_who;
    int mts4_ok = i2c_read(0x41, 0x01, &mts4_who, 1);
    r_mts4_ok = (mts4_ok == 0) ? STATUS_OK : STATUS_FAIL;
    int icp_ok = icp20100_init();
    r_icp_ok = (icp_ok == 0) ? STATUS_OK : STATUS_FAIL;

    r_nf_ok = STATUS_FAIL;
    r_nf_init_ok = STATUS_FAIL;
    r_sd_ok = STATUS_FAIL;
    r_sd_init_ok = STATUS_FAIL;
    spi_shared_gpio_init();
    nf03_init();
    r_magic = 0xDEADBEEF;
    sd_card_prepare();
    sd_probe();

    oled_startup_check(imu_ok, mts4_ok, icp_ok);

    /* Discard startup FIFO contents and begin the formal run from a clean
     * sensor time anchor. */
    (void)qmi8658_ctrl9(0x04);
    (void)qmi8658_write_reg(0x14, 0x0EU);
    qmi_clock.valid = 0;
    qmi_clock.gap_pending = 0;
    qmi_clock.last_sample_valid = 0;
    qmi_clock.reanchor_count = 0;
    qmi_clock.clock_us = 0;
    qmi_clock.last_sample_us = 0;
    r_qmi_irq_pending = 0U;
    r_qmi_poll_due = 0U;
    r_icp_poll_due = 0U;
    r_icp_late_samples = 0U;
    icp20100_start_mode3();

    log_active = 0;
    r_frame_seq = 0U;
    log_clu = 0;
    log_num = 0;
    log_lba = 0;
    log_sec_off = 0;
    total_written = 0;
    log_meta_frame_count = 0;
    issue3_frame_queue_init(&frame_queue);
    r_sd_full = 0;
    r_sd_alloc_pending = 0;
    log_next_clu = 0;
    log_next_ready = 0;
    log_prefetch.active = 0;
    log_prefetch.next_ready = 0;
    log_prefetch.allocation_pending = 0;
    log_prefetch.sd_full = 0;
    log_prefetch.current_cluster = 0;
    log_prefetch.next_cluster = 0;
    log_prefetch.sectors_per_cluster = 0;
    log_prefetch.sector_index = 0;
    log_pending_off = 0;
    log_pending_slot = 0;
    log_pending_active = 0;
    log_complete_size = 0;
    log_last_progress_ms = 0U;
    log_write_stalled = 0U;
    issue3_gzp_result_queue_init(&gzp_results);
    gzp_result_ready = 0;
    mts_result_ready = 0;
    mts_result_ms = 0;
    mts_result_temp = 0;
    log_metadata_pending = 0;
    log_metadata_size = 0;
    log_link_pending = 0;
    log_link_stage = 0;
    log_link_cluster = 0;
    log_link_next = 0;
    log_link_sector = 0;
    log_sector_buffer.offset = 0;
    log_sector_buffer.total_written = 0;
    sd_async.active = 0U;
    sd_async.result_ready = 0U;
    sd_async.result = 0U;
    log_sector_write_pending = 0U;
    log_metadata_stage = 0U;
    fat_runtime_phase = FAT_RUNTIME_IDLE;
    fat_runtime_cluster = 0U;
    fat_runtime_sector = 0U;
    log_alloc_io = 0U;

    if (r_sd_ok == STATUS_OK) {
        if (fat_init() == 0) {
            if (write_readme() != 0) {
                r_sd_ok = STATUS_FAIL;
            } else {
                uint16_t maxn = scan_max_log_num();
                log_num = maxn + 1;
                if (log_create(log_num, &log_clu) == 0) {
                    log_lba = g_data_lba + (log_clu - 2U) * g_spc;
                    log_next_clu = 0;
                    log_next_ready = 0;
                    log_prefetch.active = 1;
                    log_prefetch.allocation_pending = 1U;
                    log_prefetch.current_cluster = log_clu;
                    log_prefetch.sectors_per_cluster = g_spc;
                    log_active = 1;
                    log_last_progress_ms = 0U;
                    log_write_stalled = 0U;
                }
            }
        }
    }
    r_log_num = log_num;

    /* The runtime timer establishes the absolute origin for all periodic
     * work.  Send exactly one startup heartbeat after sensor/SD/log init has
     * produced its results; the shared schedule's next_ms remains
     * NF_HEARTBEAT_PERIOD_MS, so the following packet is on the fixed
     * 30-minute grid
     * rather than 30 minutes after TX. */
    runtime_timer_init();
    issue3_qmi_recovery_init(&qmi_recovery, runtime_now_ms());
    issue3_diag_runtime_init();
    /* Startup checks use SysTick time, which is stopped before the formal
     * runtime origin is zeroed.  Rebase their successful-read markers so the
     * first heartbeat does not look stale merely because the clock changed. */
    qmi_last_sample_ms = 0U;
    gzp_last_sample_ms = 0U;
    icp_last_sample_ms = 0U;
    mts4_last_sample_ms = 0U;
    nf_broadcast.active = 0;
    nf_broadcast.next_copy = 0;
    uint32_t startup_sequence = 0;
    if (issue3_heartbeat_startup(&nf_heartbeat_schedule, &startup_sequence)) {
        /* Begin at the actual startup-send time.  The first copy is due at
         * offset zero and is serviced before entering the main loop. */
        (void)issue3_nf_broadcast_begin(&nf_broadcast,
                                        nf03_device_id_value(),
                                        startup_sequence, runtime_now_ms());
        nf03_broadcast_service(runtime_now_ms());
    }
    while (1) {
        uint32_t now = runtime_now_ms();
        issue3_diag_sat_inc(&r_issue3_diag.main_loop_count);
        r_issue3_diag.runtime_ms = now;
        issue3_diag_observe_depths();

        /* Copy 2/3 are delayed by deterministic offsets, not by a blocking
         * 30-second wait.  All sensor, FIFO and SD work continues between
         * these calls. */
        if (!sd_async.active) nf03_broadcast_service(now);

        if (gzp_pending &&
            (uint32_t)(now - gzp_started_ms) >= GZP_CONVERSION_WAIT_MS) {
            int result = gzp6816d_finish();
            if (result == 0) {
                r_gzp_ok = STATUS_OK;
                if (issue3_diag_active())
                    issue3_diag_sat_inc(&r_issue3_diag.gzp_received);
                gzp_sample_seen = 1U;
                gzp_last_sample_ms = now;
                /* The conversion/result is still acquired on every 100 ms
                 * absolute grid point.  Only after a successful raw read do
                 * we apply the persistent two-phase decimator for V4.  An
                 * intentional omission never enters the result queue and is
                 * not a missed/drop/failure; the next conversion remains on
                 * the same absolute schedule. */
                if (issue3_decimation_keep_next(&gzp_decimation_phase) &&
                    !issue3_gzp_result_queue_push(&gzp_results, gzp_plan_ms,
                                                  r_gzp_p_raw, r_gzp_t_raw)) {
                    frame_status |= FRAME_STATUS_GZP_FAIL;
                    gzp_note_missed(1U);
                    if (issue3_diag_active())
                        issue3_diag_sat_inc(&r_issue3_diag.gzp_drop_queue);
                }
                gzp_result_ready = gzp_results.count != 0U;
                gzp_pending = 0;
            } else if (result < 0 || (uint32_t)(now - gzp_started_ms) >= GZP_CONVERSION_TIMEOUT_MS) {
                r_gzp_ok = STATUS_FAIL;
                r_gzp_p_raw = 0xFFFFFFU;
                r_gzp_t_raw = 0x7FFF;
                frame_status |= FRAME_STATUS_GZP_FAIL;
                gzp_pending = 0;
                gzp_note_missed(1);
                if (issue3_diag_active())
                    issue3_diag_sat_inc(&r_issue3_diag.gzp_drop_time);
            }
        }

        if (gzp_result_ready) {
            const Issue3GzpResult *result = issue3_gzp_result_queue_peek(
                &gzp_results);
            /* A result whose second is already retained in the SD-owned
             * physical slot (or has fallen before the retained head) can
             * never become writable.  Drop exactly that old point now so it
             * cannot hold the queue head forever; a future point that merely
             * waits for capacity remains queued for retry. */
            if (result && issue3_frame_queue_timestamp_unrecoverable(
                              &frame_queue, result->timestamp_ms)) {
                frame_status |= FRAME_STATUS_GZP_FAIL;
                gzp_note_missed(1U);
                issue3_diag_note_drop(
                    ISSUE3_DIAG_SENSOR_GZP,
                    issue3_frame_queue_timestamp_reason(
                        &frame_queue, result->timestamp_ms));
                issue3_gzp_result_queue_pop(&gzp_results);
                gzp_result_ready = gzp_results.count != 0U;
            } else if (result && gzp_result_can_enter_frame(result->timestamp_ms) &&
                frame_add_gzp(result->timestamp_ms, result->pressure_raw,
                              result->temperature_raw)) {
                issue3_gzp_result_queue_pop(&gzp_results);
                gzp_result_ready = gzp_results.count != 0U;
            } else if (result && time_reached(
                           now, result->timestamp_ms +
                                GZP_RESULT_RETENTION_MS)) {
                /* A result that stayed behind an uncommittable queue for a
                 * complete retention horizon is one real lost measurement,
                 * not a reason to keep the conversion state permanently
                 * pending.  The next dispatch resumes the absolute 100 ms
                 * grid and never replays this historical point. */
                frame_status |= FRAME_STATUS_GZP_FAIL;
                gzp_note_missed(1U);
                issue3_diag_note_drop(
                    ISSUE3_DIAG_SENSOR_GZP,
                    issue3_frame_queue_timestamp_reason(
                        &frame_queue, result->timestamp_ms));
                issue3_gzp_result_queue_pop(&gzp_results);
                gzp_result_ready = gzp_results.count != 0U;
            }
        }

        /* A conversion is only blocked by a full result queue.  Results that
         * are merely waiting for a frame slot do not advance the schedule;
         * the bounded queue absorbs the SD transaction latency. */
        if (!gzp_pending &&
            gzp_results.count < ISSUE3_GZP_RESULT_QUEUE_CAPACITY) {
            /* Up to 10 ms late is still the current 100 ms point: the
             * 0xB4 conversion is nominally about 19 ms, leaving margin.  A
             * larger delay is handled by the shared planner as historical
             * misses plus one conversion stamped at its actual start time;
             * no overdue conversion is replayed back-to-back. */
            Issue3GzpSchedule schedule = {next_gzp_ms, gzp_plan_ms};
            uint32_t plan_ms = 0;
            int decision = issue3_gzp_dispatch(&schedule, now,
                                               gzp_start_callback, 0,
                                               gzp_note_callback, 0,
                                               &plan_ms);
            next_gzp_ms = schedule.next_ms;
            if (decision == ISSUE3_GZP_SKIP) {
                /* issue3_gzp_dispatch() has already recorded all skipped
                 * grid points, including older points before the candidate. */
            } else if (decision == ISSUE3_GZP_START) {
                gzp_pending = 1;
                gzp_plan_ms = plan_ms;
                gzp_started_ms = now;
            } else if (decision == ISSUE3_GZP_START_FAILED) {
                r_gzp_ok = STATUS_FAIL;
                frame_status |= FRAME_STATUS_GZP_FAIL;
            }
        }

        if (mts_pending && (uint32_t)(now - mts_started_ms) >= 50U) {
            float body_temp = 0.0f;
            int result = mts4_finish(&body_temp);
            if (result == 0) {
                r_mts4_ok = STATUS_OK;
                mts4_sample_seen = 1U;
                mts4_last_sample_ms = now;
                r_mts4_temp = (int16_t)(body_temp * 256.0f);
                mts_result_ms = now;
                mts_result_temp = r_mts4_temp;
                mts_result_ready = 1;
                mts_pending = 0;
            } else if (result < 0 || (uint32_t)(now - mts_started_ms) > 5000U) {
                r_mts4_ok = STATUS_FAIL;
                frame_status |= FRAME_STATUS_MTS_FAIL;
                mts_pending = 0;
            }
        }
        if (mts_result_ready &&
            frame_add_mts(mts_result_ms, mts_result_temp)) {
            mts_result_ready = 0;
        }
        if (r_mts_due && !mts_pending && !mts_result_ready) {
            r_mts_due = 0;
            while (time_reached(now, next_mts_ms)) next_mts_ms += MTS4_PERIOD_MS;
            if (mts4_start() == 0) {
                mts_pending = 1;
                mts_started_ms = now;
            } else {
                r_mts4_ok = STATUS_FAIL;
                frame_status |= FRAME_STATUS_MTS_FAIL;
            }
        }

        if (r_nf_due) {
            r_nf_due = 0;
            uint32_t periodic_sequence = 0;
            if (issue3_heartbeat_periodic_due(&nf_heartbeat_schedule, now,
                                              &periodic_sequence)) {
                if (!nf_broadcast.active) {
                    (void)issue3_nf_broadcast_begin(
                        &nf_broadcast, nf03_device_id_value(),
                        periodic_sequence, now);
                    /* The service owns NF only after the shared SPI arbiter
                     * grants it; if SD is active this call merely leaves the
                     * copy pending for the next loop. */
                    nf03_broadcast_service(now);
                } else {
                    /* A due period is retried on the next service turn if a
                     * prior broadcast is still in flight; no second seq is
                     * reserved before the prior three copies complete. */
                }
            }
        }

        /* QMI has the shortest hardware FIFO horizon.  PB6/EXTI remains the
         * fast path, while the independent TIM3 poll closes the case where a
         * watermark edge was missed or never arrived.  Consume each trigger
         * before entering I2C; a failed transaction is retried by the next
         * 100 ms tick instead of spinning in the foreground. */
        if (r_qmi_irq_pending || r_qmi_poll_due) {
            r_qmi_irq_pending = 0U;
            r_qmi_poll_due = 0U;
            qmi_fifo_service();
        }
        /* ICP has no interrupt line.  Its hardware FIFO is serviced at most
         * once per TIM3 100 ms period, immediately before the next bounded
         * SD step when a transaction is active. */
        if (r_icp_poll_due) {
            r_icp_poll_due = 0U;
            icp20100_service();
        }
        now = runtime_now_ms();
        /* QMI has priority.  At most one bounded SD/FAT state-machine step
         * runs after it: link, frame sector, metadata, frame commit, then
         * successor prefetch. */
        if (sd_async.active) {
            /* Keep the SD CS/SPI transaction alive for one bounded step;
             * sensor services above have already run for this turn. */
            (void)sd_async_step();
        } else if (log_link_pending) {
            log_service_link();
        } else if (r_sd_alloc_pending) {
            log_resume_after_allocation();
        } else if (fat_runtime_phase != FAT_RUNTIME_IDLE ||
                   (log_prefetch.active && !log_prefetch.next_ready &&
                    log_prefetch.allocation_pending && !log_prefetch.sd_full &&
                    !log_alloc_io)) {
            /* Keep the successor scan moving even while a frame is waiting;
             * one resumable FAT/read/mark step is enough for this turn. */
            log_prepare_next_cluster();
        } else if ((log_pending_active || log_sector_write_pending) &&
                   log_active) {
            log_service_pending();
        } else if (log_metadata_pending) {
            log_service_metadata();
        } else if (frame_queue_commit_ready(now) == 0) {
            if (r_sd_alloc_pending) log_resume_after_allocation();
            else if (log_active && !log_prefetch.next_ready &&
                     !log_prefetch.sd_full && !log_alloc_io)
                log_prepare_next_cluster();
        }

        /* A successful sector completion is the health heartbeat for the
         * logger.  If the foreground has been unable to make progress for a
         * real timeout, make the existing log_active status report the fault
         * instead of continuing to advertise a healthy recording. */
        now = runtime_now_ms();
        runtime_log_health_service(now);

        /* This is the only runtime WFI entrance.  It masks interrupts before
         * rebuilding all work/deadline state, arms the compare without
         * clearing a newly pending flag, executes DSB;WFI, then restores
         * interrupts so a masked pending ISR is serviced immediately. */
        /* refresh() fills every field while interrupts are masked; avoid a
         * libc memset in the freestanding firmware image. */
        Issue3RuntimeWork work;
        (void)issue3_runtime_sleep_entry(&work, &runtime_sleep_ops, 0);
    }
}

void Default_Handler(void) { while (1); }
