#ifndef STM32G030_H
#define STM32G030_H

#define PERIPH_BASE     0x40000000UL
#define APBPERIPH_BASE  PERIPH_BASE
#define AHBPERIPH_BASE  (PERIPH_BASE + 0x00020000UL)

#define RCC_BASE        (AHBPERIPH_BASE + 0x00001000UL)
#define GPIOA_BASE      0x50000000UL
#define GPIOB_BASE      0x50000400UL

#define I2C1_BASE       (APBPERIPH_BASE + 0x00005400UL)
#define SPI1_BASE       (APBPERIPH_BASE + 0x00013000UL)
#define USART1_BASE     (APBPERIPH_BASE + 0x00004400UL)
#define USART2_BASE     (APBPERIPH_BASE + 0x00004800UL)
#define ADC1_BASE       (APBPERIPH_BASE + 0x0000C400UL)
#define DMA1_BASE       (AHBPERIPH_BASE + 0x00006000UL)
#define TIM3_BASE       (APBPERIPH_BASE + 0x00000400UL)
#define SYSCFG_BASE     (APBPERIPH_BASE + 0x00010000UL)
#define EXTI_BASE       (APBPERIPH_BASE + 0x00021800UL)

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
    volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t PLLSAICFGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t CIER;
    volatile uint32_t CIFR;
    volatile uint32_t CICR;
    volatile uint32_t IOPRSTR;
    volatile uint32_t AHBRSTR;
    volatile uint32_t APBRSTR1;
    volatile uint32_t APBRSTR2;
    volatile uint32_t IOPENR;
    volatile uint32_t AHBENR;
    volatile uint32_t APBENR1;
    volatile uint32_t APBENR2;
    volatile uint32_t IOPSMENR;
    volatile uint32_t AHBSMENR;
    volatile uint32_t APBSMENR1;
    volatile uint32_t APBSMENR2;
    volatile uint32_t CCIPR;
    volatile uint32_t RESERVED0;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t CRRCR;
    volatile uint32_t CCIPR2;
} RCC_TypeDef;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RESERVED0;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} TIM_TypeDef;

/* STM32G030 SPI1 uses the legacy SPI register block (not the newer G4/H7
 * CFG1/CFG2 layout).  Keeping the exact offsets here is important: the
 * original software-SPI implementation never touched this peripheral, so a
 * wrong base/offset would silently write an unrelated APB register. */
typedef struct {
    volatile uint32_t CR1;       /* 0x00 */
    volatile uint32_t CR2;       /* 0x04 */
    volatile uint32_t SR;        /* 0x08 */
    volatile uint32_t DR;        /* 0x0c */
    volatile uint32_t CRCPR;     /* 0x10 */
    volatile uint32_t RXCRCR;    /* 0x14 */
    volatile uint32_t TXCRCR;    /* 0x18 */
    volatile uint32_t I2SCFGR;   /* 0x1c */
    volatile uint32_t I2SPR;     /* 0x20 */
} SPI_TypeDef;

/* STM32G030 CMSIS layout.  Unlike the STM32F0-style layout, G030 puts the
 * EXTI trigger/pending registers first, the four EXTICR registers at 0x60,
 * and the interrupt masks at 0x80.  Keeping the reserved holes is critical:
 * writing a pending bit through a compact/legacy struct hits a reserved
 * address and HardFaults on real silicon. */
typedef struct {
    volatile uint32_t CFGR1;          /* 0x00 */
    volatile uint32_t RESERVED0[5];   /* 0x04..0x14 */
    volatile uint32_t CFGR2;          /* 0x18 */
    volatile uint32_t RESERVED1[25];  /* 0x1c..0x7c */
    volatile uint32_t IT_LINE_SR[32]; /* 0x80..0xfc */
} SYSCFG_TypeDef;

typedef struct {
    volatile uint32_t RTSR1;          /* 0x00 */
    volatile uint32_t FTSR1;          /* 0x04 */
    volatile uint32_t SWIER1;         /* 0x08 */
    volatile uint32_t RPR1;           /* 0x0c, write 1 to clear */
    volatile uint32_t FPR1;           /* 0x10, write 1 to clear */
    volatile uint32_t RESERVED1[3];   /* 0x14..0x1c */
    volatile uint32_t RESERVED2[5];   /* 0x20..0x30 */
    volatile uint32_t RESERVED3[11];  /* 0x34..0x5c */
    volatile uint32_t EXTICR[4];      /* 0x60..0x6c */
    volatile uint32_t RESERVED4[4];   /* 0x70..0x7c */
    volatile uint32_t IMR1;           /* 0x80 */
    volatile uint32_t EMR1;           /* 0x84 */
} EXTI_TypeDef;

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define RCC   ((RCC_TypeDef *)RCC_BASE)
#define SPI1  ((SPI_TypeDef *)SPI1_BASE)
#define TIM3  ((TIM_TypeDef *)TIM3_BASE)
#define SYSCFG ((SYSCFG_TypeDef *)SYSCFG_BASE)
#define EXTI  ((EXTI_TypeDef *)EXTI_BASE)

/* STM32G030K8T6 interrupt routing used by the QMI PB6 watermark. */
#define STM32G030_EXTI4_15_IRQ_BIT   (1U << 7)
#define STM32G030_QMI_EXTI_LINE      6U
#define STM32G030_QMI_EXTI_PIN       (1U << STM32G030_QMI_EXTI_LINE)
#define STM32G030_QMI_EXTICR_INDEX   1U
#define STM32G030_QMI_EXTICR_SHIFT   16U
#define STM32G030_GPIOB_EXTI_SELECT  1U

#define RCC_IOPENR_GPIOAEN  (1U << 0)
#define RCC_IOPENR_GPIOBEN  (1U << 1)
#define RCC_APBENR2_SPI1EN (1U << 12)
#define RCC_APBENR1_TIM3EN  (1U << 1)
#define RCC_APBENR2_SYSCFGEN (1U << 0)

#define TIM_CR1_CEN         (1U << 0)
#define TIM_DIER_UIE        (1U << 0)
#define TIM_DIER_CC1IE      (1U << 1)
#define TIM_SR_UIF          (1U << 0)
#define TIM_SR_CC1IF       (1U << 1)
#define TIM_EGR_UG          (1U << 0)

/* SPI1, master, software-managed NSS, MSB-first, Motorola mode 0.  The
 * peripheral clock is the reset HSI16/PCLK (16 MHz).  BR=5 gives 250 kHz for
 * SD card identification and BR=2 gives 2 MHz for runtime SD/NF traffic. */
#define SPI_CR1_MSTR        (1U << 2)
#define SPI_CR1_BR_Pos      3U
#define SPI_CR1_BR_Msk      (7U << SPI_CR1_BR_Pos)
#define SPI_CR1_SPE         (1U << 6)
#define SPI_CR1_SSI         (1U << 8)
#define SPI_CR1_SSM         (1U << 9)
#define SPI_CR2_DS_Pos      8U
#define SPI_CR2_DS_Msk      (0xFU << SPI_CR2_DS_Pos)
#define SPI_CR2_FRXTH       (1U << 12)
#define SPI_SR_RXNE         (1U << 0)
#define SPI_SR_TXE          (1U << 1)
#define SPI_SR_OVR          (1U << 6)
#define SPI_SR_BSY          (1U << 7)
#define SPI_I2SCFGR_I2SMOD  (1U << 11)

#endif
