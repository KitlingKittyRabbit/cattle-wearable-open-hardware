#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* This is a host-side layout regression, not a fake peripheral test.  The
 * offsets and line encoding are copied from the STM32G030 CMSIS definition;
 * compiling the production header here prevents a compact legacy EXTI struct
 * from silently returning. */
#include "../固件/源码/stm32g030.h"

int main(void)
{
    assert(EXTI_BASE == 0x40021800UL);
    assert(offsetof(EXTI_TypeDef, RTSR1) == 0x00U);
    assert(offsetof(EXTI_TypeDef, FTSR1) == 0x04U);
    assert(offsetof(EXTI_TypeDef, SWIER1) == 0x08U);
    assert(offsetof(EXTI_TypeDef, RPR1) == 0x0cU);
    assert(offsetof(EXTI_TypeDef, FPR1) == 0x10U);
    assert(offsetof(EXTI_TypeDef, EXTICR) == 0x60U);
    assert(offsetof(EXTI_TypeDef, IMR1) == 0x80U);
    assert(offsetof(EXTI_TypeDef, EMR1) == 0x84U);
    assert(sizeof(EXTI_TypeDef) == 0x88U);
    assert(offsetof(SYSCFG_TypeDef, CFGR1) == 0x00U);
    assert(offsetof(SYSCFG_TypeDef, CFGR2) == 0x18U);
    assert(offsetof(SYSCFG_TypeDef, IT_LINE_SR) == 0x80U);
    assert(sizeof(SYSCFG_TypeDef) == 0x100U);

    /* PB6 uses EXTI line 6, GPIOB is selector 1, and the IRQ is EXTI4_15. */
    assert(STM32G030_QMI_EXTI_PIN == 0x40U);
    assert(STM32G030_EXTI4_15_IRQ_BIT == 0x80U);
    assert(STM32G030_QMI_EXTICR_INDEX == 1U);
    assert(STM32G030_QMI_EXTICR_SHIFT == 16U);
    assert(STM32G030_GPIOB_EXTI_SELECT == 1U);
    puts("STM32G030 EXTI layout and PB6 encoding: passed");
    return 0;
}
