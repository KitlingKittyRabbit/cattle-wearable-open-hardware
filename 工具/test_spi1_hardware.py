"""Static/ELF guardrails for the production STM32G030 SPI1 migration."""

from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "固件" / "源码" / "stm32g030.h"
MAIN = ROOT / "固件" / "源码" / "main.c"
ELF = ROOT / "固件" / "编译输出" / "firmware.elf"


def test_spi1_register_map_and_clock_profiles():
    text = HEADER.read_text(encoding="utf-8")
    assert "#define SPI1_BASE       (APBPERIPH_BASE + 0x00013000UL)" in text
    assert "#define RCC_APBENR2_SPI1EN (1U << 12)" in text
    assert "#define SPI_CR1_MSTR        (1U << 2)" in text
    assert "#define SPI_CR2_DS_Pos      8U" in text
    assert "#define SPI_SR_BSY          (1U << 7)" in text

    main = MAIN.read_text(encoding="utf-8")
    assert "SPI1->CR1" in main and "SPI1->CR2" in main
    assert "spi_hw_set_rate(ISSUE3_SD_SPI_INIT_BR)" in main
    assert "spi_hw_set_rate(ISSUE3_SD_SPI_RUNTIME_BR)" in main
    assert "GPIOA->BSRR = SD_CS_PIN | NF_CS_PIN;" in main
    assert "GPIOA->AFRL &= ~((0xFU << 20) | (0xFU << 24) | (0xFU << 28));" in main
    assert "RCC->APBENR2 |= RCC_APBENR2_SPI1EN;" in main


def test_spi_transfer_has_bounded_wait_and_no_bit_bang_loop():
    main = MAIN.read_text(encoding="utf-8")
    match = re.search(
        r"static uint8_t sd_spi_xfer\(uint8_t out\)\n\{(?P<body>.*?)\n\}",
        main,
        re.S,
    )
    assert match, "生产 SD/NF 传输入口缺失"
    body = match.group("body")
    assert "spi_hw_xfer(out)" in body
    assert "GPIOA->BSRR = SD_SCK_PIN" not in body
    assert "for (uint8_t mask = 0x80" not in body
    assert "ISSUE3_SD_SPI_WAIT_LIMIT_LOOPS" in main
    assert "spi_hw_timeout()" in main


def test_clean_build_contains_spi1_path():
    assert ELF.exists(), "请先运行唯一正式固件 clean build"
    listing = subprocess.run(
        ["arm-none-eabi-objdump", "-d", str(ELF)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    # A GPIO bit-banging implementation would contain a long per-bit loop;
    # the production transfer path must instead reference the SPI1 register
    # block at 0x40013000 and its bounded timeout code.
    assert "40013000" in listing or "4001300" in listing
    assert "sd_delay" not in listing
