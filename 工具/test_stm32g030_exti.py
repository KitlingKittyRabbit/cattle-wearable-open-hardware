#!/usr/bin/env python3
"""验证 G030 EXTI 布局并核对最终 ELF 的中断向量槽位。"""
import pathlib
import struct
import subprocess
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[1]
ELF = ROOT / "固件" / "编译输出" / "firmware.elf"
HEADER_TEST = ROOT / "工具" / "test_stm32g030_exti.c"


def symbol(name):
    rows = subprocess.check_output(
        ["arm-none-eabi-nm", "-n", str(ELF)], text=True
    ).splitlines()
    for row in rows:
        parts = row.split()
        if len(parts) >= 3 and parts[2] == name:
            return int(parts[0], 16)
    raise AssertionError(f"missing ELF symbol: {name}")


def main():
    with tempfile.TemporaryDirectory() as td:
        exe = pathlib.Path(td) / "exti_layout"
        subprocess.check_call(
            ["cc", "-std=c11", "-Wall", "-Wextra", str(HEADER_TEST), "-o", str(exe)]
        )
        subprocess.check_call([str(exe)])
        vector = pathlib.Path(td) / "vector.bin"
        subprocess.check_call(
            ["arm-none-eabi-objcopy", "--dump-section", f".isr_vector={vector}", str(ELF)]
        )
        data = vector.read_bytes()

    words = struct.unpack("<" + "I" * (len(data) // 4), data)
    exti = symbol("EXTI4_15_IRQHandler") | 1
    tim3 = symbol("TIM3_IRQHandler") | 1
    assert words[16 + 7] == exti, (hex(words[23]), hex(exti))
    assert words[16 + 16] == tim3, (hex(words[32]), hex(tim3))
    print("STM32G030 EXTI 布局、PB6 映射和 IRQ 向量：通过")


if __name__ == "__main__":
    main()
