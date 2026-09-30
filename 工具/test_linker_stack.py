#!/usr/bin/env python3
"""链接期 RAM/栈保留检查，并验证静态数据侵占栈保留区会 fail closed。"""
import pathlib
import subprocess
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[1]
ELF = ROOT / "固件" / "编译输出" / "firmware.elf"
OBJ = ROOT / "固件" / "编译输出" / "main.o"
SCRIPT = ROOT / "固件" / "链接脚本" / "STM32G030K8T6.ld"
MAIN = ROOT / "固件" / "源码" / "main.c"


def symbols(path):
    output = subprocess.check_output(["arm-none-eabi-nm", "-g", str(path)], text=True)
    result = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                result[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    return result


def main():
    syms = symbols(ELF)
    assert syms["__stack_reserve"] == 576
    assert syms["_estack"] - syms["_ebss"] >= syms["__stack_reserve"]
    assert syms["_ebss"] <= syms["__ram_data_limit"]

    bad_script = SCRIPT.read_text().replace(
        "__stack_reserve = 576;", "__stack_reserve = 3200;")
    with tempfile.TemporaryDirectory() as directory:
        bad_path = pathlib.Path(directory) / "bad.ld"
        bad_path.write_text(bad_script)
        result = subprocess.run(
            ["arm-none-eabi-gcc", "-mcpu=cortex-m0plus", "-mthumb",
             "-T", str(bad_path), "-Wl,--gc-sections", "-nostartfiles",
             "-nostdlib", str(OBJ), "-lgcc", "-o", str(pathlib.Path(directory) / "bad.elf")],
            text=True, capture_output=True)
        assert result.returncode != 0
        assert "static RAM exceeds reserved stack" in (result.stdout + result.stderr)

    # Every production WFI must sit behind one of the two shared atomic
    # entries.  Keep this source-level contract alongside the ELF stack check:
    # it prevents a future short timer helper from quietly reintroducing the
    # old `while (!done) cpu_sleep()` race.
    source = MAIN.read_text(encoding="utf-8")
    short_start = source.index("static void short_sleep_ms")
    short_end = source.index("void TIM3_IRQHandler", short_start)
    short_body = source[short_start:short_end]
    assert "issue3_atomic_wait_step" in short_body
    assert "issue3_atomic_wait_begin" in short_body
    assert "short_wait_pending" in short_body or "short_wait_ops" in short_body
    assert "while (!r_short_timer_done)" not in short_body
    start_body = source[source.index("static void short_wait_start"):short_start]
    order = [
        "TIM3->DIER &= ~TIM_DIER_CC1IE",
        "TIM3->SR &= ~TIM_SR_CC1IF",
        "r_short_timer_done = 0U",
        "TIM3->CNT",
        "TIM3->CCR1",
        "TIM3->DIER |= TIM_DIER_CC1IE",
    ]
    positions = [start_body.index(token) for token in order]
    assert positions == sorted(positions)
    assert "issue3_short_timer_target" in start_body
    assert source.count('asm volatile("wfi"') == 1
    assert "issue3_runtime_sleep_entry(&work, &runtime_sleep_ops" in source
    assert "r_qmi_poll_due = 1U" in source
    assert "r_icp_poll_due = 1U" in source
    main_start = source.index("while (1)")
    main_loop = source[main_start:source.index("void Default_Handler", main_start)]
    assert "if (r_qmi_irq_pending || r_qmi_poll_due)" in main_loop
    assert "if (r_icp_poll_due)" in main_loop
    reset_definition = source.index("void Reset_Handler(void)\n{")
    qmi_body = source[source.index("static void qmi_fifo_service"):
                       reset_definition]
    assert "r_qmi_irq_pending" not in qmi_body
    assert "issue3_decimation_keep_next(&qmi_decimation_phase)" in qmi_body
    assert "if (issue3_decimation_keep_next(&gzp_decimation_phase)" in source
    assert "#define FRAME_VERSION ISSUE3_FRAME_VERSION" in source
    assert "#define FRAME_SIZE ISSUE3_FRAME_SIZE" in source
    print("链接期栈保留检查及负向测试：通过")


if __name__ == "__main__":
    main()
