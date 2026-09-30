#!/bin/bash
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
FIRMWARE_ELF="$SCRIPT_DIR/编译输出/firmware.elf"

killall openocd 2>/dev/null
sleep 1
openocd -f "$SCRIPT_DIR/openocd.cfg" &>/dev/null &
sleep 2

gdb-multiarch \
  -ex "set architecture arm" \
  -ex "target remote :3333" \
  -ex "continue" \
  "$FIRMWARE_ELF" &
GDB_PID=$!
sleep 3
kill -INT $GDB_PID 2>/dev/null
wait $GDB_PID 2>/dev/null

gdb-multiarch -batch \
  -ex "set architecture arm" \
  -ex "target remote :3333" \
  -ex "printf \"magic=0x%08X\\n\", *(volatile uint32_t*)0x20000010" \
  -ex "printf \"qmi_ok=%d mts4_ok=%d icp_ok=%d\\n\", *(volatile uint8_t*)0x20000018, *(volatile uint8_t*)0x20000014, *(volatile uint8_t*)0x2000000C" \
  -ex "printf \"ax=%d ay=%d az=%d\\n\", *(volatile int16_t*)0x20000000, *(volatile int16_t*)0x20000002, *(volatile int16_t*)0x20000004" \
  -ex "printf \"gx=%d gy=%d gz=%d\\n\", *(volatile int16_t*)0x20000006, *(volatile int16_t*)0x20000008, *(volatile int16_t*)0x2000000A" \
  -ex "printf \"mts4_temp_x256=%d\\n\", *(volatile int16_t*)0x20000016" \
  -ex "quit" "$FIRMWARE_ELF" 2>&1 | grep -E "^(magic|qmi|ax|gx|mts4)"

kill %1 2>/dev/null
