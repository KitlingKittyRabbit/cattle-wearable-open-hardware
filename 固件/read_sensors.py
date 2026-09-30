#!/usr/bin/env python3
import os
from pathlib import Path
import signal
import subprocess
import time


FIRMWARE_DIR = Path(__file__).resolve().parent
FIRMWARE_ELF = FIRMWARE_DIR / '编译输出' / 'firmware.elf'

subprocess.run(["killall", "openocd", "gdb-multiarch", "st-util"], capture_output=True)
time.sleep(1)

ocd = subprocess.Popen(["openocd", "-f", str(FIRMWARE_DIR / "openocd.cfg")],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2)

gdb = subprocess.Popen(
    ['gdb-multiarch', str(FIRMWARE_ELF)],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
def cmd(s):
    gdb.stdin.write((s + '\n').encode())
    gdb.stdin.flush()
    time.sleep(0.3)

def read_until_prompt(timeout=2):
    import select
    data = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        ready, _, _ = select.select([gdb.stdout], [], [], 0.1)
        if ready:
            chunk = gdb.stdout.read1(4096) if hasattr(gdb.stdout, 'read1') else os.read(gdb.stdout.fileno(), 4096)
            data += chunk
            if b'(gdb)' in data:
                break
    return data.decode(errors='replace')

cmd('set architecture arm')
cmd('target remote :3333')
cmd('set pagination off')
read_until_prompt()

# Resume MCU and let it run for a while
cmd('continue')
time.sleep(4)  # Let sensors initialize and collect data

# Send Ctrl-C to stop
os.kill(gdb.pid, signal.SIGINT)
time.sleep(1)
read_until_prompt()

# Now read the variables
cmd('x/1xw 0x20000010')  # magic
cmd('x/3xh 0x20000000')  # ax, ay, az
cmd('x/3xh 0x20000006')  # gx, gy, gz
cmd('x/1xh 0x20000016')  # mts4_temp
cmd('x/3xb 0x20000018')  # qmi_ok, mts4_ok, icp_ok
cmd('x/1xw 0x4002101C')  # RCC_APBENR1
cmd('x/1xw 0x48000400')  # GPIOA_MODER
time.sleep(1)
cmd('quit')

out = gdb.stdout.read().decode(errors='replace')
for line in out.splitlines():
    line = line.strip().replace('(gdb) ', '')
    if '0x2' in line or '0x4' in line:
        print(line)

gdb.wait()
ocd.terminate()
