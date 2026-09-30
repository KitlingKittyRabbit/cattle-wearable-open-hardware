from pathlib import Path
import socket
import subprocess
import time


FIRMWARE_DIR = Path(__file__).resolve().parent

subprocess.run(["killall", "openocd"], capture_output=True)
time.sleep(1)
ocd = subprocess.Popen(["openocd", "-f", str(FIRMWARE_DIR / "openocd.cfg")],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(2)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 3333))

def gdb_cmd(cmd):
    pkt = f"${cmd}#"
    checksum = sum(ord(c) for c in cmd) % 256
    pkt = f"${cmd}#{checksum:02x}"
    s.send(pkt.encode())
    time.sleep(0.3)
    return s.recv(4096).decode(errors='ignore')

def gdb_read(addr, fmt, size):
    gdb_cmd(f"m {addr},{size}")
    time.sleep(0.2)
    return s.recv(4096)

gdb_cmd("Hg 0")
time.sleep(0.5)
gdb_cmd("c")
time.sleep(1)
gdb_cmd("\x03")
time.sleep(0.5)

for addr, size in [(0x20001F00, 32)]:
    resp = gdb_cmd(f"m {addr:x},{size}")
    print(f"Read 0x{addr:08X}: {resp}")

s.close()
ocd.terminate()
