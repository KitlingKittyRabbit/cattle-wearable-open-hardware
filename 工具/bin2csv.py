#!/usr/bin/env python3
"""Convert PCB32 binary logs to CSV with fail-closed frame parsing.

V1 is the legacy fixed 1722-byte format (LOG1 magic, no per-sample time).
V2/V3 are the historical 674-byte LOG2 formats.  V4 is the current 368-byte
LOG2 format with post-read QMI/GZP half-rate capacities.  Frames are consumed
strictly at their boundaries; payload bytes are never searched for a
replacement magic.
"""
import binascii
import csv
import os
import struct
import sys

FRAME_MAGIC_V1 = 0x4C4F4731
FRAME_MAGIC_V2 = 0x4C4F4732
FRAME_VERSION_V2 = 2
FRAME_VERSION_V3 = 3
FRAME_VERSION_V4 = 4
FRAME_V1_SIZE = 1722
FRAME_V2_DATA_SIZE = 670
FRAME_V2_SIZE = 674
FRAME_V4_DATA_SIZE = 364
FRAME_V4_SIZE = 368
V2_HEADER_SIZE = 32
QMI_SLOTS_V2 = 32
GZP_SLOTS_V2 = 10
ICP_SLOTS_V2 = 2
QMI_SLOTS_V4 = 16
GZP_SLOTS_V4 = 5
ICP_SLOTS_V4 = 2
CRC_OFFSET_V2 = FRAME_V2_DATA_SIZE
CRC_OFFSET_V4 = FRAME_V4_DATA_SIZE

# V4 reuses the existing 32-bit status field.  Bits 10..12 are deliberately
# outside the historical flags and identify a persisted QMI empty-stream
# recovery attempt; older V1/V2/V3 frames always decode these as zero.
STATUS_QMI_EMPTY_RECOVERY = 1 << 10
STATUS_QMI_RECOVERY_OK = 1 << 11
STATUS_QMI_RECOVERY_FAIL = 1 << 12


def qmi_recovery_flags(status):
    return {
        'qmi_empty_recovery': int(bool(status & STATUS_QMI_EMPTY_RECOVERY)),
        'qmi_recovery_ok': int(bool(status & STATUS_QMI_RECOVERY_OK)),
        'qmi_recovery_failed': int(bool(status & STATUS_QMI_RECOVERY_FAIL)),
    }


def s16(buf, off):
    return struct.unpack_from('<h', buf, off)[0]


def u16(buf, off):
    return struct.unpack_from('<H', buf, off)[0]


def u32(buf, off):
    return struct.unpack_from('<I', buf, off)[0]


def s20(buf, off):
    value = buf[off] | (buf[off + 1] << 8) | (buf[off + 2] << 16)
    value &= 0xFFFFF
    if value & 0x80000:
        value -= 0x100000
    return value


def u24(buf, off):
    return buf[off] | (buf[off + 1] << 8) | (buf[off + 2] << 16)


def _need(buf, off, size, what):
    if off < 0 or off + size > len(buf):
        raise ValueError(f"偏移 {off} 的 {what} 截断：需要 {size} 字节")


def is_v2(buf, off):
    return off + 4 <= len(buf) and u32(buf, off) == FRAME_MAGIC_V2


def _parse_log2(buf, off, version, frame_size, crc_offset,
                qmi_slots, gzp_slots, icp_slots):
    _need(buf, off, V2_HEADER_SIZE, "V2 帧头")
    if u32(buf, off) != FRAME_MAGIC_V2:
        raise ValueError(f"偏移 {off} 不是 LOG2 magic")
    actual_version = u16(buf, off + 4)
    if actual_version != version:
        raise ValueError(f"偏移 {off} 的版本不匹配：期望 {version}，实际 {actual_version}")
    if u16(buf, off + 6) != V2_HEADER_SIZE:
        raise ValueError(f"偏移 {off} 的 LOG2 header_size 错误")
    _need(buf, off, frame_size, f"V{version} 帧")

    expected_crc = u32(buf, off + crc_offset)
    actual_crc = binascii.crc32(buf[off:off + crc_offset]) & 0xFFFFFFFF
    if expected_crc != actual_crc:
        raise ValueError(f"偏移 {off} 的 V{version} CRC32 错误：期望 0x{expected_crc:08x}，实际 0x{actual_crc:08x}")

    qmi_count, gzp_count, icp_count, mts_valid = struct.unpack_from('<4B', buf, off + 16)
    if qmi_count > qmi_slots:
        raise ValueError(f"偏移 {off} 的 qmi_count 超过容量")
    if gzp_count > gzp_slots:
        raise ValueError(f"偏移 {off} 的 gzp_count 超过容量")
    if icp_count > icp_slots:
        raise ValueError(f"偏移 {off} 的 icp_count 超过容量")
    if mts_valid > 1:
        raise ValueError(f"偏移 {off} 的 mts_valid 非法")

    frame = {
        'version': version, 'seq': u32(buf, off + 8),
        'frame_start_ms': u32(buf, off + 12), 'status': u32(buf, off + 20),
        'qmi_dropped': u16(buf, off + 24),
        'qmi_fifo_overflows': u16(buf, off + 26),
        'qmi_reanchors': u16(buf, off + 28) if version in
                         (FRAME_VERSION_V3, FRAME_VERSION_V4) else 0,
        'gzp_missed': u16(buf, off + 30),
        'qmi': [], 'gzp': [], 'icp': [], 'mts4': None,
    }
    frame.update(qmi_recovery_flags(frame['status']))
    p = off + V2_HEADER_SIZE
    for _ in range(qmi_count):
        frame['qmi'].append((u32(buf, p),) + struct.unpack_from('<6h', buf, p + 4))
        p += 16
    p = off + V2_HEADER_SIZE + qmi_slots * 16
    for _ in range(gzp_count):
        frame['gzp'].append((u32(buf, p), u24(buf, p + 4), s16(buf, p + 7)))
        p += 10
    p = off + V2_HEADER_SIZE + qmi_slots * 16 + gzp_slots * 10
    for _ in range(icp_count):
        frame['icp'].append((u32(buf, p), s20(buf, p + 4), s20(buf, p + 7)))
        p += 10
    if mts_valid:
        p = off + V2_HEADER_SIZE + qmi_slots * 16 + gzp_slots * 10 + icp_slots * 10
        frame['mts4'] = (u32(buf, p), s16(buf, p + 4))
    return frame


def parse_v2(buf, off, version=None):
    if version is None:
        _need(buf, off, 6, "LOG2 版本")
        version = u16(buf, off + 4)
    if version not in (FRAME_VERSION_V2, FRAME_VERSION_V3):
        raise ValueError(f"偏移 {off} 的未知 V2/V3 版本 {version}")
    return _parse_log2(buf, off, version, FRAME_V2_SIZE, CRC_OFFSET_V2,
                       QMI_SLOTS_V2, GZP_SLOTS_V2, ICP_SLOTS_V2)


def parse_v4(buf, off):
    return _parse_log2(buf, off, FRAME_VERSION_V4, FRAME_V4_SIZE,
                       CRC_OFFSET_V4, QMI_SLOTS_V4, GZP_SLOTS_V4,
                       ICP_SLOTS_V4)


def parse_v1(buf, off):
    _need(buf, off, FRAME_V1_SIZE, "V1 帧")
    if u32(buf, off) != FRAME_MAGIC_V1:
        raise ValueError(f"偏移 {off} 不是 V1 magic")
    frame = {
        'version': 1, 'seq': u32(buf, off + 4),
        'frame_start_ms': u32(buf, off + 8), 'status': 0, 'qmi_dropped': 0,
        'qmi_fifo_overflows': 0,
        'gzp_missed': 0, 'qmi_reanchors': 0,
        'qmi': [], 'gzp': [], 'icp': [], 'mts4': None,
    }
    frame.update(qmi_recovery_flags(frame['status']))
    p = off + 16
    for i in range(112):
        frame['qmi'].append((None,) + struct.unpack_from('<6h', buf, p + i * 12))
    p += 112 * 12
    for i in range(50):
        frame['gzp'].append((None, u24(buf, p + i * 6), s16(buf, p + i * 6 + 3)))
    p += 50 * 6
    for i in range(10):
        frame['icp'].append((None, s20(buf, p + i * 6), s20(buf, p + i * 6 + 3)))
    frame['mts4'] = (None, s16(buf, off + 1720))
    return frame


def rows_for_frame(frame):
    valid = 1 if frame['version'] in (FRAME_VERSION_V2, FRAME_VERSION_V3,
                                      FRAME_VERSION_V4) else 0
    common = [frame['version'], frame['seq'], frame['frame_start_ms'], valid,
              frame['status'], frame['qmi_dropped'], frame['qmi_fifo_overflows'],
              frame['gzp_missed'], frame['qmi_reanchors'],
              frame['qmi_empty_recovery'], frame['qmi_recovery_ok'],
              frame['qmi_recovery_failed']]
    for i, item in enumerate(frame['qmi']):
        ts, ax, ay, az, gx, gy, gz = item
        yield common + ['qmi', i, ts, ax, ay, az, gx, gy, gz, '', '', '', '', '']
    for i, item in enumerate(frame['gzp']):
        ts, pressure, temp = item
        yield common + ['gzp', i, ts, '', '', '', '', '', '', pressure, temp, '', '', '']
    for i, item in enumerate(frame['icp']):
        ts, pressure, temp = item
        yield common + ['icp', i, ts, '', '', '', '', '', '', '', '', pressure, temp, '']
    if frame['mts4'] is not None:
        ts, temp = frame['mts4']
        yield common + ['mts4', 0, ts, '', '', '', '', '', '', '', '', '', '', temp]


def parse_frame(buf, off):
    _need(buf, off, 4, "帧 magic")
    magic = u32(buf, off)
    if magic == FRAME_MAGIC_V2:
        _need(buf, off, 6, "LOG2 版本")
        version = u16(buf, off + 4)
        if version in (FRAME_VERSION_V2, FRAME_VERSION_V3):
            return parse_v2(buf, off, version), FRAME_V2_SIZE
        if version == FRAME_VERSION_V4:
            return parse_v4(buf, off), FRAME_V4_SIZE
        raise ValueError(f"偏移 {off} 的未知 LOG2 版本 {version}")
    if magic == FRAME_MAGIC_V1:
        return parse_v1(buf, off), FRAME_V1_SIZE
    raise ValueError(f"偏移 {off} 的未知 magic/version 0x{magic:08x}")


def convert(infile, outfile):
    with open(infile, 'rb') as source:
        buf = source.read()
    rows = []
    off = 0
    frames = 0
    while off < len(buf):
        frame, size = parse_frame(buf, off)
        rows.extend(rows_for_frame(frame))
        frames += 1
        off += size

    with open(outfile, 'w', newline='') as target:
        writer = csv.writer(target)
        writer.writerow([
            'format_version', 'frame_seq', 'frame_start_ms', 'timestamp_valid',
            'status', 'qmi_fifo_dropped', 'qmi_fifo_overflows', 'gzp_missed',
            'qmi_reanchors', 'qmi_empty_recovery', 'qmi_recovery_ok',
            'qmi_recovery_failed',
            'channel', 'sample_idx', 'sample_ms',
            'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'gzp_p_raw', 'gzp_t_raw',
            'icp_p_raw', 'icp_t_raw', 'mts4_x256',
        ])
        writer.writerows(rows)
    print(f"{infile}: {len(buf)} bytes, {frames} frames parsed -> {outfile}")


def main():
    args = sys.argv[1:]
    files = []
    output = None
    i = 0
    while i < len(args):
        if args[i] == '-o' and i + 1 < len(args):
            output = args[i + 1]
            i += 2
            continue
        if not args[i].startswith('-'):
            files.append(args[i])
        i += 1
    for infile in files:
        convert(infile, output or (os.path.splitext(infile)[0] + '.csv'))


if __name__ == '__main__':
    main()
