#!/usr/bin/env python3
"""bin2csv V1/V2/V3/V4 与损坏输入的最小解析回归测试。"""
import binascii
import csv
import struct
import tempfile

import bin2csv


def frame(version):
    size = bin2csv.FRAME_V4_SIZE if version == 4 else bin2csv.FRAME_V2_SIZE
    crc_offset = bin2csv.CRC_OFFSET_V4 if version == 4 else bin2csv.CRC_OFFSET_V2
    buf = bytearray(size)
    struct.pack_into('<IHH', buf, 0, bin2csv.FRAME_MAGIC_V2, version, 32)
    struct.pack_into('<4B', buf, 16, 1, 0, 0, 0)
    if version == 3:
        struct.pack_into('<H', buf, 28, 2)
    struct.pack_into('<I', buf, crc_offset,
                     binascii.crc32(buf[:crc_offset]) & 0xFFFFFFFF)
    return bytes(buf)


def main():
    v1 = bytearray(bin2csv.FRAME_V1_SIZE)
    struct.pack_into('<I', v1, 0, bin2csv.FRAME_MAGIC_V1)
    assert bin2csv.parse_frame(bytes(v1), 0)[0]['version'] == 1
    assert bin2csv.parse_frame(frame(2), 0)[0]['version'] == 2
    assert bin2csv.parse_frame(frame(3), 0)[0]['qmi_reanchors'] == 2
    v4, v4_size = bin2csv.parse_frame(frame(4), 0)
    assert v4['version'] == 4 and v4_size == bin2csv.FRAME_V4_SIZE
    assert v4['qmi_reanchors'] == 0

    recovery = bytearray(frame(4))
    struct.pack_into('<I', recovery, 20,
                     bin2csv.STATUS_QMI_EMPTY_RECOVERY |
                     bin2csv.STATUS_QMI_RECOVERY_OK)
    struct.pack_into('<I', recovery, bin2csv.CRC_OFFSET_V4,
                     binascii.crc32(recovery[:bin2csv.CRC_OFFSET_V4]) &
                     0xFFFFFFFF)
    recovered, _ = bin2csv.parse_frame(bytes(recovery), 0)
    assert recovered['qmi_empty_recovery'] == 1
    assert recovered['qmi_recovery_ok'] == 1
    assert recovered['qmi_recovery_failed'] == 0

    over_capacity = bytearray(frame(4))
    over_capacity[16] = bin2csv.QMI_SLOTS_V4 + 1
    struct.pack_into('<I', over_capacity, bin2csv.CRC_OFFSET_V4,
                     binascii.crc32(over_capacity[:bin2csv.CRC_OFFSET_V4]) &
                     0xFFFFFFFF)
    try:
        bin2csv.parse_frame(bytes(over_capacity), 0)
    except ValueError as exc:
        assert 'qmi_count' in str(exc)
    else:
        raise AssertionError('V4 容量越界未停止')

    with tempfile.NamedTemporaryFile(suffix='.bin') as source, \
            tempfile.NamedTemporaryFile(suffix='.csv') as target:
        source.write(bytes(v1) + frame(2) + frame(3) + frame(4))
        source.flush()
        bin2csv.convert(source.name, target.name)
        with open(target.name, newline='') as csv_file:
            rows = list(csv.reader(csv_file))
        assert rows[0][3] == 'timestamp_valid'
        assert [row[3] for row in rows[1:]] == ['0'] * 173 + ['1', '1', '1']

    unknown = bytearray(frame(4))
    struct.pack_into('<H', unknown, 4, 99)
    try:
        bin2csv.parse_frame(bytes(unknown), 0)
    except ValueError as exc:
        assert '未知' in str(exc)
    else:
        raise AssertionError('未知版本未停止')

    try:
        bin2csv.parse_frame(frame(4)[:-1], 0)
    except ValueError as exc:
        assert '截断' in str(exc)
    else:
        raise AssertionError('截断帧未停止')

    corrupt = bytearray(frame(4))
    corrupt[40] ^= 1
    try:
        bin2csv.parse_frame(bytes(corrupt), 0)
    except ValueError as exc:
        assert 'CRC32' in str(exc)
    else:
        raise AssertionError('损坏 CRC 未停止')

    print('bin2csv V1/V2/V3/V4、未知版本、截断和 CRC 测试：通过')


if __name__ == '__main__':
    main()
