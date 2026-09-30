# PCB32 真机验证记录

文件名：`<嘉立创订单编号>_<YYYYMMDD>.md`

## 基本信息

- 嘉立创订单编号：
- PCB revision / 硬件版本：
- 固件 commit SHA：
  - 待验证示例：`5686ea1c37b0004df4906eab4df61ffe01ffcf79`
  - 上述 SHA 只是待验证版本示例，不代表已经完成真机验证或任何项目已通过。
- 测试日期：
- 操作者：
- 供电方式及条件：
- SD 卡品牌 / 容量 / 文件系统：
- NF-03 接收端：`可用` / `不可用`
- 测试环境备注：

本模板不是测试结果。复制本文件后，必须在人工烧录和真机操作完成后填写实际信息；不得预填或推断 `PASS`。

## 结果摘要

| 项目 | 结果 | 备注 |
| --- | --- | --- |
| 启动流程 | `PASS` / `FAIL` / `NOT TESTED` | |
| QMI8658A | `PASS` / `FAIL` / `NOT TESTED` | |
| GZP | `PASS` / `FAIL` / `NOT TESTED` | |
| ICP-20100 | `PASS` / `FAIL` / `NOT TESTED` | |
| MTS4 | `PASS` / `FAIL` / `NOT TESTED` | |
| SD 卡持续存储 | `PASS` / `FAIL` / `NOT TESTED` | |
| SD 数据解析 | `PASS` / `FAIL` / `NOT TESTED` | |
| NF-03 开机本地自检 | `PASS` / `FAIL` / `NOT TESTED` | |
| NF-03 真实无线收包 | `PASS` / `FAIL` / `NOT TESTED` | |
| Reset 回归 | `PASS` / `FAIL` / `NOT TESTED` | |

## 1. 启动流程

- OLED 自检显示：
- 自检是否结束：
- 是否自动进入采集：
- SD 状态及观察：
- NF-03 状态及观察：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

## 2. 传感器观察

### QMI8658A

- 静止读数：
- 移动 / 旋转后的读数变化：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

### GZP

- 未压缩读数：
- 压缩读数：
- 释放后的读数：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

### ICP-20100

- 初始压力 / 温度：
- 动作及对应读数变化：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

### MTS4

- 初始温度：
- 接触或捂热后的读数：
- 松开后的读数：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

## 3. SD 卡

- 新记录文件路径：
- 初始文件大小：
- 运行后文件大小：
- SD 持续存储结果：`PASS` / `FAIL` / `NOT TESTED`
- 解析命令：
- 解析结果：
- 小型样本路径：
- SD 数据解析结果：`PASS` / `FAIL` / `NOT TESTED`
- 说明：SD 文件正常产生并持续增长、但解析失败时，应记录 SD 持续存储为 `PASS`、SD 数据解析为 `FAIL`，并在异常中记录解析失败现象。

## 4. NF-03

### 开机本地自检

- OLED 状态：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

### 真实无线收包

- 接收端及配置：
- 测试时间范围：
- 收包数量 / 关键字段：
- 小型接收日志路径：
- 没有接收端时填写 `NOT TESTED`。
- 结果：`PASS` / `FAIL` / `NOT TESTED`

## 5. Reset 回归

- 执行次数：
- 每次启动结果：
- SD 恢复情况：
- NF-03 恢复情况：
- 传感器异常或偶发问题：
- 结果：`PASS` / `FAIL` / `NOT TESTED`

## 6. 异常与后续事项

- 异常现象：
- 偶发问题：
- 未完成项目：
- 相关证据路径：

## 7. 测试声明

- 本记录是否由人工烧录并操作真实硬件完成：`是` / `否`
- 本记录中的结果是否基于实际观察：`是` / `否`
- 未实际执行的项目是否填写为 `NOT TESTED`：`是` / `否`
