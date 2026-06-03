# Upper Monitor Host Tools

## Quick Start: One Command Launcher

The recommended way to start the PC tools is:

```powershell
python -m python_host.run_all
```

If the config file does not provide a serial port, the launcher lists available
ports and lets you choose one in the terminal.

You can also pass the serial settings explicitly:

```powershell
python -m python_host.run_all --port COM5 --baud 460800
```

Default launcher config:

```text
config/launcher.json
```

Config keys:

- `port`: serial port, for example `COM5`; empty means interactive selection.
- `baud`: serial baud rate; default is `460800`.
- `udp_host`: UDP target host; default is `127.0.0.1`.
- `udp_port`: UDP target port; default is `5005`.
- `visualizer`: Raylib executable path; default is `build/upper_monitor_visualizer.exe`.
- `out_dir`: capture output directory; empty means `captures/<start_timestamp>`.

Useful launcher options:

```powershell
python -m python_host.run_all --choose-baud
python -m python_host.run_all --config config/launcher.json
python -m python_host.run_all --no-visualizer --port COM5 --baud 460800
```

Manual startup is still supported for debugging:

```powershell
python -m python_host.capture --port COM5 --baud 460800
.\build\upper_monitor_visualizer.exe
```

这个目录包含电脑端的两部分程序：

- `python_host`：从 STM32 串口读取 84 字节姿态包，异步保存 `raw.bin` 和 `aligned.csv`，并把对齐后的 `250Hz` 子帧通过 UDP 转发给可视化程序。
- `visualizer`：使用 `raylib` 接收 UDP 数据，渲染右臂三段骨架和手掌刚体 `Pitch` 实时示波器。

## 串口协议

STM32 发给电脑的每个节点包固定为 `84` 字节小端结构：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `uint8_t` | 固定 `0xAA` |
| `node_id` | `uint8_t` | `0x01=upper_arm`, `0x02=forearm`, `0x03=hand_palm` |
| `seq_cnt` | `uint8_t` | 循环序号，PC 端按它做三节点对齐 |
| `quats[20]` | `float[20]` | 连续 `5` 帧四元数，顺序为 `[(w,x,y,z) * 5]` |
| `checksum` | `uint8_t` | 对前 83 字节逐字节异或 |

## 传感器物理佩戴映射

`node_id` 表示三段刚体传感器，不表示关节本身：

| `node_id` | 数据名 | 物理佩戴位置 | 运动学意义 |
| --- | --- | --- | --- |
| `0x01` | `upper_arm` | 右大臂外侧正中央，三角肌下方与肱三头肌交界处 | 右大臂刚体绝对姿态 |
| `0x02` | `forearm` | 右小臂上端、靠近手肘的外侧，尺骨背面 | 右小臂刚体绝对姿态 |
| `0x03` | `hand_palm` | 右手手背正中央，第三掌骨处，跨过腕关节 | 右手掌刚体绝对姿态，用于捕捉开枪后坐力 Pitch 跳动 |

## UDP 协议

Python 发给 Raylib 的每个 UDP datagram 固定为 `76` 字节：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `magic` | `char[4]` | 固定 `"UMON"` |
| `version` | `uint8_t` | 当前为 `1` |
| `seq_cnt` | `uint8_t` | 来自串口对齐窗口 |
| `sample_idx` | `uint8_t` | `0..4`，表示该 20ms 窗口中的第几帧 |
| `reserved` | `uint8_t` | 固定 `0` |
| `timestamp_ms` | `uint64_t` | Unix epoch 毫秒时间戳 |
| `upper_arm` | `float[4]` | 右大臂刚体 `(w,x,y,z)` |
| `forearm` | `float[4]` | 右小臂刚体 `(w,x,y,z)` |
| `hand_palm` | `float[4]` | 右手掌刚体 `(w,x,y,z)` |
| `upper_arm_stale` | `uint8_t` | `1` 表示该节点超时沿用旧姿态 |
| `forearm_stale` | `uint8_t` | 同上 |
| `hand_palm_stale` | `uint8_t` | 同上 |
| `reserved2` | `uint8_t` | 固定 `0` |
| `hand_palm_pitch_deg` | `float` | 手掌刚体俯仰角，单位度 |
| `hand_palm_pitch_dps` | `float` | 手掌刚体俯仰角速度，单位度/秒 |

## Python 采集端

### 安装依赖

```powershell
python -m pip install -r requirements.txt
```

### 运行

```powershell
python -m python_host.capture --port COM5 --baud 115200
```

可选参数：

- `--udp-host`：默认 `127.0.0.1`
- `--udp-port`：默认 `5005`
- `--out-dir`：默认 `captures/<启动时间戳>`

采集输出：

- `raw.bin`：合法的原始 84 字节串口包顺序拼接
- `aligned.csv`：对齐后的 `250Hz` 子帧表，每行一个样本

`aligned.csv` 的列顺序固定为：

```text
timestamp_ms,seq_cnt,sample_idx,
upper_arm_w,upper_arm_x,upper_arm_y,upper_arm_z,
forearm_w,forearm_x,forearm_y,forearm_z,
hand_palm_w,hand_palm_x,hand_palm_y,hand_palm_z,
upper_arm_stale,forearm_stale,hand_palm_stale,
hand_palm_pitch_deg,hand_palm_pitch_dps
```

## Raylib 可视化端

本机 `raylib` 路径已固定为：

```text
D:/biancheng/C_third_library/raylib
```

### 构建

```powershell
cmake -S visualizer -B build -G "MinGW Makefiles"
cmake --build build
```

如果你的 `raylib` 以后换了位置，可以覆盖：

```powershell
cmake -S visualizer -B build -G "MinGW Makefiles" -DRAYLIB_ROOT=D:/your/path/raylib
```

### 运行

先启动 Python 采集端，再运行：

```powershell
.\build\upper_monitor_visualizer.exe
```

### 交互

- `C`：以当前姿态作为零位做一次静止校准
- `ESC`：退出

窗口布局：

- 上方 `2/3`：右臂骨架 3D 视图
- 下方 `1/3`：手掌刚体 `Pitch` 实时滚动示波器

## 本地验证

Python 语法检查：

```powershell
python -m compileall python_host
```

Raylib 构建检查：

```powershell
cmake --build build
```
