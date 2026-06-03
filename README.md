# 上肢姿态监测电脑端工具

这个目录包含电脑端的两部分程序：

- `python_host`：从 STM32 串口读取 `84` 字节姿态包，异步保存 `raw.bin` 和 `aligned.csv`，并把对齐后的 `250Hz` 子帧通过 UDP 转发给可视化程序。
- `visualizer`：使用 `raylib` 接收 UDP 数据，渲染右臂三段骨架、手掌刚体板状模型，以及 `hand_palm Pitch` 实时示波器。

## 快速开始

推荐使用一键启动器：

```powershell
python -m python_host.run_all
```

如果配置文件中没有填写串口号，启动器会自动列出当前可用串口，并在终端中让你选择。

也可以手动指定串口和波特率：

```powershell
python -m python_host.run_all --port COM5 --baud 460800
```

默认配置文件路径：

```text
config/launcher.json
```

默认配置内容：

```json
{
  "port": "",
  "baud": 460800,
  "udp_host": "127.0.0.1",
  "udp_port": 5005,
  "visualizer": "build/upper_monitor_visualizer.exe",
  "out_dir": ""
}
```

配置项说明：

- `port`：串口名，例如 `COM5`；留空时进入交互式选择。
- `baud`：串口波特率，默认 `460800`。
- `udp_host`：UDP 目标地址，默认 `127.0.0.1`。
- `udp_port`：UDP 目标端口，默认 `5005`。
- `visualizer`：Raylib 可执行文件路径，默认 `build/upper_monitor_visualizer.exe`。
- `out_dir`：采集输出目录；留空时自动使用 `captures/<启动时间戳>`。

常用启动命令：

```powershell
python -m python_host.run_all --choose-baud
python -m python_host.run_all --config config/launcher.json
python -m python_host.run_all --no-visualizer --port COM5 --baud 460800
```

如果你想单独调试某一部分，也可以分开启动：

```powershell
python -m python_host.capture --port COM5 --baud 460800
.\build\upper_monitor_visualizer.exe
```

## 传感器佩戴语义

本项目的 3 个 IMU 传感器严格表示三段刚体姿态，而不是关节点本身：

| `node_id` | 数据名称 | 物理佩戴位置 | 运动学意义 |
| --- | --- | --- | --- |
| `0x01` | `upper_arm` | 右大臂外侧正中央，三角肌下方与肱三头肌交界处 | 右大臂刚体绝对姿态 |
| `0x02` | `forearm` | 右小臂上端、靠近手肘的外侧，尺骨背面 | 右小臂刚体绝对姿态 |
| `0x03` | `hand_palm` | 右手手背正中央，第三掌骨区域，传感器跨过腕关节 | 右手掌刚体绝对姿态 |

在 3D 骨架中，人体关节点仍然使用肩、肘、腕的关节链表达，但传感器数据必须分别映射给：

- `upper_arm`：肩到肘这一段
- `forearm`：肘到腕这一段
- `hand_palm`：腕到手掌末端这一段

## 串口协议

STM32 发给电脑的每个节点包固定为 `84` 字节小端结构：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `header` | `uint8_t` | 固定为 `0xAA` |
| `node_id` | `uint8_t` | `0x01=upper_arm`，`0x02=forearm`，`0x03=hand_palm` |
| `seq_cnt` | `uint8_t` | 循环序号，电脑端按它进行三节点对齐 |
| `quats[20]` | `float[20]` | 连续 `5` 帧四元数，顺序为 `[(w,x,y,z) * 5]` |
| `checksum` | `uint8_t` | 对前 `83` 字节逐字节异或得到 |

## UDP 协议

Python 发给 Raylib 的每个 UDP datagram 固定为 `76` 字节：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `magic` | `char[4]` | 固定为 `"UMON"` |
| `version` | `uint8_t` | 当前版本固定为 `1` |
| `seq_cnt` | `uint8_t` | 来自串口对齐窗口的序号 |
| `sample_idx` | `uint8_t` | `0..4`，表示该 `20ms` 窗口内的第几帧 |
| `reserved` | `uint8_t` | 固定为 `0` |
| `timestamp_ms` | `uint64_t` | Unix epoch 毫秒时间戳 |
| `upper_arm` | `float[4]` | 右大臂四元数 `(w,x,y,z)` |
| `forearm` | `float[4]` | 右小臂四元数 `(w,x,y,z)` |
| `hand_palm` | `float[4]` | 右手掌四元数 `(w,x,y,z)` |
| `upper_arm_stale` | `uint8_t` | `1` 表示该节点在对齐时使用了旧数据补齐 |
| `forearm_stale` | `uint8_t` | 同上 |
| `hand_palm_stale` | `uint8_t` | 同上 |
| `reserved2` | `uint8_t` | 固定为 `0` |
| `hand_palm_pitch_deg` | `float` | 手掌刚体绕 `X` 轴的 `Pitch` 角，单位为度 |
| `hand_palm_pitch_dps` | `float` | 手掌刚体绕 `X` 轴的 `Pitch` 角速度，单位为度每秒 |

## Python 采集端

### 安装依赖

```powershell
python -m pip install -r requirements.txt
```

### 独立运行

```powershell
python -m python_host.capture --port COM5 --baud 460800
```

`python_host.capture` 的命令行参数：

- `--port`：必填，串口名，例如 `COM5`
- `--baud`：可选，默认 `115200`
- `--udp-host`：可选，默认 `127.0.0.1`
- `--udp-port`：可选，默认 `5005`
- `--out-dir`：可选，默认 `captures/<启动时间戳>`

说明：当前一键启动器默认波特率是 `460800`，这是为了匹配现阶段 STM32F103 测试固件；如果你单独运行 `python_host.capture`，建议显式写上 `--baud 460800`。

### 对齐与落盘

Python 端按 `seq_cnt` 对三节点数据做窗口对齐：

- 每个节点每包携带连续 `5` 帧四元数。
- 当同一个 `seq_cnt` 的三个节点都到齐时，立即展开为 `5` 个 `250Hz` 子帧。
- 某节点等待超过 `40ms` 仍未到达时，沿用该节点最近一次有效四元数，并把对应 `stale` 标记置为 `1`。
- 每个子帧时间戳以窗口完成时刻为锚点，按 `4ms` 间隔逆推，保证三节点严格同轴。

采集输出文件：

- `raw.bin`：所有合法 `84` 字节串口包的原始二进制顺序追加。
- `aligned.csv`：对齐后的 `250Hz` 子帧表，每行对应一个子帧。

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

本机 `raylib` 默认路径为：

```text
D:/biancheng/C_third_library/raylib
```

### 构建

```powershell
cmake -S visualizer -B build -G "MinGW Makefiles"
cmake --build build
```

如果你的 `raylib` 路径变了，可以覆盖 `RAYLIB_ROOT`：

```powershell
cmake -S visualizer -B build -G "MinGW Makefiles" -DRAYLIB_ROOT=D:/your/path/raylib
```

### 运行

如果不使用一键启动器，请先启动 Python 采集端，再运行：

```powershell
.\build\upper_monitor_visualizer.exe
```

### 画面与交互

窗口布局：

- 上方 `2/3`：右臂三段骨架的 `3D` 视图
- 下方 `1/3`：`hand_palm Pitch` 最近 `2` 秒的实时滚动示波器

渲染说明：

- 大臂和小臂使用圆柱体骨段表示。
- 手掌使用板状长方体表示，而不是圆柱体。
- 某节点 `stale` 时，对应关节会变红，对应骨段也会变成偏灰红色。

交互说明：

- `LMB`：拖动旋转视角
- 鼠标滚轮：缩放视角
- `V`：重置默认视角
- `C`：以当前姿态记录零位校准
- `R`：清除当前校准
- `ESC`：退出程序

校准说明：

- 真实传感器模式下，建议手臂保持参考姿态后按 `C` 做零位校准。
- STM32F103 的水平举枪测试模式下，一般不要按 `C`；如果误按了，可以按 `R` 清除校准。

## 本地验证

Python 语法检查：

```powershell
python -m compileall python_host
```

Raylib 构建检查：

```powershell
cmake --build build
```

## 目录说明

- `python_host/`：串口采集、协议解析、三节点对齐、CSV 落盘、UDP 转发、一键启动器
- `visualizer/`：Raylib 可视化、UDP 接收、骨架渲染、示波器渲染
- `config/launcher.json`：一键启动器默认配置
- `captures/`：采集输出目录
