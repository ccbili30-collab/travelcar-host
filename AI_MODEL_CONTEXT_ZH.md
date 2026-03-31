# TravelCar Host AI 上下文说明

这份文档面向后续接手本项目的 AI 模型或工程师，目标是帮助快速理解 `travelcar_host` 的真实结构、运行机制、协议约定、关键耦合点，以及后续修改时应优先注意的问题。

## 1. 项目定位

`travelcar_host` 是 TravelCar 项目的电脑上位机。

它不是下位机平衡算法本体，而是一个基于 Python + Tkinter 的桌面控制台，负责：

- 连接 STM32 的蓝牙串口/COM 口
- 发送控制指令
- 接收并展示 STM32 回传状态
- 拉取 ESP32-CAM 的 MJPEG 视频流
- 在视频上叠加 HUD
- 展示实时曲线图
- 在线下发 PID 参数
- 输出 OpenClaw 可读取的状态文件和最新帧

主程序只有一个核心文件：

- `travelcar_host_app.py`

依赖很少：

- `pyserial`
- `requests`
- `Pillow`

## 2. 当前文件结构

```text
travelcar_host/
  README.md
  requirements.txt
  travelcar_host_app.py
  runtime/
    openclaw/
      latest_frame.jpg
      latest_status.json
```

说明：

- `runtime/openclaw/latest_frame.jpg` 保存最近一帧画面
- `runtime/openclaw/latest_status.json` 保存最近状态快照
- `__pycache__` 不是逻辑的一部分，可忽略

## 3. 入口与整体架构

程序入口在 `travelcar_host_app.py` 末尾的 `main()`：

1. 创建 Tk 根窗口
2. 实例化 `TravelCarHostApp`
3. 写入启动日志
4. 进入 `root.mainloop()`

整个脚本可以按职责分成 5 块：

1. 常量和数据结构
2. 串口客户端 `SerialClient`
3. 视频客户端 `MjpegStreamClient`
4. UI 与业务主控制器 `TravelCarHostApp`
5. OpenClaw 集成输出

## 4. 协议约定

当前上位机默认采用“单字符 + 换行”的下行协议。

### 4.1 上位机 -> STM32

默认命令映射：

- `forward` -> `w`
- `backward` -> `s`
- `left` -> `a`
- `right` -> `d`
- `speed_up` -> `u`
- `speed_down` -> `l`
- `stop_drive` -> `q`
- `estop` -> `q`

注意：

- 当前实现里“普通停止”和“急停”都映射到 `q`
- 实际语义是否区分，取决于 STM32 固件

### 4.2 STM32 -> 上位机

正常遥测格式：

```text
D:[距离],L:[左轮角度],R:[右轮角度],S:[当前速度]
```

例如：

```text
D:15.5,L:720.5,R:719.8,S:180
```

特殊状态报文：

- `!WARN:OBSTACLE`
- `!INFO:EMERGENCY STOP`

### 4.3 PID 在线调参

上位机还支持发送额外文本命令：

```text
PID:kp_balance,kd_balance,kp_speed,ki_speed
```

例如：

```text
PID:18.0,0.8,0.4,0.02
```

这部分是否生效，取决于下位机是否支持该文本协议。

## 5. 线程与事件机制

这是本项目最重要的实现特征之一。

### 5.1 设计原则

后台线程不直接更新 Tk 控件。

串口线程和视频线程只负责采集数据，然后通过 `event_queue` 把事件投递回主线程。

主线程中的 `process_events()` 定时轮询队列，并执行：

- 写日志
- 处理下位机反馈
- 渲染视频帧
- 更新界面状态

### 5.2 事件类型

当前队列事件有 3 类：

- `("log", msg)`
- `("feedback", line)`
- `("frame", jpeg_bytes)`

如果未来要扩展功能，建议继续沿用这个模型，不要让后台线程直接 `.set()` Tk 变量或直接修改 Tk 控件。

## 6. 核心类说明

### 6.1 `SerialClient`

职责：

- 打开/关闭串口
- 发送一行文本命令
- 后台循环读取串口反馈
- 断线自动重连

关键行为：

- `connect()` 会先 `disconnect()` 再重连
- `send_line()` 自动补 `\n`
- `_run()` 中 `readline()` 读到完整一行后，交给上层回调
- 发生串口异常时，会根据 `auto_reconnect` 决定是否重试

### 6.2 `MjpegStreamClient`

职责：

- 通过 HTTP 拉取 MJPEG 流
- 从字节流中切分 JPEG 帧
- 统计 FPS

关键行为：

- `requests.get(..., stream=True)`
- 自己扫描 JPEG 起止标记 `FFD8` 和 `FFD9`
- 每拿到一帧就投递给上层

### 6.3 `ScrollingChart`

职责：

- 使用纯 Tk Canvas 画轻量滚动曲线

当前用于两条曲线：

- 前方距离
- 速度挡位

### 6.4 `TravelCarHostApp`

这是整个程序的主控制器，承担了绝大多数业务逻辑：

- UI 初始化
- 键盘绑定
- 串口控制
- 视频控制
- 遥测解析
- 状态同步
- OpenClaw 输出

它不是一个纯 UI 类，而是“界面 + 状态 + 业务”的单体控制器。

## 7. UI 布局

界面分左右两栏。

### 7.1 左侧

- 顶部标题栏
- 主视频区
- 视频控制条
- 两张实时曲线图

### 7.2 右侧

可滚动控制面板，包含：

- 实时状态
- 串口 / 蓝牙
- 移动控制
- PID 在线调参
- 手动发送指令
- 紧急停止按钮
- 运行日志

### 7.3 全屏第一人称模式

通过 `Toplevel` 新开全屏窗口，不是切换主窗口布局。

特点：

- 与主窗口共用同一视频数据源
- 共用同一套键盘控制逻辑
- 叠 HUD
- 底部显示最后反馈和 FPS

## 8. 状态流转

### 8.1 串口状态

串口状态来源于：

- 用户点击连接/断开
- `SerialClient` 是否成功连接
- `process_events()` 收到日志后，根据 `is_connected` 修正显示

当前状态文本常见值：

- `未连接`
- `连接中 (COMx)`
- `已连接 (COMx)`
- `断线（重连中…）`

### 8.2 视频状态

视频状态来源于：

- `start_video()` 设置为 `连接中…`
- `process_events()` 收到视频帧后设为 `播放中`
- `stop_video()` 设为 `已停止`

### 8.3 遥测状态

下位机文本进入 `_handle_feedback()` 后会分三条路径：

1. `!WARN:` 告警
2. `!INFO:` 信息
3. 常规 `D/L/R/S` 遥测

常规遥测会更新：

- `distance_var`
- `left_angle_var`
- `right_angle_var`
- `speed_var`
- `alert_var`
- `self._telem`
- `_chart_dist_data`
- `_chart_speed_data`

## 9. HUD 机制

HUD 由 `draw_hud()` 基于 PIL 在图像上叠加：

- 距离
- 速度
- 左轮角度
- 右轮角度
- FPS
- 告警横幅

注意：

- HUD 只影响显示用图像
- `latest_frame_image` 保存的是原始帧，不是带 HUD 的帧
- OpenClaw 默认写出的 `latest_frame.jpg` 也是原始帧

如果未来想让导出帧或 OpenClaw 输出包含 HUD，需要改写当前逻辑。

## 10. OpenClaw 集成

当前已经预留了与外部系统联动的文件输出：

- `latest_frame.jpg`
- `latest_status.json`

状态 JSON 包含：

- 串口状态
- 视频状态
- 最后指令
- 最后反馈
- 距离/左右轮角度/速度
- 告警
- FPS
- PID 参数
- 帧文件路径

这意味着外部程序无需直接接管 Tk 或串口，只要轮询这个目录就能拿到状态。

## 11. 当前代码中的真实耦合点

后续修改前，必须优先理解下面这些耦合关系。

### 11.1 改协议，不只是改 `COMMANDS`

如果修改控制协议，通常至少需要同时检查：

- `COMMANDS`
- `KEY_BINDINGS`
- 移动控制区按钮文案
- 手动发送指令说明
- README 里的协议说明

### 11.2 新增遥测字段，不只是改解析函数

如果下位机新增例如电池电压、姿态角、模式状态等字段，通常至少要联动：

- `Telemetry` 数据结构
- `_parse_telemetry()`
- `_handle_feedback()`
- 右侧状态面板
- `draw_hud()`
- `write_openclaw_status()`
- 如需图表，还要增加新的图表缓存和绘制逻辑

### 11.3 改线程逻辑要非常谨慎

当前线程边界比较清晰：

- 串口线程只读串口
- 视频线程只读视频
- 主线程更新 UI

不要轻易让后台线程直接操作 Tk 控件，否则会引入随机崩溃或 GUI 不稳定问题。

### 11.4 UI 状态和 OpenClaw 状态文件不是完全同步

这是当前代码一个很容易忽略的点：

- `send_command()` 更新了 `last_command_var`，但不会立即写 `latest_status.json`
- `start_video()` / `stop_video()` 改了视频状态，但不会立即写状态文件
- `!WARN:` / `!INFO:` 在 `_handle_feedback()` 中会提前 `return`

这意味着：

- 某些 UI 上已变化的状态，OpenClaw 文件里可能暂时还是旧值
- 特殊告警/信息报文当前不会触发 `write_openclaw_status()`

如果未来要做更严格的外部联动，需要修复这个同步缺口。

### 11.5 `Escape` 有双重作用

在全屏模式下：

- `Escape` 被键盘映射为 `estop`
- 同时又绑定为退出全屏

所以按一次 `Esc` 的当前效果是：

1. 发送急停
2. 关闭全屏窗口

这可能是预期行为，也可能是后续需要拆分的交互点。

### 11.6 停止和急停当前共用同一协议字符

当前：

- `stop_drive` -> `q`
- `estop` -> `q`

如果下位机将来要区分“普通停车”和“急停锁死”，这里需要重新设计。

## 12. 当前代码与文档的偏差

需要注意，项目文档中存在不同历史阶段的信息，不应全部视为当前事实。

例如：

- 顶层白皮书强调过云台控制、方向键控制等能力
- 但当前 `travelcar_host_app.py` 并没有真正实现 `CAM:*` 类型的命令映射
- 当前实际主线协议仍是 `w/s/a/d/u/l/q`

所以以后判断“系统现在支持什么”，应优先以：

1. `travelcar_host_app.py`
2. `travelcar_host/README.md`
3. 当前运行态文件

为准，而不是只看顶层愿景文档。

## 13. 后续建议的修改策略

如果未来继续演进，建议按下面的原则改：

### 13.1 小改动

适合直接在现有单文件内完成：

- 新增按钮
- 改默认地址
- 改协议字符
- 增加一个状态字段
- 优化 HUD 样式
- 修复状态同步问题

### 13.2 中改动

建议开始拆函数或拆模块：

- 增加更多遥测字段和更多图表
- 增加设备配置保存
- 增加协议版本兼容层
- 增加录像/截图/日志导出

### 13.3 大改动

建议重构：

- 把串口、视频、遥测解析、OpenClaw 输出、UI 分成多个模块
- 把协议层抽象成独立类
- 把状态集中到单一状态模型，而不是分散在多个 Tk 变量和缓存里

## 14. AI 修改前的建议检查清单

任何模型在修改这份代码前，建议先回答下面几个问题：

1. 这次修改是否会影响串口协议？
2. 这次修改是否会影响 OpenClaw 读取的 JSON 结构？
3. 这次修改是否需要同步更新 HUD、状态面板、日志或图表？
4. 这次修改是否会破坏“后台线程只投递事件，主线程更新 UI”的边界？
5. 这次修改是否需要同时更新 README 和界面文案？

## 15. 当前结论

`travelcar_host` 当前是一个已经具备实际可用性的单文件上位机：

- 协议简单直接
- UI 完整
- 视频与控制链路已打通
- 已支持 OpenClaw 外部集成

它最大的优点是：

- 集中
- 直观
- 易于快速迭代

它最大的风险是：

- 业务逻辑都集中在一个类里
- 状态同步不是完全统一
- 文档与真实代码存在少量偏差

后续所有修改都应优先基于“真实运行代码”而不是“历史白皮书设想”。
