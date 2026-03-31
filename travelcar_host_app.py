"""
TravelCar 上位机控制台 v2.0
==========================
全面重写版本，功能包括：
  - 现代暗色主题 (完全自绘 ttk.Style)
  - 视频区叠加 HUD (距离 / 速度 / 角度 / 告警)
  - 实时波形图 (距离 + 速度滚动曲线, tkinter Canvas 实现, 无需 matplotlib)
  - PID 在线调参面板 (Kp/Kd/Ki 实时下发)
  - 串口断线自动重连
  - 帧率统计
  - 全屏第一人称驾驶模式 (HUD + WASD)
  - OpenClaw 集成接口 (写 latest_frame.jpg / latest_status.json)
"""
from __future__ import annotations

import collections
import io
import json
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Deque, Optional

import requests
import serial
from PIL import Image, ImageDraw, ImageFont, ImageTk, ImageOps
from serial.tools import list_ports

# ---------------------------------------------------------------------------
# 路径 & 全局常量
# ---------------------------------------------------------------------------
APP_DIR = Path(__file__).resolve().parent
OPENCLAW_DIR = APP_DIR / "runtime" / "openclaw"
OPENCLAW_DIR.mkdir(parents=True, exist_ok=True)
OPENCLAW_FRAME_PATH = OPENCLAW_DIR / "latest_frame.jpg"
OPENCLAW_STATUS_PATH = OPENCLAW_DIR / "latest_status.json"

# 协议指令表
COMMANDS: dict[str, str] = {
    "forward":    "w",
    "backward":   "s",
    "left":       "a",
    "right":      "d",
    "speed_up":   "u",
    "speed_down": "l",
    "stop_drive": "q",
    "estop":      "q",
}

# 曲线图配置
CHART_POINTS = 120          # 最多保留120个数据点
CHART_HEIGHT  = 120         # 像素高度
CHART_FPS     = 10          # 刷新频率 (每秒)

# 暗色主题色板
PALETTE = {
    "bg":          "#0d1117",
    "surface":     "#161b22",
    "surface2":    "#21262d",
    "border":      "#30363d",
    "accent":      "#58a6ff",
    "accent2":     "#3fb950",
    "warn":        "#f85149",
    "warn_soft":   "#ff7b72",
    "text":        "#e6edf3",
    "text_muted":  "#8b949e",
    "hud_bg":      "#000000cc",
    "chart_dist":  "#58a6ff",
    "chart_speed": "#3fb950",
    "chart_bg":    "#0d1117",
    "chart_grid":  "#21262d",
}

# ---------------------------------------------------------------------------
# 数据类
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class KeyBinding:
    command_name: str
    stop_on_release: bool = False

KEY_BINDINGS: dict[str, KeyBinding] = {
    "w":      KeyBinding("forward",    stop_on_release=True),
    "s":      KeyBinding("backward",   stop_on_release=True),
    "a":      KeyBinding("left",       stop_on_release=True),
    "d":      KeyBinding("right",      stop_on_release=True),
    "space":  KeyBinding("stop_drive"),
    "u":      KeyBinding("speed_up"),
    "l":      KeyBinding("speed_down"),
    "Escape": KeyBinding("estop"),
}

@dataclass
class Telemetry:
    """解析后的下位机遥测数据快照"""
    distance:    float = 0.0   # cm
    left_angle:  float = 0.0   # deg
    right_angle: float = 0.0   # deg
    speed:       int   = 0     # 0-255
    alert:       str   = ""
    valid:       bool  = False

# ---------------------------------------------------------------------------
# SerialClient — 串口通信（含自动重连）
# ---------------------------------------------------------------------------
class SerialClient:
    RECONNECT_INTERVAL = 3.0  # 秒

    def __init__(self, log_cb, line_cb):
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop    = threading.Event()
        self._log     = log_cb
        self._line_cb = line_cb
        self._port    = ""
        self._baud    = 9600
        self._auto_reconnect = False
        self.is_connected = False

    # ---- 公共接口 ----------------------------------------------------------

    def connect(self, port: str, baudrate: int, auto_reconnect: bool = True):
        self.disconnect()
        self._port = port
        self._baud = baudrate
        self._auto_reconnect = auto_reconnect
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._auto_reconnect = False
        self._stop.set()
        self._close_serial()
        self._thread = None
        self.is_connected = False

    def send_line(self, payload: str):
        if not self.is_connected or self._serial is None:
            raise serial.SerialException("串口未连接")
        wire = f"{payload}\n".encode("utf-8")
        self._serial.write(wire)
        self._log(f">> {payload}")

    # ---- 内部 --------------------------------------------------------------

    def _close_serial(self):
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _open_serial(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self._port, baudrate=self._baud, timeout=0.1
            )
            self.is_connected = True
            self._log(f"串口已连接：{self._port} @ {self._baud}")
            return True
        except serial.SerialException as exc:
            self._log(f"串口连接失败：{exc}")
            self.is_connected = False
            return False

    def _run(self):
        while not self._stop.is_set():
            if not self._open_serial():
                if self._auto_reconnect:
                    self._log(f"将在 {self.RECONNECT_INTERVAL:.0f}s 后重试…")
                    self._stop.wait(self.RECONNECT_INTERVAL)
                    continue
                else:
                    break

            # 读取循环
            while not self._stop.is_set():
                try:
                    raw = self._serial.readline()
                except serial.SerialException as exc:
                    self._log(f"串口读取错误：{exc}")
                    break
                if raw:
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        self._line_cb(line)

            # 离开读取循环 → 断线
            self.is_connected = False
            self._close_serial()
            if self._auto_reconnect and not self._stop.is_set():
                self._log("串口断线，尝试重连…")
                self._stop.wait(self.RECONNECT_INTERVAL)
            else:
                break

        self._log("串口线程已退出")

# ---------------------------------------------------------------------------
# MjpegStreamClient — 视频流
# ---------------------------------------------------------------------------
class MjpegStreamClient:
    def __init__(self, frame_cb, log_cb):
        self._frame_cb = frame_cb
        self._log      = log_cb
        self._thread: Optional[threading.Thread] = None
        self._stop     = threading.Event()
        self.is_running = False
        self._fps_count  = 0
        self._fps_ts     = time.time()
        self.fps: float  = 0.0

    def start(self, url: str):
        self.stop()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(url,), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self.is_running = False

    def _run(self, url: str):
        self.is_running = True
        self._log(f"正在连接视频流：{url}")
        try:
            with requests.get(url, stream=True, timeout=5) as resp:
                resp.raise_for_status()
                buf = bytearray()
                for chunk in resp.iter_content(chunk_size=4096):
                    if self._stop.is_set():
                        break
                    if not chunk:
                        continue
                    buf.extend(chunk)
                    while True:
                        s = buf.find(b"\xff\xd8")
                        e = buf.find(b"\xff\xd9", s + 2)
                        if s == -1 or e == -1:
                            break
                        jpg = bytes(buf[s: e + 2])
                        del buf[: e + 2]
                        self._frame_cb(jpg)
                        # FPS 统计
                        self._fps_count += 1
                        now = time.time()
                        if now - self._fps_ts >= 1.0:
                            self.fps = self._fps_count / (now - self._fps_ts)
                            self._fps_count = 0
                            self._fps_ts = now
        except requests.RequestException as exc:
            self._log(f"视频流错误：{exc}")
        finally:
            self.is_running = False
            self._log("视频流已停止")

# ---------------------------------------------------------------------------
# ScrollingChart — 纯 tkinter Canvas 实时波形图
# ---------------------------------------------------------------------------
class ScrollingChart(tk.Canvas):
    """轻量级滚动折线图，无外部依赖"""

    def __init__(self, parent, label: str, color: str, unit: str = "",
                 y_min: float = 0.0, y_max: float = 100.0, **kwargs):
        super().__init__(
            parent,
            bg=PALETTE["chart_bg"],
            highlightthickness=1,
            highlightbackground=PALETTE["border"],
            **kwargs,
        )
        self._label  = label
        self._color  = color
        self._unit   = unit
        self._y_min  = y_min
        self._y_max  = y_max
        self._data: Deque[float] = collections.deque(maxlen=CHART_POINTS)
        self.bind("<Configure>", lambda _e: self._redraw())

    def push(self, value: float):
        self._data.append(value)
        self._redraw()

    def _redraw(self):
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 2 or h < 2:
            return
        self.delete("all")

        # 背景
        self.create_rectangle(0, 0, w, h, fill=PALETTE["chart_bg"], outline="")

        # 网格横线 (4条)
        for i in range(1, 4):
            y = int(h * i / 4)
            self.create_line(0, y, w, y, fill=PALETTE["chart_grid"])

        # 标签 & 当前值
        cur = self._data[-1] if self._data else 0.0
        self.create_text(
            6, 4, anchor="nw",
            text=f"{self._label}: {cur:.1f}{self._unit}",
            fill=self._color, font=("Consolas", 10, "bold"),
        )

        if len(self._data) < 2:
            return

        # 折线
        pts = list(self._data)
        rng = self._y_max - self._y_min or 1.0
        step = w / (CHART_POINTS - 1)
        coords: list[float] = []
        for i, v in enumerate(pts):
            x = (CHART_POINTS - len(pts) + i) * step
            y = h - (v - self._y_min) / rng * (h - 4) - 2
            y = max(2.0, min(float(h - 2), y))
            coords.extend([x, y])

        if len(coords) >= 4:
            self.create_line(coords, fill=self._color, width=2, smooth=True)

# ---------------------------------------------------------------------------
# HUD 覆盖绘制（在 PIL Image 上叠加信息）
# ---------------------------------------------------------------------------
def draw_hud(image: Image.Image, telem: Telemetry, fps: float) -> Image.Image:
    """在图像右下角叠加 HUD 信息，左上角叠加告警"""
    img = image.copy()
    draw = ImageDraw.Draw(img, "RGBA")
    w, h = img.size

    # ── 遥测数据块（右下）──────────────────────────────────────────────────
    lines = [
        f"DIST  {telem.distance:>6.1f} cm",
        f"SPEED {telem.speed:>6d}",
        f"L-ANG {telem.left_angle:>6.1f}°",
        f"R-ANG {telem.right_angle:>6.1f}°",
        f"FPS   {fps:>6.1f}",
    ]
    font_size = max(12, h // 35)
    try:
        font = ImageFont.truetype("consola.ttf", font_size)
    except Exception:
        font = ImageFont.load_default()

    line_h = font_size + 4
    box_w  = font_size * 14
    box_h  = line_h * len(lines) + 8
    bx, by = w - box_w - 8, h - box_h - 8

    draw.rectangle([bx - 4, by - 4, w - 4, h - 4], fill=(0, 0, 0, 160))
    for i, txt in enumerate(lines):
        draw.text((bx, by + i * line_h + 4), txt,
                  fill=(88, 166, 255, 230), font=font)

    # ── 告警横幅（顶部居中）───────────────────────────────────────────────
    if telem.alert:
        banner = f"  ⚠  {telem.alert}  "
        draw.rectangle([0, 0, w, font_size + 12], fill=(248, 81, 73, 200))
        draw.text((w // 2, 6), banner, fill=(255, 255, 255, 255),
                  font=font, anchor="mt")

    return img

# ---------------------------------------------------------------------------
# 主应用
# ---------------------------------------------------------------------------
class TravelCarHostApp:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("TravelCar 上位机 v2.0")
        self.root.geometry("1400x820")
        self.root.minsize(1100, 700)
        self.root.configure(bg=PALETTE["bg"])

        # --- 状态变量 -------------------------------------------------------
        self.event_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.active_drive_key: Optional[str] = None
        self.latest_frame_image: Optional[Image.Image] = None
        self.fullscreen_window: Optional[tk.Toplevel] = None
        self.fullscreen_video_label: Optional[tk.Label] = None
        self._video_photo    = None
        self._fp_video_photo = None
        self._telem = Telemetry()
        self._auto_reconnect_var = tk.BooleanVar(value=True)

        # Tk 变量
        self.port_var           = tk.StringVar()
        self.baud_var           = tk.StringVar(value="9600")
        self.video_url_var      = tk.StringVar(value="http://192.168.4.1:81/stream")
        self.serial_status_var  = tk.StringVar(value="未连接")
        self.video_status_var   = tk.StringVar(value="已停止")
        self.last_command_var   = tk.StringVar(value="-")
        self.last_feedback_var  = tk.StringVar(value="-")
        self.distance_var       = tk.StringVar(value="-")
        self.left_angle_var     = tk.StringVar(value="-")
        self.right_angle_var    = tk.StringVar(value="-")
        self.speed_var          = tk.StringVar(value="-")
        self.alert_var          = tk.StringVar(value="无")
        self.fps_var            = tk.StringVar(value="0.0 fps")
        self.manual_cmd_var     = tk.StringVar(value="PING")
        self.hud_enabled_var    = tk.BooleanVar(value=True)

        # PID 变量
        self.kp_balance_var = tk.StringVar(value="18.0")
        self.kd_balance_var = tk.StringVar(value="0.8")
        self.kp_speed_var   = tk.StringVar(value="0.4")
        self.ki_speed_var   = tk.StringVar(value="0.02")

        # 客户端
        self.serial_client = SerialClient(self.enqueue_log, self.enqueue_feedback)
        self.video_client  = MjpegStreamClient(self.enqueue_frame, self.enqueue_log)

        # 波形图数据
        self._chart_dist_data: Deque[float]  = collections.deque(maxlen=CHART_POINTS)
        self._chart_speed_data: Deque[float] = collections.deque(maxlen=CHART_POINTS)

        self._apply_theme()
        self._build_ui()
        self._bind_keys()
        self.refresh_ports()
        self.write_openclaw_status()

        # 定时任务
        self.root.after(40,   self.process_events)
        self.root.after(1000, self._update_fps_label)
        self.root.after(1000 // CHART_FPS, self._tick_charts)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ======================================================================
    # 主题
    # ======================================================================
    def _apply_theme(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")

        bg  = PALETTE["bg"]
        sf  = PALETTE["surface"]
        sf2 = PALETTE["surface2"]
        bd  = PALETTE["border"]
        tx  = PALETTE["text"]
        mu  = PALETTE["text_muted"]
        ac  = PALETTE["accent"]

        style.configure(".",
            background=bg, foreground=tx,
            fieldbackground=sf2, insertbackground=tx,
            bordercolor=bd, relief="flat",
        )
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=tx, font=("Microsoft YaHei", 10))
        style.configure("Header.TLabel", background=bg, foreground=ac,
                        font=("Microsoft YaHei", 13, "bold"))
        style.configure("Muted.TLabel", background=bg, foreground=mu,
                        font=("Microsoft YaHei", 9))
        style.configure("Value.TLabel", background=bg, foreground=PALETTE["accent2"],
                        font=("Consolas", 11, "bold"))

        style.configure("TLabelframe",
            background=sf, foreground=mu,
            bordercolor=bd, relief="solid", padding=10,
        )
        style.configure("TLabelframe.Label",
            background=sf, foreground=mu,
            font=("Microsoft YaHei", 9),
        )

        style.configure("TButton",
            background=sf2, foreground=tx,
            bordercolor=bd, relief="flat",
            padding=(10, 5),
            font=("Microsoft YaHei", 10),
        )
        style.map("TButton",
            background=[("active", ac), ("pressed", "#1f6feb")],
            foreground=[("active", "#ffffff")],
        )

        style.configure("Accent.TButton",
            background=ac, foreground="#ffffff",
            font=("Microsoft YaHei", 10, "bold"),
        )
        style.map("Accent.TButton",
            background=[("active", "#79c0ff"), ("pressed", "#1f6feb")],
        )

        style.configure("TEntry",
            fieldbackground=sf2, foreground=tx,
            insertcolor=tx, bordercolor=bd,
        )
        style.configure("TCombobox",
            fieldbackground=sf2, foreground=tx,
            selectbackground=ac, selectforeground="#ffffff",
        )
        style.configure("TScrollbar", background=sf2, troughcolor=bg, bordercolor=bd)
        style.configure("TCheckbutton", background=bg, foreground=tx)

    # ======================================================================
    # UI 构建
    # ======================================================================
    def _build_ui(self):
        self.root.columnconfigure(0, weight=7)
        self.root.columnconfigure(1, weight=3)
        self.root.rowconfigure(0, weight=1)

        # ── 左侧：视频 + 波形图 ────────────────────────────────────────────
        left = tk.Frame(self.root, bg=PALETTE["bg"])
        left.grid(row=0, column=0, sticky="nsew", padx=(12, 6), pady=12)
        left.rowconfigure(1, weight=3)
        left.rowconfigure(2, weight=1)
        left.columnconfigure(0, weight=1)

        # 标题栏
        title_bar = tk.Frame(left, bg=PALETTE["bg"])
        title_bar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        tk.Label(title_bar, text="TravelCar 上位机 v2.0",
                 bg=PALETTE["bg"], fg=PALETTE["accent"],
                 font=("Microsoft YaHei", 14, "bold")).pack(side="left")
        self.fps_label = tk.Label(title_bar, textvariable=self.fps_var,
                                  bg=PALETTE["bg"], fg=PALETTE["text_muted"],
                                  font=("Consolas", 10))
        self.fps_label.pack(side="right", padx=4)
        tk.Checkbutton(title_bar, text="HUD",
                       variable=self.hud_enabled_var,
                       bg=PALETTE["bg"], fg=PALETTE["text_muted"],
                       selectcolor=PALETTE["surface2"],
                       activebackground=PALETTE["bg"],
                       font=("Microsoft YaHei", 9)).pack(side="right", padx=8)

        # 视频画面
        video_frame = tk.Frame(left, bg=PALETTE["surface"], bd=1,
                                relief="solid", highlightbackground=PALETTE["border"])
        video_frame.grid(row=1, column=0, sticky="nsew")
        video_frame.rowconfigure(0, weight=1)
        video_frame.columnconfigure(0, weight=1)

        self.video_label = tk.Label(
            video_frame,
            text="暂无视频画面\n\n启动视频流后显示实时画面",
            bg=PALETTE["surface"], fg=PALETTE["text_muted"],
            font=("Microsoft YaHei", 14),
        )
        self.video_label.grid(row=0, column=0, sticky="nsew")
        self.video_label.bind("<Double-Button-1>", lambda _e: self.open_first_person_mode())

        # 视频控制条
        vctrl = tk.Frame(left, bg=PALETTE["surface2"])
        vctrl.grid(row=1, column=0, sticky="sew", ipady=4)
        tk.Label(vctrl, text="视频流地址",
                 bg=PALETTE["surface2"], fg=PALETTE["text_muted"],
                 font=("Microsoft YaHei", 9)).pack(side="left", padx=(8, 4))
        tk.Entry(vctrl, textvariable=self.video_url_var,
                 bg=PALETTE["surface2"], fg=PALETTE["text"], insertbackground=PALETTE["text"],
                 relief="flat", font=("Consolas", 10), width=30).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(vctrl, text="启动", style="Accent.TButton",
                   command=self.start_video).pack(side="left", padx=2)
        ttk.Button(vctrl, text="停止", command=self.stop_video).pack(side="left", padx=2)
        ttk.Button(vctrl, text="全屏", command=self.open_first_person_mode).pack(side="left", padx=2)
        ttk.Button(vctrl, text="导出帧", command=self.export_latest_frame).pack(side="left", padx=(2, 8))

        # 波形图区
        chart_area = tk.Frame(left, bg=PALETTE["bg"])
        chart_area.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        chart_area.columnconfigure(0, weight=1)
        chart_area.columnconfigure(1, weight=1)
        chart_area.rowconfigure(0, weight=1)

        self.chart_dist = ScrollingChart(
            chart_area, label="前方距离", color=PALETTE["chart_dist"],
            unit=" cm", y_min=0.0, y_max=200.0, height=CHART_HEIGHT,
        )
        self.chart_dist.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        self.chart_speed = ScrollingChart(
            chart_area, label="速度挡位", color=PALETTE["chart_speed"],
            unit="", y_min=0.0, y_max=255.0, height=CHART_HEIGHT,
        )
        self.chart_speed.grid(row=0, column=1, sticky="nsew", padx=(4, 0))

        # ── 右侧：可滚动控制面板 ───────────────────────────────────────────
        right_outer = tk.Frame(self.root, bg=PALETTE["bg"])
        right_outer.grid(row=0, column=1, sticky="nsew", padx=(0, 12), pady=12)
        right_outer.rowconfigure(0, weight=1)
        right_outer.columnconfigure(0, weight=1)

        canvas = tk.Canvas(right_outer, bg=PALETTE["bg"], highlightthickness=0)
        scrollbar = ttk.Scrollbar(right_outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        self._scroll_frame = tk.Frame(canvas, bg=PALETTE["bg"])
        _cw = canvas.create_window((0, 0), window=self._scroll_frame, anchor="nw")

        def _on_scf(_e):
            canvas.configure(scrollregion=canvas.bbox("all"))
        def _on_canvas(_e):
            canvas.itemconfigure(_cw, width=_e.width)
        def _on_wheel(_e):
            canvas.yview_scroll(int(-1 * (_e.delta / 120)), "units")

        self._scroll_frame.bind("<Configure>", _on_scf)
        canvas.bind("<Configure>", _on_canvas)
        canvas.bind_all("<MouseWheel>", _on_wheel)
        self._scroll_frame.columnconfigure(0, weight=1)

        # 填充右侧面板各区域
        self._build_status_panel(self._scroll_frame, row=0)
        self._build_serial_panel(self._scroll_frame, row=1)
        self._build_control_panel(self._scroll_frame, row=2)
        self._build_pid_panel(self._scroll_frame, row=3)
        self._build_manual_panel(self._scroll_frame, row=4)
        self._build_safety_panel(self._scroll_frame, row=5)
        self._build_log_panel(self._scroll_frame, row=6)

    # ---- 右侧子面板 --------------------------------------------------------

    def _lf(self, parent, text, row, **kw) -> tk.Frame:
        """创建暗色 LabelFrame"""
        container = tk.Frame(parent, bg=PALETTE["surface"],
                             highlightthickness=1,
                             highlightbackground=PALETTE["border"])
        container.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        container.columnconfigure(0, weight=1)
        tk.Label(container, text=f" {text} ",
                 bg=PALETTE["surface"], fg=PALETTE["text_muted"],
                 font=("Microsoft YaHei", 9)).grid(row=0, column=0, sticky="w", padx=4)
        inner = tk.Frame(container, bg=PALETTE["surface"])
        inner.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        inner.columnconfigure(1, weight=1)
        return inner

    def _status_row(self, parent, row, label, var, is_value=False):
        tk.Label(parent, text=label, bg=PALETTE["surface"],
                 fg=PALETTE["text_muted"], font=("Microsoft YaHei", 9)
                 ).grid(row=row, column=0, sticky="w", pady=2, padx=(0, 12))
        style = "Consolas" if is_value else "Microsoft YaHei"
        fg    = PALETTE["accent2"] if is_value else PALETTE["text"]
        tk.Label(parent, textvariable=var, bg=PALETTE["surface"],
                 fg=fg, font=(style, 10, "bold" if is_value else "normal"),
                 anchor="e").grid(row=row, column=1, sticky="e", pady=2)

    def _build_status_panel(self, parent, row):
        inner = self._lf(parent, "实时状态", row)
        self._status_row(inner, 0, "串口",     self.serial_status_var)
        self._status_row(inner, 1, "视频",     self.video_status_var)
        self._status_row(inner, 2, "最后指令", self.last_command_var)
        self._status_row(inner, 3, "下位机回报", self.last_feedback_var)
        ttk.Separator(inner, orient="horizontal").grid(row=4, column=0, columnspan=2, sticky="ew", pady=4)
        self._status_row(inner, 5, "前方距离", self.distance_var,   is_value=True)
        self._status_row(inner, 6, "左轮角度", self.left_angle_var, is_value=True)
        self._status_row(inner, 7, "右轮角度", self.right_angle_var,is_value=True)
        self._status_row(inner, 8, "速度挡位", self.speed_var,      is_value=True)
        self._status_row(inner, 9, "系统提示", self.alert_var)

    def _build_serial_panel(self, parent, row):
        inner = self._lf(parent, "串口 / 蓝牙", row)

        tk.Label(inner, text="串口号", bg=PALETTE["surface"],
                 fg=PALETTE["text_muted"], font=("Microsoft YaHei", 9)
                 ).grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(inner, text="刷新", command=self.refresh_ports
                   ).grid(row=0, column=2, padx=(0, 0))

        tk.Label(inner, text="波特率", bg=PALETTE["surface"],
                 fg=PALETTE["text_muted"], font=("Microsoft YaHei", 9)
                 ).grid(row=1, column=0, sticky="w", pady=(6, 0))
        tk.Entry(inner, textvariable=self.baud_var,
                 bg=PALETTE["surface2"], fg=PALETTE["text"],
                 insertbackground=PALETTE["text"], relief="flat",
                 font=("Consolas", 10)
                 ).grid(row=1, column=1, sticky="ew", padx=6, pady=(6, 0))

        btn_row = tk.Frame(inner, bg=PALETTE["surface"])
        btn_row.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        btn_row.columnconfigure((0, 1), weight=1)
        ttk.Button(btn_row, text="连接", style="Accent.TButton",
                   command=self.connect_serial).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(btn_row, text="断开",
                   command=self.disconnect_serial).grid(row=0, column=1, sticky="ew")

        tk.Checkbutton(inner, text="断线自动重连",
                       variable=self._auto_reconnect_var,
                       bg=PALETTE["surface"], fg=PALETTE["text_muted"],
                       selectcolor=PALETTE["surface2"],
                       activebackground=PALETTE["surface"],
                       font=("Microsoft YaHei", 9)
                       ).grid(row=3, column=0, columnspan=3, sticky="w", pady=(6, 0))

    def _build_control_panel(self, parent, row):
        inner = self._lf(parent, "移动控制", row)
        for c in range(3):
            inner.columnconfigure(c, weight=1)

        self._hold_btn(inner, "▲ 前进 W", "forward",  0, 1)
        self._hold_btn(inner, "◄ 左转 A", "left",     1, 0)
        ttk.Button(inner, text="■ 停止",
                   command=lambda: self.send_command("stop_drive")
                   ).grid(row=1, column=1, sticky="ew", padx=3, pady=3)
        self._hold_btn(inner, "► 右转 D", "right",    1, 2)
        self._hold_btn(inner, "▼ 后退 S", "backward", 2, 1)

        speed_row = tk.Frame(inner, bg=PALETTE["surface"])
        speed_row.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        speed_row.columnconfigure((0, 1), weight=1)
        ttk.Button(speed_row, text="加速 U",
                   command=lambda: self.send_command("speed_up")
                   ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(speed_row, text="减速 L",
                   command=lambda: self.send_command("speed_down")
                   ).grid(row=0, column=1, sticky="ew")

        tk.Label(inner,
                 text="WASD 控制移动；松键自动停止；U/L 调整速度挡",
                 bg=PALETTE["surface"], fg=PALETTE["text_muted"],
                 font=("Microsoft YaHei", 8), wraplength=220, justify="left"
                 ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(6, 0))

    def _build_pid_panel(self, parent, row):
        inner = self._lf(parent, "PID 在线调参", row)
        inner.columnconfigure(1, weight=1)

        params = [
            ("直立 Kp", self.kp_balance_var),
            ("直立 Kd", self.kd_balance_var),
            ("速度 Kp", self.kp_speed_var),
            ("速度 Ki", self.ki_speed_var),
        ]
        for i, (label, var) in enumerate(params):
            tk.Label(inner, text=label, bg=PALETTE["surface"],
                     fg=PALETTE["text_muted"], font=("Microsoft YaHei", 9)
                     ).grid(row=i, column=0, sticky="w", pady=3, padx=(0, 8))
            tk.Entry(inner, textvariable=var, width=8,
                     bg=PALETTE["surface2"], fg=PALETTE["accent2"],
                     insertbackground=PALETTE["text"], relief="flat",
                     font=("Consolas", 10)
                     ).grid(row=i, column=1, sticky="ew", padx=(0, 8))

        ttk.Button(inner, text="下发 PID 参数", style="Accent.TButton",
                   command=self.send_pid_params
                   ).grid(row=len(params), column=0, columnspan=2, sticky="ew", pady=(8, 0))

        tk.Label(inner,
                 text="格式：PID:kp_b,kd_b,kp_s,ki_s  需要下位机固件支持",
                 bg=PALETTE["surface"], fg=PALETTE["text_muted"],
                 font=("Microsoft YaHei", 8), wraplength=220, justify="left"
                 ).grid(row=len(params)+1, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def _build_manual_panel(self, parent, row):
        inner = self._lf(parent, "手动发送指令", row)
        inner.columnconfigure(0, weight=1)
        tk.Entry(inner, textvariable=self.manual_cmd_var,
                 bg=PALETTE["surface2"], fg=PALETTE["text"],
                 insertbackground=PALETTE["text"], relief="flat",
                 font=("Consolas", 10)
                 ).grid(row=0, column=0, sticky="ew")
        ttk.Button(inner, text="发送", style="Accent.TButton",
                   command=self.send_manual_command
                   ).grid(row=0, column=1, padx=(8, 0))
        tk.Label(inner, text="示例：PING  w  q  PID:18,0.8,0.4,0.02",
                 bg=PALETTE["surface"], fg=PALETTE["text_muted"],
                 font=("Microsoft YaHei", 8)
                 ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def _build_safety_panel(self, parent, row):
        container = tk.Frame(parent, bg=PALETTE["surface"],
                             highlightthickness=1,
                             highlightbackground=PALETTE["border"])
        container.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        container.columnconfigure(0, weight=1)
        tk.Button(
            container,
            text="⚡  紧 急 停 止",
            bg="#b91c1c", fg="white", activebackground="#991b1b",
            font=("Microsoft YaHei", 14, "bold"),
            relief="flat", bd=0, pady=10,
            command=self.emergency_stop,
        ).grid(row=0, column=0, sticky="ew")

    def _build_log_panel(self, parent, row):
        container = tk.Frame(parent, bg=PALETTE["surface"],
                             highlightthickness=1,
                             highlightbackground=PALETTE["border"])
        container.grid(row=row, column=0, sticky="nsew")
        container.rowconfigure(1, weight=1)
        container.columnconfigure(0, weight=1)
        tk.Label(container, text=" 运行日志 ",
                 bg=PALETTE["surface"], fg=PALETTE["text_muted"],
                 font=("Microsoft YaHei", 9)).grid(row=0, column=0, sticky="w", padx=4)
        self.log_text = tk.Text(
            container, height=10, wrap="word",
            bg=PALETTE["bg"], fg=PALETTE["text_muted"],
            insertbackground=PALETTE["text"],
            font=("Consolas", 9), relief="flat",
        )
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))
        self.log_text.configure(state="disabled")

    def _hold_btn(self, parent, text, cmd, row, col):
        b = ttk.Button(parent, text=text)
        b.grid(row=row, column=col, sticky="ew", padx=3, pady=3)
        b.bind("<ButtonPress-1>",   lambda _e: self.start_drive_command(cmd))
        b.bind("<ButtonRelease-1>", lambda _e: self.stop_drive())
        return b

    # ======================================================================
    # 键盘绑定
    # ======================================================================
    def _bind_keys(self):
        self.root.bind("<KeyPress>",   self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)

    def _bind_fullscreen_keys(self, win: tk.Toplevel):
        win.bind("<KeyPress>",         self.on_key_press)
        win.bind("<KeyRelease>",       self.on_key_release)
        win.bind("<Escape>",           lambda _e: self.close_first_person_mode())
        win.bind("<Double-Button-1>",  lambda _e: self.close_first_person_mode())

    def on_key_press(self, event):
        k = event.keysym
        b = KEY_BINDINGS.get(k) or KEY_BINDINGS.get((event.char or "").lower())
        if b is None:
            return
        if b.stop_on_release:
            if self.active_drive_key == k:
                return
            self.active_drive_key = k
            self.start_drive_command(b.command_name)
        else:
            self.send_command(b.command_name)

    def on_key_release(self, event):
        k = event.keysym
        b = KEY_BINDINGS.get(k) or KEY_BINDINGS.get((event.char or "").lower())
        if b is None or not b.stop_on_release:
            return
        if self.active_drive_key == k:
            self.active_drive_key = None
            self.stop_drive()

    # ======================================================================
    # 串口操作
    # ======================================================================
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self.enqueue_log(f"串口刷新：{', '.join(ports) if ports else '未发现设备'}")

    def connect_serial(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("缺少串口", "请先选择 COM 口。")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showwarning("波特率错误", "波特率必须是整数。")
            return
        self.serial_client.connect(port, baud,
                                   auto_reconnect=self._auto_reconnect_var.get())
        self.serial_status_var.set(f"连接中 ({port})")

    def disconnect_serial(self):
        self.serial_client.disconnect()
        self.serial_status_var.set("未连接")
        self.enqueue_log("串口已手动断开")

    # ======================================================================
    # 视频操作
    # ======================================================================
    def start_video(self):
        url = self.video_url_var.get().strip()
        if not url:
            messagebox.showwarning("缺少地址", "请填写视频流地址。")
            return
        self.video_status_var.set("连接中…")
        self.video_client.start(url)

    def stop_video(self):
        self.video_client.stop()
        self.video_status_var.set("已停止")
        self.video_label.configure(image="", text="暂无视频画面")
        if self.fullscreen_video_label:
            self.fullscreen_video_label.configure(image="", text="暂无视频画面")
        self._video_photo = None
        self._fp_video_photo = None

    def open_first_person_mode(self):
        if self.fullscreen_window and self.fullscreen_window.winfo_exists():
            self.fullscreen_window.focus_force()
            return

        win = tk.Toplevel(self.root)
        win.title("TravelCar 第一人称驾驶")
        win.attributes("-fullscreen", True)
        win.configure(bg="#000000")
        win.rowconfigure(0, weight=1)
        win.columnconfigure(0, weight=1)
        self.fullscreen_window = win
        self._bind_fullscreen_keys(win)

        stage = tk.Frame(win, bg="#000000")
        stage.grid(row=0, column=0, sticky="nsew")
        stage.rowconfigure(0, weight=1)
        stage.columnconfigure(0, weight=1)

        self.fullscreen_video_label = tk.Label(
            stage, text="暂无视频画面",
            bg="#000000", fg="#e2e8f0",
            font=("Microsoft YaHei", 28, "bold"),
        )
        self.fullscreen_video_label.grid(row=0, column=0, sticky="nsew")

        # HUD 提示条
        hud = tk.Frame(stage, bg="#000000aa")
        hud.place(relx=0.0, rely=0.0, relwidth=1.0)
        tk.Label(hud,
                 text="第一人称模式   W/A/S/D 移动   U/L 调速   空格/Esc 停止   双击/Esc 退出",
                 bg="#000000", fg=PALETTE["accent"],
                 font=("Microsoft YaHei", 11, "bold"),
                 padx=12, pady=6).pack(fill="x")

        # 状态条（底部）
        foot = tk.Frame(stage, bg="#000000")
        foot.place(relx=0.0, rely=0.94, relwidth=1.0)
        tk.Label(foot, textvariable=self.last_feedback_var,
                 bg="#000000", fg="#93c5fd",
                 font=("Consolas", 11), padx=12, pady=4).pack(side="left")
        tk.Label(foot, textvariable=self.fps_var,
                 bg="#000000", fg=PALETTE["text_muted"],
                 font=("Consolas", 10), padx=12, pady=4).pack(side="right")

        if self.latest_frame_image:
            self._update_fullscreen(self.latest_frame_image)
        win.focus_force()

    def close_first_person_mode(self):
        if self.fullscreen_window and self.fullscreen_window.winfo_exists():
            self.fullscreen_window.destroy()
        self.fullscreen_window = None
        self.fullscreen_video_label = None

    def export_latest_frame(self):
        if self.latest_frame_image is None:
            messagebox.showwarning("暂无画面", "当前没有可导出的视频帧。")
            return
        self.latest_frame_image.save(OPENCLAW_FRAME_PATH, format="JPEG", quality=90)
        self.write_openclaw_status()
        self.enqueue_log(f"已导出帧：{OPENCLAW_FRAME_PATH}")

    # ======================================================================
    # 指令发送
    # ======================================================================
    def start_drive_command(self, cmd: str):
        self.send_command(cmd)

    def stop_drive(self):
        self.send_command("stop_drive")

    def emergency_stop(self):
        self.active_drive_key = None
        self.send_command("estop")

    def send_command(self, cmd_name: str):
        payload = COMMANDS[cmd_name]
        try:
            self.serial_client.send_line(payload)
        except serial.SerialException as exc:
            self.enqueue_log(f"发送失败：{exc}")
            self.serial_status_var.set("未连接")
            return
        self.last_command_var.set(payload)

    def send_manual_command(self):
        payload = self.manual_cmd_var.get().strip()
        if not payload:
            messagebox.showwarning("缺少指令", "请输入指令内容。")
            return
        try:
            self.serial_client.send_line(payload)
        except serial.SerialException as exc:
            self.enqueue_log(f"发送失败：{exc}")
            self.serial_status_var.set("未连接")
            return
        self.last_command_var.set(payload)

    def send_pid_params(self):
        try:
            kp_b = float(self.kp_balance_var.get())
            kd_b = float(self.kd_balance_var.get())
            kp_s = float(self.kp_speed_var.get())
            ki_s = float(self.ki_speed_var.get())
        except ValueError:
            messagebox.showwarning("格式错误", "PID 参数必须是数字。")
            return
        payload = f"PID:{kp_b},{kd_b},{kp_s},{ki_s}"
        try:
            self.serial_client.send_line(payload)
        except serial.SerialException as exc:
            self.enqueue_log(f"PID 下发失败：{exc}")
            self.serial_status_var.set("未连接")
            return
        self.enqueue_log(f"已下发 PID 参数：{payload}")

    # ======================================================================
    # 事件队列 & 处理
    # ======================================================================
    def enqueue_log(self, msg: str):
        self.event_queue.put(("log", msg))

    def enqueue_feedback(self, line: str):
        self.event_queue.put(("feedback", line))

    def enqueue_frame(self, jpeg: bytes):
        self.event_queue.put(("frame", jpeg))

    def process_events(self):
        try:
            while True:
                kind, payload = self.event_queue.get_nowait()
                if kind == "log":
                    self._append_log(str(payload))
                    # 更新串口连接状态
                    if self.serial_client.is_connected:
                        p = self.port_var.get()
                        self.serial_status_var.set(f"已连接 ({p})")
                    else:
                        # 仅在没有手动断开时显示断线
                        cur = self.serial_status_var.get()
                        if cur.startswith("已连接") or cur.startswith("连接中"):
                            self.serial_status_var.set("断线（重连中…）")
                elif kind == "feedback":
                    self._handle_feedback(str(payload))
                elif kind == "frame" and isinstance(payload, bytes):
                    self._render_frame(payload)
                    if self.video_client.is_running:
                        self.video_status_var.set("播放中")
        except queue.Empty:
            pass
        self.root.after(40, self.process_events)

    # ======================================================================
    # 渲染 & 解析
    # ======================================================================
    def _render_frame(self, jpeg: bytes):
        try:
            image = Image.open(io.BytesIO(jpeg)).convert("RGB")
        except Exception as exc:
            self._append_log(f"帧解析失败：{exc}")
            return
        self.latest_frame_image = image.copy()
        self._write_openclaw_frame()

        # HUD 叠加
        display_img = image
        if self.hud_enabled_var.get():
            display_img = draw_hud(image, self._telem, self.video_client.fps)

        # 主窗口视频标签
        w = max(self.video_label.winfo_width(),  640)
        h = max(self.video_label.winfo_height(), 480)
        thumb = ImageOps.contain(display_img, (w, h))
        self._video_photo = ImageTk.PhotoImage(thumb)
        self.video_label.configure(image=self._video_photo, text="")

        self._update_fullscreen(image)

    def _update_fullscreen(self, image: Image.Image):
        if not self.fullscreen_video_label or not self.fullscreen_window \
                or not self.fullscreen_window.winfo_exists():
            return
        w = max(self.fullscreen_window.winfo_width(),  640)
        h = max(self.fullscreen_window.winfo_height(), 480)
        img = image
        if self.hud_enabled_var.get():
            img = draw_hud(image, self._telem, self.video_client.fps)
        frame = ImageOps.contain(img, (w, h))
        self._fp_video_photo = ImageTk.PhotoImage(frame)
        self.fullscreen_video_label.configure(image=self._fp_video_photo, text="")

    def _handle_feedback(self, line: str):
        self.last_feedback_var.set(line)

        if line.startswith("!WARN:"):
            msg = line.removeprefix("!WARN:").strip()
            self.alert_var.set(f"警告：{msg}")
            self._telem.alert = msg
            self._append_log(f"<< 警告：{msg}")
            return

        if line.startswith("!INFO:"):
            msg = line.removeprefix("!INFO:").strip()
            self.alert_var.set(f"信息：{msg}")
            self._telem.alert = msg
            self._append_log(f"<< 信息：{msg}")
            return

        parsed = self._parse_telemetry(line)
        if parsed:
            d = parsed.get("D", "0")
            l = parsed.get("L", "0")
            r = parsed.get("R", "0")
            s = parsed.get("S", "0")

            self.distance_var.set(f"{d} cm")
            self.left_angle_var.set(f"{l} °")
            self.right_angle_var.set(f"{r} °")
            self.speed_var.set(s)
            self.alert_var.set("无")

            # 更新遥测快照
            try:
                self._telem = Telemetry(
                    distance=float(d),
                    left_angle=float(l),
                    right_angle=float(r),
                    speed=int(s),
                    alert="",
                    valid=True,
                )
            except ValueError:
                pass

            # 推送曲线图数据
            self._chart_dist_data.append(self._telem.distance)
            self._chart_speed_data.append(float(self._telem.speed))

        self.write_openclaw_status()
        self._append_log(f"<< {line}")

    def _parse_telemetry(self, line: str) -> dict[str, str] | None:
        parts = [seg.strip() for seg in line.split(",") if ":" in seg]
        if not parts:
            return None
        result = {}
        for part in parts:
            k, _, v = part.partition(":")
            k = k.strip()
            if k in {"D", "L", "R", "S"}:
                result[k] = v.strip()
        return result or None

    def _append_log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # ======================================================================
    # 定时刷新
    # ======================================================================
    def _update_fps_label(self):
        self.fps_var.set(f"{self.video_client.fps:.1f} fps")
        self.root.after(1000, self._update_fps_label)

    def _tick_charts(self):
        if self._chart_dist_data:
            self.chart_dist.push(self._chart_dist_data[-1])
        if self._chart_speed_data:
            self.chart_speed.push(self._chart_speed_data[-1])
        self.root.after(1000 // CHART_FPS, self._tick_charts)

    # ======================================================================
    # OpenClaw 集成
    # ======================================================================
    def _write_openclaw_frame(self):
        if self.latest_frame_image is None:
            return
        try:
            self.latest_frame_image.save(OPENCLAW_FRAME_PATH, format="JPEG", quality=80)
        except Exception as exc:
            self._append_log(f"写入帧失败：{exc}")

    def write_openclaw_status(self):
        payload = {
            "updated_at":    time.strftime("%Y-%m-%d %H:%M:%S"),
            "serial_status": self.serial_status_var.get(),
            "video_status":  self.video_status_var.get(),
            "last_command":  self.last_command_var.get(),
            "last_feedback": self.last_feedback_var.get(),
            "distance":      self.distance_var.get(),
            "left_angle":    self.left_angle_var.get(),
            "right_angle":   self.right_angle_var.get(),
            "speed":         self.speed_var.get(),
            "alert":         self.alert_var.get(),
            "fps":           f"{self.video_client.fps:.1f}",
            "pid": {
                "kp_balance": self.kp_balance_var.get(),
                "kd_balance": self.kd_balance_var.get(),
                "kp_speed":   self.kp_speed_var.get(),
                "ki_speed":   self.ki_speed_var.get(),
            },
            "frame_path": str(OPENCLAW_FRAME_PATH),
        }
        try:
            OPENCLAW_STATUS_PATH.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        except Exception as exc:
            self._append_log(f"写入状态失败：{exc}")

    # ======================================================================
    # 关闭
    # ======================================================================
    def on_close(self):
        try:
            self.video_client.stop()
            self.serial_client.disconnect()
            self.close_first_person_mode()
        finally:
            self.root.destroy()


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------
def main():
    root = tk.Tk()
    app = TravelCarHostApp(root)
    app._append_log("TravelCar 上位机 v2.0 已启动")
    app._append_log("协议：9600 bps  |  w/s/a/d/u/l/q + \\n")
    app._append_log("键盘控制前请先点击窗口获得焦点")
    app._append_log("PID 调参格式：PID:kp_b,kd_b,kp_s,ki_s")
    root.mainloop()


if __name__ == "__main__":
    main()
