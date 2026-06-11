# -*- coding: gbk -*-
import heapq
import math
import queue
import re
import threading
import tkinter as tk
from tkinter import filedialog
from tkinter import font as tkfont
from tkinter import messagebox
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from c_nav_bridge import CNavEngine
except (ImportError, OSError):
    CNavEngine = None


DATA_RE = re.compile(r"^\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)")
RETURN_RE = re.compile(r"^\s*R\s*(?:\[.*\])?\s*$", re.IGNORECASE)
DEFAULT_BAUD = "115200"
DEFAULT_TURN_ANGLE_DEG = "60"
DEFAULT_MIN_SEGMENT_M = "0.5"
DEFAULT_MERGE_DISTANCE_M = "3"
DEFAULT_POINT_LIMIT = "50"
DEFAULT_PLAY_INTERVAL_MS = "120"
DEFAULT_SNAP_DISTANCE_M = "3.0"
DEFAULT_DIRECTION_MATCH_DEG = "45"
DEFAULT_DIRECTION_BIAS_STRENGTH = "1.0"
DEFAULT_DIRECTION_BIAS_STEP_M = "1.0"
DEFAULT_DIRECTION_BIAS_MOVE_THRESHOLD_M = "0.2"
LARGE_UI_FONT_SIZE = 14
LARGE_UI_HEADING_FONT_SIZE = 15
CANVAS_INFO_FONT = ("Microsoft YaHei UI", 16, "bold")
CANVAS_TEXT_FONT = ("Microsoft YaHei UI", 14)
CANVAS_MARK_FONT = ("Microsoft YaHei UI", 13)


class SerialReader(threading.Thread):
    def __init__(self, port, baudrate, out_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.out_queue = out_queue
        self.stop_event = stop_event

    def run(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=0.2) as ser:
                self.out_queue.put(("status", "connected"))
                while not self.stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue

                    line = raw.decode("ascii", errors="ignore").strip()
                    match = DATA_RE.match(line)
                    if match is None:
                        self.out_queue.put(("ignored", line))
                        continue

                    x_mm, y_mm, z_mm, yaw_cdeg = [int(item) for item in match.groups()]
                    self.out_queue.put(("point", x_mm, y_mm, z_mm, yaw_cdeg, line))
        except Exception as exc:
            self.out_queue.put(("error", str(exc)))
        finally:
            self.out_queue.put(("status", "disconnected"))


class LoraMapViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa位置建图与返航")
        self._setup_large_fonts()
        self.root.geometry("1360x900")

        self.points = []
        self.trace_points = []
        self.key_points = []
        self.graph_nodes = []
        self.graph_edges = {}
        self.key_node_ids = []
        self.return_route = []
        self.home_node_id = None
        self.last_key_node_id = None
        self.total_point_count = 0
        self.file_entries = []
        self.file_index = 0
        self.file_playing = False
        self.file_return_mode = False
        self.direction_bias_last_raw_point = None
        self.direction_bias_last_point = None
        self.direction_bias_changed_count = 0
        self.forward_raw_points = []
        self.forward_snap_points = []
        self.forward_snap_edge = None
        self.forward_snap_changed_count = 0
        self.return_snap_points = []
        self.return_raw_points = []
        self.snap_route_points = []
        self.snap_segment_index = 0
        self.message_queue = queue.Queue()
        self.reader = None
        self.stop_event = None
        self.last_line = ""
        self.status_var = tk.StringVar(value="空闲")
        self.c_engine = None
        self.c_debug = {}

        self.port_var = tk.StringVar(value="COM3")
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)
        self.manual_mode_var = tk.BooleanVar(value=False)
        self.turn_angle_var = tk.StringVar(value=DEFAULT_TURN_ANGLE_DEG)
        self.min_segment_var = tk.StringVar(value=DEFAULT_MIN_SEGMENT_M)
        self.merge_distance_var = tk.StringVar(value=DEFAULT_MERGE_DISTANCE_M)
        self.point_limit_var = tk.StringVar(value=DEFAULT_POINT_LIMIT)
        self.file_path_var = tk.StringVar(value="")
        self.play_interval_var = tk.StringVar(value=DEFAULT_PLAY_INTERVAL_MS)
        self.file_build_map_var = tk.BooleanVar(value=True)
        self.file_navigation_var = tk.BooleanVar(value=True)
        self.direction_bias_var = tk.BooleanVar(value=False)
        self.direction_bias_strength_var = tk.StringVar(value=DEFAULT_DIRECTION_BIAS_STRENGTH)
        self.direction_bias_step_var = tk.StringVar(value=DEFAULT_DIRECTION_BIAS_STEP_M)
        self.direction_bias_move_threshold_var = tk.StringVar(value=DEFAULT_DIRECTION_BIAS_MOVE_THRESHOLD_M)
        self.forward_snap_var = tk.BooleanVar(value=True)
        self.return_snap_var = tk.BooleanVar(value=True)
        self.c_engine_var = tk.BooleanVar(value=False)
        self.mirror_display_var = tk.BooleanVar(value=True)
        self.snap_distance_var = tk.StringVar(value=DEFAULT_SNAP_DISTANCE_M)
        self.direction_match_var = tk.StringVar(value=DEFAULT_DIRECTION_MATCH_DEG)

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.process_queue)

    def _setup_large_fonts(self):
        font_sizes = {
            "TkDefaultFont": LARGE_UI_FONT_SIZE,
            "TkTextFont": LARGE_UI_FONT_SIZE,
            "TkFixedFont": LARGE_UI_FONT_SIZE,
            "TkMenuFont": LARGE_UI_FONT_SIZE,
            "TkHeadingFont": LARGE_UI_HEADING_FONT_SIZE,
            "TkCaptionFont": LARGE_UI_FONT_SIZE,
            "TkSmallCaptionFont": LARGE_UI_FONT_SIZE,
            "TkIconFont": LARGE_UI_FONT_SIZE,
            "TkTooltipFont": LARGE_UI_FONT_SIZE,
        }
        for font_name, font_size in font_sizes.items():
            try:
                tkfont.nametofont(font_name).configure(size=font_size)
            except tk.TclError:
                pass

        style = ttk.Style(self.root)
        style.configure("TButton", padding=(8, 5))
        style.configure("TCheckbutton", padding=(4, 4))
        style.configure("TEntry", padding=(3, 4))
        style.configure("TCombobox", padding=(3, 4))

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        top.pack(side=tk.TOP, fill=tk.X)

        serial_row = ttk.Frame(top)
        serial_row.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(serial_row, text="串口").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(serial_row, textvariable=self.port_var, width=14)
        self.port_box.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(serial_row, text="波特率").pack(side=tk.LEFT)
        self.baud_box = ttk.Combobox(
            serial_row,
            textvariable=self.baud_var,
            width=10,
            values=("9600", "57600", "115200", "230400", "460800", "921600"),
        )
        self.baud_box.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(serial_row, text="刷新串口", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = ttk.Button(serial_row, text="连接", command=self.connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(12, 4))
        self.disconnect_btn = ttk.Button(serial_row, text="断开", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(serial_row, text="清空", command=self.clear_points).pack(side=tk.LEFT, padx=(12, 4))

        status_row = ttk.Frame(top)
        status_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        ttk.Label(status_row, textvariable=self.status_var).pack(side=tk.LEFT)

        map_row = ttk.Frame(top)
        map_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        self.manual_check = ttk.Checkbutton(
            map_row,
            text="手动落点",
            variable=self.manual_mode_var,
            command=self.update_manual_cursor,
        )
        self.manual_check.pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(map_row, text="转角阈值(°)").pack(side=tk.LEFT)
        ttk.Entry(map_row, textvariable=self.turn_angle_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(map_row, text="最短线段(m)").pack(side=tk.LEFT)
        ttk.Entry(map_row, textvariable=self.min_segment_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(map_row, text="节点合并(m)").pack(side=tk.LEFT)
        ttk.Entry(map_row, textvariable=self.merge_distance_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(map_row, text="轨迹缓存(点)").pack(side=tk.LEFT)
        ttk.Entry(map_row, textvariable=self.point_limit_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        map_action_row = ttk.Frame(top)
        map_action_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        ttk.Button(map_action_row, text="应用参数", command=self.update_map_from_ui).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(map_action_row, text="返航路线", command=self.plan_return_route).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(map_action_row, text="C算法", variable=self.c_engine_var, command=self.toggle_c_engine).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Checkbutton(map_action_row, text="镜像显示", variable=self.mirror_display_var, command=self.draw_map).pack(side=tk.LEFT, padx=(12, 4))

        file_row = ttk.Frame(top)
        file_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        ttk.Label(file_row, text="轨迹文件").pack(side=tk.LEFT)
        ttk.Entry(file_row, textvariable=self.file_path_var, width=30).pack(side=tk.LEFT, padx=(4, 4), fill=tk.X, expand=True)
        ttk.Button(file_row, text="选择", command=self.browse_file).pack(side=tk.LEFT, padx=4)
        ttk.Button(file_row, text="加载", command=self.load_file).pack(side=tk.LEFT, padx=4)

        file_control_row = ttk.Frame(top)
        file_control_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        ttk.Button(file_control_row, text="播放", command=self.start_file_playback).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(file_control_row, text="暂停", command=self.pause_file_playback).pack(side=tk.LEFT, padx=4)
        ttk.Button(file_control_row, text="单步", command=self.step_file_playback).pack(side=tk.LEFT, padx=4)
        ttk.Label(file_control_row, text="间隔(ms)").pack(side=tk.LEFT, padx=(12, 0))
        ttk.Entry(file_control_row, textvariable=self.play_interval_var, width=6).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Checkbutton(file_control_row, text="建图", variable=self.file_build_map_var).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(file_control_row, text="导航", variable=self.file_navigation_var).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(file_control_row, text="返航校正", variable=self.return_snap_var).pack(side=tk.LEFT, padx=4)

        snap_row = ttk.Frame(top)
        snap_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        ttk.Checkbutton(snap_row, text="前进校正", variable=self.forward_snap_var).pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(snap_row, text="吸附距离(m)").pack(side=tk.LEFT)
        ttk.Entry(snap_row, textvariable=self.snap_distance_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(snap_row, text="方向阈值(°)").pack(side=tk.LEFT)
        ttk.Entry(snap_row, textvariable=self.direction_match_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        source_row = ttk.Frame(top)
        source_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))

        ttk.Checkbutton(source_row, text="四向原始修正", variable=self.direction_bias_var).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(source_row, text="四向强度(0-1)").pack(side=tk.LEFT)
        ttk.Entry(source_row, textvariable=self.direction_bias_strength_var, width=6).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(source_row, text="四向步长(m)").pack(side=tk.LEFT)
        ttk.Entry(source_row, textvariable=self.direction_bias_step_var, width=6).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(source_row, text="移动阈值(m)").pack(side=tk.LEFT)
        ttk.Entry(source_row, textvariable=self.direction_bias_move_threshold_var, width=6).pack(side=tk.LEFT, padx=(4, 12))

        self.canvas = tk.Canvas(self.root, background="#f7f9fb", highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw_map())
        self.canvas.bind("<Button-1>", self.on_canvas_click)

    def refresh_ports(self):
        if list_ports is None:
            self.port_box["values"] = ()
            self.status_var.set("缺少pyserial: python -m pip install pyserial")
            return

        ports = [item.device for item in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def connect(self):
        if serial is None:
            msg = "需要安装pyserial。\n运行: python -m pip install pyserial"
            self.status_var.set("缺少pyserial")
            messagebox.showerror("缺少依赖", msg)
            return

        if self.reader is not None:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("串口无效", "请输入串口，例如COM3。")
            return

        try:
            baudrate = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("波特率无效", "波特率必须是数字。")
            return

        self.stop_event = threading.Event()
        self.reader = SerialReader(port, baudrate, self.message_queue, self.stop_event)
        self.reader.start()
        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL)
        self.status_var.set("连接中")

    def disconnect(self):
        if self.stop_event is not None:
            self.stop_event.set()
        self.reader = None
        self.stop_event = None
        self.connect_btn.configure(state=tk.NORMAL)
        self.disconnect_btn.configure(state=tk.DISABLED)
        self.status_var.set("已断开")

    def clear_points(self):
        self.file_playing = False
        self.file_index = 0
        self.file_return_mode = False
        self._reset_map_state(keep_points=False)
        self._reset_source_correction_state()
        self.trace_points = []
        self._reset_forward_snap_state()
        self.return_raw_points = []
        self.return_snap_points = []
        self.snap_route_points = []
        self.snap_segment_index = 0
        self.return_route = []
        self.last_line = ""
        self._reset_c_engine(show_error=False)
        self.status_var.set("空闲")
        self.draw_map()

    def toggle_c_engine(self):
        if self.c_engine_var.get():
            if not self._initialize_c_engine(show_error=True):
                self.c_engine_var.set(False)
                return
            self.clear_points()
            self.status_var.set("C算法已启用，地图已清空")
        else:
            self.c_engine = None
            self.c_debug = {}
            self.clear_points()
            self.status_var.set("Python算法已启用，地图已清空")

    def _initialize_c_engine(self, show_error):
        params = self._read_map_params(show_error=show_error)
        snap_params = self._read_snap_params(show_error=show_error)
        if params is None or snap_params is None:
            return False
        if CNavEngine is None:
            if show_error:
                messagebox.showerror("C算法不可用", "无法加载 c_nav_bridge.py。")
            return False

        turn_angle_deg, min_segment_m, merge_distance_m, _point_limit = params
        snap_distance_m, direction_match_deg = snap_params
        try:
            engine = CNavEngine()
            status = engine.initialize(
                turn_angle_deg,
                min_segment_m,
                merge_distance_m,
                snap_distance_m,
                direction_match_deg,
                self.forward_snap_var.get(),
                self.return_snap_var.get(),
            )
        except (OSError, AttributeError) as exc:
            if show_error:
                messagebox.showerror("C算法不可用", "加载 dijkstra.dll 失败:\n%s" % exc)
            return False

        if status != 0:
            if show_error:
                messagebox.showerror("C算法初始化失败", "状态码: %d" % status)
            return False

        self.c_engine = engine
        self.c_debug = engine.debug()
        return True

    def _reset_c_engine(self, show_error):
        if not self.c_engine_var.get():
            self.c_engine = None
            self.c_debug = {}
            return True
        return self._initialize_c_engine(show_error=show_error)

    def _sync_c_map_state(self):
        if self.c_engine is None:
            return

        self.c_debug = self.c_engine.debug()
        self.graph_nodes = self.c_engine.nodes()
        self.graph_edges = {}
        for node_a, node_b in self.c_engine.edges():
            if node_a < 0 or node_b < 0 or node_a >= len(self.graph_nodes) or node_b >= len(self.graph_nodes):
                continue
            weight = self._distance_xy(self.graph_nodes[node_a], self.graph_nodes[node_b])
            self._add_edge(self.graph_edges, node_a, node_b, weight)

        key_count = min(self.c_debug.get("key_count", 0), len(self.graph_nodes))
        self.key_points = [
            (x_m, y_m, 0.0, 0.0)
            for x_m, y_m in self.graph_nodes[:key_count]
        ]
        self.key_node_ids = list(range(key_count))

        home_id = self.c_debug.get("home_node_id", -1)
        last_key_id = self.c_debug.get("last_key_node_id", -1)
        self.home_node_id = home_id if 0 <= home_id < len(self.graph_nodes) else None
        self.last_key_node_id = last_key_id if 0 <= last_key_id < len(self.graph_nodes) else None
        self.return_route = self.c_engine.route()
        self.snap_route_points = list(self.return_route)

    def process_queue(self):
        while True:
            try:
                item = self.message_queue.get_nowait()
            except queue.Empty:
                break

            kind = item[0]
            if kind == "point":
                _kind, x_mm, y_mm, z_mm, yaw_cdeg, line = item
                point = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, yaw_cdeg / 100.0)
                self.add_point(point, line)
            elif kind == "ignored":
                self.last_line = item[1]
            elif kind == "error":
                self.status_var.set("串口错误: %s" % item[1])
                self.connect_btn.configure(state=tk.NORMAL)
                self.disconnect_btn.configure(state=tk.DISABLED)
                self.reader = None
                self.stop_event = None
            elif kind == "status":
                if item[1] == "disconnected" and self.reader is not None:
                    self.disconnect()
                else:
                    self.status_var.set(self._status_text(item[1]))

        self.root.after(50, self.process_queue)

    def _status_text(self, text):
        status_map = {
            "connected": "已连接",
            "disconnected": "已断开",
            "connecting": "连接中",
            "idle": "空闲",
        }
        return status_map.get(text, text)

    def browse_file(self):
        path = filedialog.askopenfilename(
            title="选择轨迹文件",
            filetypes=(("文本文件", "*.txt"), ("所有文件", "*.*")),
        )
        if path:
            self.file_path_var.set(path)
            self.load_file()

    def load_file(self):
        path = self.file_path_var.get().strip()
        if not path:
            messagebox.showerror("文件无效", "请选择或输入轨迹文件路径。")
            return

        try:
            with open(path, "r", encoding="gbk", errors="ignore") as file_obj:
                lines = file_obj.readlines()
        except OSError as exc:
            messagebox.showerror("读取失败", str(exc))
            return

        entries = []
        return_mode = False
        ignored_count = 0
        for raw_line in lines:
            line = raw_line.strip()
            if not line:
                continue

            if RETURN_RE.match(line):
                return_mode = True
                entries.append(("return_marker", None, line))
                continue

            match = DATA_RE.match(line)
            if match is None:
                ignored_count += 1
                continue

            x_mm, y_mm, z_mm, yaw_cdeg = [int(item) for item in match.groups()]
            mode = "return" if return_mode else "forward"
            point = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, yaw_cdeg / 100.0)
            entries.append(("point", point, line, mode))

        self.file_entries = entries
        self.file_index = 0
        self.file_playing = False
        self.file_return_mode = False
        self._reset_file_playback_state()
        forward_count = sum(1 for item in entries if item[0] == "point" and item[3] == "forward")
        return_count = sum(1 for item in entries if item[0] == "point" and item[3] == "return")
        self.status_var.set("文件已加载: 前进 %d  返航 %d  忽略 %d" % (forward_count, return_count, ignored_count))
        self.draw_map()

    def start_file_playback(self):
        if not self.file_entries:
            self.load_file()
        if not self.file_entries:
            return

        if self.file_index >= len(self.file_entries):
            self._reset_file_playback_state()
        elif self.file_index == 0:
            self._reset_file_playback_state()

        self.file_playing = True
        self._schedule_file_step()

    def pause_file_playback(self):
        self.file_playing = False
        self.status_var.set("文件回放已暂停: %d/%d" % (self.file_index, len(self.file_entries)))

    def step_file_playback(self):
        if not self.file_entries:
            self.load_file()
        if not self.file_entries:
            return
        if self.file_index == 0:
            self._reset_file_playback_state()
        self._process_next_file_entry()

    def _reset_file_playback_state(self):
        self.file_index = 0
        self.file_return_mode = False
        self._reset_map_state(keep_points=False)
        self._reset_source_correction_state()
        self.trace_points = []
        self._reset_forward_snap_state()
        self.return_raw_points = []
        self.return_snap_points = []
        self.snap_route_points = []
        self.snap_segment_index = 0
        self.return_route = []
        self.last_line = ""
        self._reset_c_engine(show_error=False)

    def _schedule_file_step(self):
        try:
            interval_ms = int(self.play_interval_var.get().strip())
        except ValueError:
            interval_ms = int(DEFAULT_PLAY_INTERVAL_MS)
        interval_ms = max(1, interval_ms)
        self.root.after(interval_ms, self._file_playback_tick)

    def _file_playback_tick(self):
        if not self.file_playing:
            return

        if not self._process_next_file_entry():
            self.file_playing = False
            self.status_var.set("文件回放完成: %d/%d" % (self.file_index, len(self.file_entries)))
            return

        self._schedule_file_step()

    def _process_next_file_entry(self):
        if self.file_index >= len(self.file_entries):
            return False

        entry = self.file_entries[self.file_index]
        self.file_index += 1
        if entry[0] == "return_marker":
            _kind, _point, line = entry
            self.file_return_mode = True
            self.last_line = line
            if self.c_engine_var.get() and self.c_engine is not None:
                if self.file_navigation_var.get():
                    result = self.c_engine.enter_return()
                    self._sync_c_map_state()
                    self.status_var.set("C返航: %s  路线点: %d" % (result["status_name"], len(self.return_route)))
                else:
                    self.return_route = []
            elif self.file_navigation_var.get() and self.home_node_id is not None and self.graph_nodes:
                self._start_return_navigation()
            else:
                self.return_route = []
            if not self.c_engine_var.get():
                self.status_var.set("检测到返航包: %d/%d" % (self.file_index, len(self.file_entries)))
            self.draw_map()
            return True

        _kind, point, line, mode = entry
        build_map = mode == "forward" and self.file_build_map_var.get()
        update_navigation = mode == "return" and self.file_navigation_var.get()
        self.add_point(
            point,
            line,
            mode=mode,
            build_map=build_map,
            update_navigation=update_navigation,
            from_file=True,
        )
        return True

    def add_point(self, point, line, mode="forward", build_map=True, update_navigation=False, from_file=False):
        if self.c_engine_var.get():
            self._add_point_with_c_engine(point, line, mode, build_map, update_navigation, from_file)
            return

        point = self._apply_source_correction(point)
        params = self._read_map_params(show_error=False)
        self.trace_points.append((point, mode))
        map_point = point
        if mode == "forward":
            self.forward_raw_points.append(point)
            if build_map and params is not None and self.forward_snap_var.get():
                map_point = self._snap_forward_point(point)
            else:
                self.forward_snap_edge = None
            self.forward_snap_points.append(map_point)
            if self._distance_2d(point, map_point) > 1e-6:
                self.forward_snap_changed_count += 1
        if mode == "return":
            self.return_raw_points.append(point)
        self.points.append(map_point)
        self.total_point_count += 1
        self.last_line = line
        if not update_navigation:
            self.return_route = []

        if params is not None:
            turn_angle_deg, min_segment_m, merge_distance_m, point_limit = params
            self._trim_points(point_limit)
            if build_map:
                self._update_incremental_map(turn_angle_deg, min_segment_m, merge_distance_m)
            elif update_navigation:
                snapped_point = None
                if self.return_snap_var.get():
                    snapped_point = self._snap_return_point(point)
                if snapped_point is not None:
                    self.return_snap_points.append(snapped_point)
                    self._update_return_route_from_snap(snapped_point)
                else:
                    self._preview_return_navigation(point, merge_distance_m)
            self.status_var.set("采样: %d  缓存: %d  关键点: %d  地图节点: %d" % (
                self.total_point_count,
                len(self.points),
                len(self.key_points),
                len(self.graph_nodes),
            ))
        self.draw_map()

    def _add_point_with_c_engine(self, point, line, mode, build_map, update_navigation, from_file):
        if self.c_engine is None and not self._initialize_c_engine(show_error=False):
            self.status_var.set("C算法不可用，请重新启用C算法")
            return

        debug_before = self.c_engine.debug()
        effective_mode = "return" if debug_before.get("return_mode", 0) else mode
        self.trace_points.append((point, effective_mode))
        if effective_mode == "forward":
            self.forward_raw_points.append(point)
        else:
            self.return_raw_points.append(point)

        should_process = True
        if from_file:
            should_process = (
                (effective_mode == "forward" and build_map)
                or (effective_mode == "return" and update_navigation)
            )

        corrected_point = point
        result = None
        if should_process:
            result = self.c_engine.input_point(point)
            corrected_point = result["corrected_point"]
            self._sync_c_map_state()

        if effective_mode == "forward":
            self.forward_snap_points.append(corrected_point)
            if self._distance_2d(point, corrected_point) > 1e-6:
                self.forward_snap_changed_count += 1
        elif self.return_snap_var.get():
            self.return_snap_points.append(corrected_point)

        self.points.append(corrected_point)
        self.total_point_count += 1
        self.last_line = line
        params = self._read_map_params(show_error=False)
        if params is not None:
            self._trim_points(params[3])

        if result is None:
            self.status_var.set("C算法未处理此点: 文件建图或导航未开启")
        else:
            last_us = self.c_debug.get("last_process_us", 0)
            max_us = self.c_debug.get("max_process_us", 0)
            checked_edges = self.c_debug.get("checked_edge_count", 0)
            self.status_var.set(
                "C算法: %s  耗时: %dus  最大: %dus  检查边: %d" % (
                    result["status_name"],
                    last_us,
                    max_us,
                    checked_edges,
                )
            )
        self.draw_map()

    def update_manual_cursor(self):
        cursor = "crosshair" if self.manual_mode_var.get() else ""
        self.canvas.configure(cursor=cursor)

    def on_canvas_click(self, event):
        if not self.manual_mode_var.get():
            return

        x_m, y_m = self.screen_to_world(event.x, event.y)
        yaw_deg = 0.0
        if self.points:
            prev_x, prev_y, _prev_z, _prev_yaw = self.points[-1]
            dx = x_m - prev_x
            dy = y_m - prev_y
            if math.hypot(dx, dy) > 1e-6:
                yaw_deg = math.degrees(math.atan2(dx, dy))

        line = "手动落点: %.2f,%.2f" % (x_m, y_m)
        self.add_point((x_m, y_m, 0.0, yaw_deg), line)

    def update_map_from_ui(self):
        params = self._read_map_params(show_error=True)
        if params is None:
            return
        if self.c_engine_var.get():
            if not self._initialize_c_engine(show_error=True):
                return
            self.clear_points()
            self.status_var.set("C算法参数已应用，地图已清空")
            return

        _turn_angle_deg, _min_segment_m, _merge_distance_m, point_limit = params
        self._trim_points(point_limit)
        self.return_route = []
        self.status_var.set("参数已应用: 缓存 %d  关键点 %d  地图节点 %d" % (
            len(self.points),
            len(self.key_points),
            len(self.graph_nodes),
        ))
        self.draw_map()

    def rebuild_map(self, show_error):
        params = self._read_map_params(show_error)
        if params is None:
            return False

        turn_angle_deg, min_segment_m, merge_distance_m, point_limit = params
        cached_points = list(self.points[-point_limit:])
        self._reset_map_state(keep_points=False)
        self._reset_forward_snap_state()
        self.points = []
        for point in cached_points:
            self.points.append(point)
            self._update_incremental_map(turn_angle_deg, min_segment_m, merge_distance_m)
        self.total_point_count = len(cached_points)
        return True

    def _read_map_params(self, show_error):
        try:
            turn_angle_deg = float(self.turn_angle_var.get().strip())
            min_segment_m = float(self.min_segment_var.get().strip())
            merge_distance_m = float(self.merge_distance_var.get().strip())
            point_limit = int(self.point_limit_var.get().strip())
        except ValueError:
            if show_error:
                messagebox.showerror("参数无效", "建图参数必须是数字。")
            else:
                self.status_var.set("建图参数无效")
            return None

        if turn_angle_deg <= 0.0 or min_segment_m < 0.0 or merge_distance_m < 0.0 or point_limit < 3:
            if show_error:
                messagebox.showerror("参数无效", "转角阈值必须大于0，距离参数不能小于0，轨迹缓存至少3点。")
            else:
                self.status_var.set("建图参数无效")
            return None

        return turn_angle_deg, min_segment_m, merge_distance_m, point_limit

    def _read_snap_params(self, show_error):
        try:
            snap_distance_m = float(self.snap_distance_var.get().strip())
            direction_match_deg = float(self.direction_match_var.get().strip())
        except ValueError:
            if show_error:
                messagebox.showerror("参数无效", "校正参数必须是数字。")
            else:
                self.status_var.set("校正参数无效")
            return None

        if snap_distance_m < 0.0 or direction_match_deg <= 0.0 or direction_match_deg > 180.0:
            if show_error:
                messagebox.showerror("参数无效", "吸附距离不能小于0，方向阈值必须在0到180度之间。")
            else:
                self.status_var.set("校正参数无效")
            return None

        return snap_distance_m, direction_match_deg

    def _read_direction_bias_params(self, show_error):
        try:
            strength = float(self.direction_bias_strength_var.get().strip())
            step_m = float(self.direction_bias_step_var.get().strip())
            move_threshold_m = float(self.direction_bias_move_threshold_var.get().strip())
        except ValueError:
            if show_error:
                messagebox.showerror("参数无效", "四向原始修正参数必须是数字。")
            else:
                self.status_var.set("四向原始修正参数无效")
            return None

        if strength < 0.0 or strength > 1.0 or step_m <= 0.0 or move_threshold_m < 0.0:
            if show_error:
                messagebox.showerror("参数无效", "四向强度必须在0到1之间，四向步长必须大于0，移动阈值不能小于0。")
            else:
                self.status_var.set("四向原始修正参数无效")
            return None

        return strength, step_m, move_threshold_m

    def _apply_source_correction(self, point):
        if not self.direction_bias_var.get():
            self.direction_bias_last_raw_point = point
            self.direction_bias_last_point = point
            return point

        params = self._read_direction_bias_params(show_error=False)
        if params is None:
            self.direction_bias_last_raw_point = point
            self.direction_bias_last_point = point
            return point

        strength, step_m, move_threshold_m = params
        x_m, y_m, z_m, yaw_deg = point
        target_yaw = self._nearest_cardinal_yaw(yaw_deg)
        yaw_delta = self._angle_delta_deg(yaw_deg, target_yaw)
        corrected_yaw = self._normalize_yaw_deg(yaw_deg + yaw_delta * strength)

        if self.direction_bias_last_point is None or self.direction_bias_last_raw_point is None or strength <= 0.0:
            corrected_point = (x_m, y_m, z_m, corrected_yaw)
        else:
            last_x, last_y, _last_z, _last_yaw = self.direction_bias_last_point
            raw_move_m = self._distance_2d(self.direction_bias_last_raw_point, point)
            if raw_move_m <= max(move_threshold_m, 1e-6):
                # 掉头或原地转向时，原始坐标没有有效位移，只更新航向，不默认前进一步。
                corrected_point = (last_x, last_y, z_m, corrected_yaw)
            else:
                yaw_rad = math.radians(corrected_yaw)
                # 四向原始修正属于输入层：确认有移动后，用方向和步长生成位置，丢弃本帧原始坐标。
                corrected_point = (
                    last_x + math.sin(yaw_rad) * step_m,
                    last_y + math.cos(yaw_rad) * step_m,
                    z_m,
                    corrected_yaw,
                )

        if self._source_point_changed(point, corrected_point):
            self.direction_bias_changed_count += 1
        self.direction_bias_last_raw_point = point
        self.direction_bias_last_point = corrected_point
        return corrected_point

    def _source_point_changed(self, raw_point, corrected_point):
        if self._distance_2d(raw_point, corrected_point) > 1e-6:
            return True
        yaw_delta = self._angle_delta_deg(raw_point[3], corrected_point[3])
        return abs(yaw_delta) > 1e-6

    def _nearest_cardinal_yaw(self, yaw_deg):
        direction_index = math.floor((yaw_deg + 45.0) / 90.0)
        return self._normalize_yaw_deg(direction_index * 90.0)

    def _angle_delta_deg(self, start_deg, end_deg):
        return self._normalize_yaw_deg(end_deg - start_deg)

    def _normalize_yaw_deg(self, yaw_deg):
        return (yaw_deg + 180.0) % 360.0 - 180.0

    def _reset_source_correction_state(self):
        self.direction_bias_last_raw_point = None
        self.direction_bias_last_point = None
        self.direction_bias_changed_count = 0

    def _reset_forward_snap_state(self):
        self.forward_raw_points = []
        self.forward_snap_points = []
        self.forward_snap_edge = None
        self.forward_snap_changed_count = 0

    def _reset_map_state(self, keep_points):
        if not keep_points:
            self.points = []
            self.total_point_count = 0
        self.key_points = []
        self.graph_nodes = []
        self.graph_edges = {}
        self.key_node_ids = []
        self.home_node_id = None
        self.last_key_node_id = None
        self.snap_route_points = []
        self.snap_segment_index = 0
        self.forward_snap_edge = None

    def _trim_points(self, point_limit):
        if len(self.points) > point_limit:
            del self.points[:-point_limit]

    def _update_incremental_map(self, turn_angle_deg, min_segment_m, merge_distance_m):
        if not self.points:
            return

        if self.home_node_id is None:
            node_id = self._find_or_add_node(self.graph_nodes, self.points[-1], merge_distance_m)
            self.home_node_id = node_id
            self.last_key_node_id = node_id
            self.key_node_ids = [node_id]
            self.key_points = [self.points[-1]]
            return

        if len(self.points) < 3:
            return

        prev_key = self._node_as_point(self.last_key_node_id)
        current = self.points[-2]
        next_point = self.points[-1]
        prev_len = self._distance_2d(prev_key, current)
        next_len = self._distance_2d(current, next_point)
        if prev_len < max(min_segment_m, 1e-6) or next_len < 1e-6:
            return

        angle = self._turn_angle(prev_key, current, next_point)
        if angle >= turn_angle_deg:
            self._commit_key_point(current, merge_distance_m)

    def _node_as_point(self, node_id):
        x_m, y_m = self.graph_nodes[node_id]
        return x_m, y_m, 0.0, 0.0

    def _commit_key_point(self, point, merge_distance_m):
        if self.last_key_node_id is None:
            node_id = self._find_or_add_node(self.graph_nodes, point, merge_distance_m)
            self.home_node_id = node_id
            self.last_key_node_id = node_id
            self.key_node_ids = [node_id]
            self.key_points = [point]
            return node_id

        node_id = self._find_or_add_node(self.graph_nodes, point, merge_distance_m)
        if node_id != self.last_key_node_id:
            # 已确认的边只增量追加，不再随最新点重建拖动。
            self._add_incremental_segment(self.last_key_node_id, node_id, merge_distance_m)
            self.last_key_node_id = node_id
            self.key_node_ids.append(node_id)
            self.key_points.append(point)
        return node_id

    def _commit_current_point(self, merge_distance_m):
        if not self.points:
            return None
        return self._commit_key_point(self.points[-1], merge_distance_m)

    def _start_return_navigation(self):
        params = self._read_map_params(show_error=False)
        if params is None or not self.points:
            return

        _turn_angle_deg, _min_segment_m, merge_distance_m, _point_limit = params
        start_id = self._commit_current_point(merge_distance_m)
        if start_id is None or self.home_node_id is None:
            self.return_route = []
            return

        route_ids, _distance_m = self._shortest_path(start_id, self.home_node_id)
        route_points = self._route_ids_to_points(self.graph_nodes, route_ids)
        self.return_route = route_points
        # 返航校正以关键点图上的最短返航路线为基准，而不是单纯沿首次轨迹反向。
        self.snap_route_points = list(route_points)
        self.snap_segment_index = 0

    def _snap_return_point(self, point):
        snap_params = self._read_snap_params(show_error=False)
        if snap_params is None or len(self.snap_route_points) < 2:
            return None

        snap_distance_m, direction_match_deg = snap_params
        movement = self._current_return_delta()
        predicted = self._predict_corrected_point(point)
        predicted_xy = (predicted[0], predicted[1])

        if movement is None:
            return predicted

        candidate = self._find_snap_segment(predicted_xy, movement, snap_distance_m, direction_match_deg)
        if candidate is None:
            return predicted

        segment_index, proj_x, proj_y = candidate
        self.snap_segment_index = segment_index
        return proj_x, proj_y, point[2], point[3]

    def _current_return_delta(self):
        if len(self.return_raw_points) < 2:
            return None

        prev_point = self.return_raw_points[-2]
        current = self.return_raw_points[-1]
        dx = current[0] - prev_point[0]
        dy = current[1] - prev_point[1]
        if math.hypot(dx, dy) < 1e-6:
            return None
        return dx, dy

    def _predict_corrected_point(self, point):
        if not self.return_snap_points or len(self.return_raw_points) < 2:
            return point

        dx, dy = self._raw_delta_between_last_points()
        last_x, last_y, _last_z, _last_yaw = self.return_snap_points[-1]
        return last_x + dx, last_y + dy, point[2], point[3]

    def _raw_delta_between_last_points(self):
        prev_point = self.return_raw_points[-2]
        current = self.return_raw_points[-1]
        return current[0] - prev_point[0], current[1] - prev_point[1]

    def _find_snap_segment(self, point_xy, movement_xy, snap_distance_m, direction_match_deg):
        if len(self.snap_route_points) < 2:
            return None

        best = None
        best_score = float("inf")
        candidate_indices = self._snap_candidate_indices()
        for segment_index in candidate_indices:
            start_xy = self.snap_route_points[segment_index]
            end_xy = self.snap_route_points[segment_index + 1]
            if not self._direction_matches(movement_xy, start_xy, end_xy, direction_match_deg):
                continue

            projection = self._project_point_on_segment(point_xy, start_xy, end_xy)
            if projection is None:
                continue

            proj_x, proj_y, position, distance_m = projection
            if distance_m > snap_distance_m:
                continue

            # 优先保持当前段，其次才切换到邻近或后续匹配段。
            switch_penalty = abs(segment_index - self.snap_segment_index) * 0.2
            score = distance_m + switch_penalty
            if score < best_score:
                best_score = score
                best = (segment_index, proj_x, proj_y)

        return best

    def _snap_candidate_indices(self):
        last_index = len(self.snap_route_points) - 2
        if last_index < 0:
            return []

        start = max(0, self.snap_segment_index - 1)
        end = min(last_index, self.snap_segment_index + 2)
        indices = list(range(start, end + 1))
        for index in range(0, last_index + 1):
            if index not in indices:
                indices.append(index)
        return indices

    def _direction_matches(self, movement_xy, start_xy, end_xy, direction_match_deg):
        seg_dx = end_xy[0] - start_xy[0]
        seg_dy = end_xy[1] - start_xy[1]
        seg_len = math.hypot(seg_dx, seg_dy)
        move_len = math.hypot(movement_xy[0], movement_xy[1])
        if seg_len < 1e-6 or move_len < 1e-6:
            return False

        cos_value = (movement_xy[0] * seg_dx + movement_xy[1] * seg_dy) / (move_len * seg_len)
        cos_value = max(-1.0, min(1.0, cos_value))
        angle = math.degrees(math.acos(cos_value))
        return angle <= direction_match_deg

    def _update_return_route_from_snap(self, snapped_point):
        if not self.snap_route_points:
            self.return_route = []
            return

        snapped_xy = (snapped_point[0], snapped_point[1])
        next_index = min(self.snap_segment_index + 1, len(self.snap_route_points))
        self.return_route = [snapped_xy] + self.snap_route_points[next_index:]

    def _snap_forward_point(self, point):
        snap_params = self._read_snap_params(show_error=False)
        if snap_params is None or not self.graph_edges:
            self.forward_snap_edge = None
            return point

        predicted = self._predict_forward_point(point)
        movement = self._current_forward_delta()
        if movement is None:
            return predicted

        snap_distance_m, direction_match_deg = snap_params
        predicted_xy = (predicted[0], predicted[1])
        candidate = self._find_forward_snap_edge(
            predicted_xy,
            movement,
            snap_distance_m,
            direction_match_deg,
        )
        if candidate is None:
            return predicted

        a_id, b_id, proj_x, proj_y = candidate
        self.forward_snap_edge = (a_id, b_id)
        return proj_x, proj_y, point[2], point[3]

    def _current_forward_delta(self):
        if len(self.forward_raw_points) < 2:
            return None

        prev_point = self.forward_raw_points[-2]
        current = self.forward_raw_points[-1]
        dx = current[0] - prev_point[0]
        dy = current[1] - prev_point[1]
        if math.hypot(dx, dy) < 1e-6:
            return None
        return dx, dy

    def _predict_forward_point(self, point):
        if not self.forward_snap_points or len(self.forward_raw_points) < 2:
            return point

        dx, dy = self._raw_forward_delta_between_last_points()
        last_x, last_y, _last_z, _last_yaw = self.forward_snap_points[-1]
        return last_x + dx, last_y + dy, point[2], point[3]

    def _raw_forward_delta_between_last_points(self):
        prev_point = self.forward_raw_points[-2]
        current = self.forward_raw_points[-1]
        return current[0] - prev_point[0], current[1] - prev_point[1]

    def _find_forward_snap_edge(self, point_xy, movement_xy, snap_distance_m, direction_match_deg):
        best = None
        best_score = float("inf")
        for a_id, neighbors in self.graph_edges.items():
            for b_id in neighbors:
                if b_id <= a_id:
                    continue

                a_xy = self.graph_nodes[a_id]
                b_xy = self.graph_nodes[b_id]
                if not self._direction_matches_bidirectional(movement_xy, a_xy, b_xy, direction_match_deg):
                    continue

                projection = self._project_point_on_segment(point_xy, a_xy, b_xy)
                if projection is None:
                    continue

                proj_x, proj_y, _position, distance_m = projection
                if distance_m > snap_distance_m:
                    continue

                # 前进校正只读取已有图边，不移动旧节点，避免重复路径把地图拖偏。
                edge_key = (a_id, b_id)
                switch_penalty = 0.0 if edge_key == self.forward_snap_edge else 0.2
                score = distance_m + switch_penalty
                if score < best_score:
                    best_score = score
                    best = (a_id, b_id, proj_x, proj_y)

        return best

    def _direction_matches_bidirectional(self, movement_xy, a_xy, b_xy, direction_match_deg):
        return (
            self._direction_matches(movement_xy, a_xy, b_xy, direction_match_deg)
            or self._direction_matches(movement_xy, b_xy, a_xy, direction_match_deg)
        )

    def _preview_return_navigation(self, point, merge_distance_m):
        if self.home_node_id is None or not self.graph_nodes:
            self.return_route = []
            return

        nodes = list(self.graph_nodes)
        edges = {}
        for node_id, neighbors in self.graph_edges.items():
            edges[node_id] = dict(neighbors)

        start_id = self._find_or_add_node(nodes, point, merge_distance_m)
        if start_id >= len(self.graph_nodes):
            self._connect_temp_point_to_graph(nodes, edges, start_id, merge_distance_m)

        route_ids, _distance_m = self._shortest_path_in_graph(edges, start_id, self.home_node_id)
        self.return_route = self._route_ids_to_points(nodes, route_ids)

    def _connect_temp_point_to_graph(self, nodes, edges, start_id, merge_distance_m):
        start_xy = nodes[start_id]
        best_edge = None
        best_distance = float("inf")

        for a_id, neighbors in edges.items():
            for b_id in neighbors:
                if b_id <= a_id:
                    continue

                projection = self._project_point_on_segment(start_xy, nodes[a_id], nodes[b_id])
                if projection is None:
                    continue

                proj_x, proj_y, position, distance = projection
                if distance < best_distance:
                    best_distance = distance
                    best_edge = (a_id, b_id, proj_x, proj_y, position)

        attach_limit = max(merge_distance_m, 0.5)
        if best_edge is not None and best_distance <= attach_limit:
            a_id, b_id, proj_x, proj_y, position = best_edge
            if position <= 1e-6:
                attach_id = a_id
            elif position >= 1.0 - 1e-6:
                attach_id = b_id
            else:
                attach_id = self._find_or_add_node(nodes, (proj_x, proj_y, 0.0, 0.0), merge_distance_m)
                self._remove_edge(edges, a_id, b_id)
                self._add_edge(edges, a_id, attach_id, self._distance_xy(nodes[a_id], nodes[attach_id]))
                self._add_edge(edges, attach_id, b_id, self._distance_xy(nodes[attach_id], nodes[b_id]))

            self._add_edge(edges, start_id, attach_id, self._distance_xy(nodes[start_id], nodes[attach_id]))
            return

        nearest_id = self._nearest_node_id(nodes, start_xy, exclude_id=start_id)
        if nearest_id is not None:
            self._add_edge(edges, start_id, nearest_id, self._distance_xy(nodes[start_id], nodes[nearest_id]))

    def _project_point_on_segment(self, point_xy, a_xy, b_xy):
        px, py = point_xy
        ax, ay = a_xy
        bx, by = b_xy
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq < 1e-12:
            return None

        position = ((px - ax) * dx + (py - ay) * dy) / length_sq
        position = self._clamp_unit(position)
        proj_x = ax + position * dx
        proj_y = ay + position * dy
        distance = math.hypot(px - proj_x, py - proj_y)
        return proj_x, proj_y, position, distance

    def _nearest_node_id(self, nodes, point_xy, exclude_id=None):
        best_id = None
        best_distance = float("inf")
        for node_id, node_xy in enumerate(nodes):
            if node_id == exclude_id:
                continue

            distance = self._distance_xy(point_xy, node_xy)
            if distance < best_distance:
                best_distance = distance
                best_id = node_id
        return best_id

    def _route_ids_to_points(self, nodes, route_ids):
        return [nodes[node_id] for node_id in route_ids]

    def _simplify_points(self, points, turn_angle_deg, min_segment_m):
        if len(points) <= 2:
            return list(points)

        key_points = [points[0]]
        for index in range(1, len(points) - 1):
            prev_key = key_points[-1]
            current = points[index]
            next_point = points[index + 1]

            prev_len = self._distance_2d(prev_key, current)
            next_len = self._distance_2d(current, next_point)
            if prev_len < max(min_segment_m, 1e-6) or next_len < 1e-6:
                continue

            angle = self._turn_angle(prev_key, current, next_point)
            if angle >= turn_angle_deg:
                key_points.append(current)

        # 终点始终保留，确保返航起点是最新位置点。
        if key_points[-1] is not points[-1]:
            key_points.append(points[-1])
        return key_points

    def _build_graph(self, key_points, merge_distance_m):
        nodes = []
        edges = {}
        key_node_ids = []
        segments = []

        for point in key_points:
            node_id = self._find_or_add_node(nodes, point, merge_distance_m)
            key_node_ids.append(node_id)

        for index in range(1, len(key_node_ids)):
            a_id = key_node_ids[index - 1]
            b_id = key_node_ids[index]
            if a_id == b_id:
                continue

            segments.append((a_id, b_id))

        # 交叉线段需要拆成共享节点，否则视觉相交但最短路不连通。
        self._add_split_segments(nodes, edges, segments, merge_distance_m)
        return nodes, edges, key_node_ids

    def _add_incremental_segment(self, a_id, b_id, merge_distance_m):
        self._add_incremental_segment_to_graph(
            self.graph_nodes,
            self.graph_edges,
            a_id,
            b_id,
            merge_distance_m,
        )

    def _add_incremental_segment_to_graph(self, nodes, edges, a_id, b_id, merge_distance_m):
        if a_id == b_id:
            return

        a_xy = nodes[a_id]
        b_xy = nodes[b_id]
        if self._distance_xy(a_xy, b_xy) < 1e-6:
            return

        new_segment_points = [(0.0, a_id), (1.0, b_id)]
        old_segment_points = {}
        intersection_node_ids = []

        for c_id, neighbors in list(edges.items()):
            for d_id in list(neighbors.keys()):
                if d_id <= c_id:
                    continue
                if a_id in (c_id, d_id) or b_id in (c_id, d_id):
                    continue

                c_xy = nodes[c_id]
                d_xy = nodes[d_id]
                intersection = self._segment_intersection(a_xy, b_xy, c_xy, d_xy)
                if intersection is None:
                    continue

                x_m, y_m, pos_new, pos_old = intersection
                node_id = self._intersection_node_id(
                    nodes,
                    (a_id, b_id),
                    pos_new,
                    (c_id, d_id),
                    pos_old,
                    (x_m, y_m),
                    merge_distance_m,
                )
                new_segment_points.append((pos_new, node_id))
                intersection_node_ids.append(node_id)
                old_segment_points.setdefault((c_id, d_id), [(0.0, c_id), (1.0, d_id)])
                old_segment_points[(c_id, d_id)].append((pos_old, node_id))

        for (c_id, d_id), points_on_segment in old_segment_points.items():
            self._remove_edge(edges, c_id, d_id)
            self._add_ordered_edges(nodes, edges, points_on_segment)

        self._add_ordered_edges(nodes, edges, new_segment_points)
        for node_id in sorted(set(intersection_node_ids)):
            self._cluster_intersection_area(nodes, edges, node_id, merge_distance_m)

    def _add_ordered_edges(self, nodes, edges, points_on_segment):
        ordered = self._ordered_unique_segment_points(points_on_segment)
        for index in range(1, len(ordered)):
            a_id = ordered[index - 1]
            b_id = ordered[index]
            if a_id == b_id:
                continue

            weight = self._distance_xy(nodes[a_id], nodes[b_id])
            if weight > 1e-6:
                self._add_edge(edges, a_id, b_id, weight)

    def _add_split_segments(self, nodes, edges, segments, merge_distance_m):
        segment_points = []
        intersection_node_ids = []
        for a_id, b_id in segments:
            segment_points.append([(0.0, a_id), (1.0, b_id)])

        for index in range(len(segments)):
            a_id, b_id = segments[index]
            a_xy = nodes[a_id]
            b_xy = nodes[b_id]
            if self._distance_xy(a_xy, b_xy) < 1e-6:
                continue

            for other_index in range(index + 1, len(segments)):
                c_id, d_id = segments[other_index]
                if a_id in (c_id, d_id) or b_id in (c_id, d_id):
                    continue

                c_xy = nodes[c_id]
                d_xy = nodes[d_id]
                intersection = self._segment_intersection(a_xy, b_xy, c_xy, d_xy)
                if intersection is None:
                    continue

                x_m, y_m, pos_a, pos_b = intersection
                node_id = self._intersection_node_id(
                    nodes,
                    (a_id, b_id),
                    pos_a,
                    (c_id, d_id),
                    pos_b,
                    (x_m, y_m),
                    merge_distance_m,
                )
                segment_points[index].append((pos_a, node_id))
                segment_points[other_index].append((pos_b, node_id))
                intersection_node_ids.append(node_id)

        for points_on_segment in segment_points:
            ordered = self._ordered_unique_segment_points(points_on_segment)
            for index in range(1, len(ordered)):
                a_id = ordered[index - 1]
                b_id = ordered[index]
                if a_id == b_id:
                    continue

                weight = self._distance_xy(nodes[a_id], nodes[b_id])
                if weight > 1e-6:
                    self._add_edge(edges, a_id, b_id, weight)

        for node_id in sorted(set(intersection_node_ids)):
            self._cluster_intersection_area(nodes, edges, node_id, merge_distance_m)

    def _intersection_node_id(self, nodes, segment_a, pos_a, segment_b, pos_b, intersection_xy, merge_distance_m):
        node_id = self._endpoint_node_id(segment_a, pos_a)
        if node_id is not None:
            return node_id

        node_id = self._endpoint_node_id(segment_b, pos_b)
        if node_id is not None:
            return node_id

        x_m, y_m = intersection_xy
        node_id = self._find_or_add_node(nodes, (x_m, y_m, 0.0, 0.0), merge_distance_m)
        nodes[node_id] = (x_m, y_m)
        return node_id

    def _cluster_intersection_area(self, nodes, edges, center_id, radius_m):
        if radius_m <= 0.0 or center_id >= len(nodes):
            return

        center_xy = nodes[center_id]
        cluster_ids = [
            node_id
            for node_id, node_xy in enumerate(nodes)
            if self._distance_xy(node_xy, center_xy) <= radius_m
        ]
        if len(cluster_ids) <= 1:
            return

        # 路口区域内的多个近邻节点统一到交点，避免八字交叉区被拆成多个小路口。
        for node_id in cluster_ids:
            nodes[node_id] = center_xy

        for node_id in cluster_ids:
            if node_id != center_id:
                self._add_edge(edges, center_id, node_id, 0.0)

        self._refresh_cluster_edge_weights(nodes, edges, cluster_ids)

    def _refresh_cluster_edge_weights(self, nodes, edges, cluster_ids):
        cluster_set = set(cluster_ids)
        for a_id in cluster_ids:
            for b_id in list(edges.get(a_id, {}).keys()):
                if b_id in cluster_set:
                    weight = 0.0
                else:
                    weight = self._distance_xy(nodes[a_id], nodes[b_id])
                edges[a_id][b_id] = weight
                edges.setdefault(b_id, {})
                edges[b_id][a_id] = weight

    def _endpoint_node_id(self, segment, position):
        if position <= 1e-6:
            return segment[0]
        if position >= 1.0 - 1e-6:
            return segment[1]
        return None

    def _ordered_unique_segment_points(self, points_on_segment):
        ordered = []
        seen = set()
        for _position, node_id in sorted(points_on_segment, key=lambda item: item[0]):
            if node_id in seen:
                continue
            seen.add(node_id)
            ordered.append(node_id)
        return ordered

    def _segment_intersection(self, a_xy, b_xy, c_xy, d_xy):
        ax, ay = a_xy
        bx, by = b_xy
        cx, cy = c_xy
        dx, dy = d_xy

        # 使用参数方程求非平行线段交点，返回各自线段上的比例位置。
        rx = bx - ax
        ry = by - ay
        sx = dx - cx
        sy = dy - cy
        denominator = self._cross(rx, ry, sx, sy)
        if abs(denominator) < 1e-9:
            return None

        qpx = cx - ax
        qpy = cy - ay
        pos_a = self._cross(qpx, qpy, sx, sy) / denominator
        pos_b = self._cross(qpx, qpy, rx, ry) / denominator
        if not self._is_unit_range(pos_a) or not self._is_unit_range(pos_b):
            return None

        pos_a = self._clamp_unit(pos_a)
        pos_b = self._clamp_unit(pos_b)
        x_m = ax + pos_a * rx
        y_m = ay + pos_a * ry
        return x_m, y_m, pos_a, pos_b

    def _cross(self, ax, ay, bx, by):
        return ax * by - ay * bx

    def _is_unit_range(self, value):
        return -1e-6 <= value <= 1.0 + 1e-6

    def _clamp_unit(self, value):
        return max(0.0, min(1.0, value))

    def _find_or_add_node(self, nodes, point, merge_distance_m):
        x_m, y_m, _z_m, _yaw_deg = point
        for index, node in enumerate(nodes):
            if self._distance_xy(node, (x_m, y_m)) <= merge_distance_m:
                return index

        nodes.append((x_m, y_m))
        return len(nodes) - 1

    def _add_edge(self, edges, a_id, b_id, weight):
        edges.setdefault(a_id, {})
        edges.setdefault(b_id, {})
        current = edges[a_id].get(b_id)
        if current is None or weight < current:
            edges[a_id][b_id] = weight
            edges[b_id][a_id] = weight

    def _remove_edge(self, edges, a_id, b_id):
        if a_id in edges:
            edges[a_id].pop(b_id, None)
        if b_id in edges:
            edges[b_id].pop(a_id, None)

    def plan_return_route(self):
        if not self.points:
            messagebox.showinfo("返航路线", "还没有位置点。")
            return

        if self.c_engine_var.get():
            if self.c_engine is None and not self._initialize_c_engine(show_error=True):
                return
            result = self.c_engine.enter_return()
            self.file_return_mode = True
            self._sync_c_map_state()
            if result["route_valid"]:
                self.status_var.set(
                    "C返航路线: 下一关键点 %.2fm  路线点: %d" % (
                        result["distance_to_next_m"],
                        len(self.return_route),
                    )
                )
            else:
                self.status_var.set("C返航路线: %s" % result["status_name"])
                messagebox.showwarning("返航路线", "C算法未找到到起点的路线。")
            self.draw_map()
            return

        params = self._read_map_params(show_error=True)
        if params is None:
            return
        _turn_angle_deg, _min_segment_m, merge_distance_m, _point_limit = params

        self.file_navigation_var.set(True)
        start_id = self._commit_current_point(merge_distance_m)
        if start_id is None or self.home_node_id is None:
            messagebox.showinfo("返航路线", "地图节点不足。")
            return

        home_id = self.home_node_id
        route, distance_m = self._shortest_path(start_id, home_id)
        self.return_route = self._route_ids_to_points(self.graph_nodes, route)
        # 手动触发返航时，同样使用图上的最短路线作为校正基准。
        self.snap_route_points = list(self.return_route)
        self.snap_segment_index = 0

        if route:
            self.status_var.set("返航路线: %.2fm  节点: %d" % (distance_m, len(route)))
        else:
            self.status_var.set("未找到返航路线")
            messagebox.showwarning("返航路线", "当前地图中没有找到到起点的路线。")
        self.draw_map()

    def _shortest_path(self, start_id, home_id):
        return self._shortest_path_in_graph(self.graph_edges, start_id, home_id)

    def _shortest_path_in_graph(self, edges, start_id, home_id):
        if start_id == home_id:
            return [home_id], 0.0

        distances = {start_id: 0.0}
        previous = {}
        heap = [(0.0, start_id)]
        visited = set()

        while heap:
            distance, node_id = heapq.heappop(heap)
            if node_id in visited:
                continue
            visited.add(node_id)

            if node_id == home_id:
                break

            for next_id, weight in edges.get(node_id, {}).items():
                new_distance = distance + weight
                if new_distance < distances.get(next_id, float("inf")):
                    distances[next_id] = new_distance
                    previous[next_id] = node_id
                    heapq.heappush(heap, (new_distance, next_id))

        if home_id not in distances:
            return [], 0.0

        route = [home_id]
        while route[-1] != start_id:
            route.append(previous[route[-1]])
        route.reverse()
        return route, distances[home_id]

    def _turn_angle(self, prev_point, current, next_point):
        ax = current[0] - prev_point[0]
        ay = current[1] - prev_point[1]
        bx = next_point[0] - current[0]
        by = next_point[1] - current[1]
        len_a = math.hypot(ax, ay)
        len_b = math.hypot(bx, by)
        if len_a < 1e-6 or len_b < 1e-6:
            return 0.0

        cos_value = (ax * bx + ay * by) / (len_a * len_b)
        cos_value = max(-1.0, min(1.0, cos_value))
        return math.degrees(math.acos(cos_value))

    def _distance_2d(self, a_point, b_point):
        return math.hypot(a_point[0] - b_point[0], a_point[1] - b_point[1])

    def _distance_xy(self, a_xy, b_xy):
        return math.hypot(a_xy[0] - b_xy[0], a_xy[1] - b_xy[1])

    def draw_map(self):
        canvas = self.canvas
        canvas.delete("all")

        view = self._view_params()

        def world_to_screen(x_m, y_m):
            return self.world_to_screen(x_m, y_m, view)

        self._draw_grid(canvas, world_to_screen, view["bounds"])

        self._draw_trace_points(canvas, world_to_screen)

        self._draw_graph(canvas, world_to_screen)
        self._draw_forward_snap_points(canvas, world_to_screen)
        self._draw_active_segment(canvas, world_to_screen)
        self._draw_return_route(canvas, world_to_screen)
        self._draw_return_snap_points(canvas, world_to_screen)
        self._draw_key_points(canvas, world_to_screen)

        if self.points:
            self._draw_current_point(canvas, world_to_screen)

        self._draw_info(canvas)

    def _view_params(self):
        canvas = self.canvas
        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)

        bounds = self._world_bounds()
        min_x, max_x, min_y, max_y = bounds
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        margin = 48
        usable_width = max(width - 2 * margin, 1)
        usable_height = max(height - 2 * margin, 1)
        scale = min(usable_width / span_x, usable_height / span_y)
        scale = max(scale, 1.0)

        return {
            "width": width,
            "height": height,
            "margin": margin,
            "scale": scale,
            "bounds": bounds,
        }

    def world_to_screen(self, x_m, y_m, view):
        min_x, _max_x, min_y, _max_y = view["bounds"]
        margin = view["margin"]
        height = view["height"]
        scale = view["scale"]
        sx = margin + (x_m - min_x) * scale
        if self.mirror_display_var.get():
            sx = view["width"] - sx
        sy = height - margin - (y_m - min_y) * scale
        return sx, sy

    def screen_to_world(self, sx, sy):
        view = self._view_params()
        min_x, _max_x, min_y, _max_y = view["bounds"]
        margin = view["margin"]
        height = view["height"]
        scale = view["scale"]
        if self.mirror_display_var.get():
            sx = view["width"] - sx
        x_m = min_x + (sx - margin) / scale
        y_m = min_y + (height - margin - sy) / scale
        return x_m, y_m

    def _world_bounds(self):
        xs = [0.0]
        ys = [0.0]
        for x_m, y_m, _z_m, _yaw_deg in self.points:
            xs.append(x_m)
            ys.append(y_m)
        for point, _mode in self.trace_points:
            x_m, y_m, _z_m, _yaw_deg = point
            xs.append(x_m)
            ys.append(y_m)
        for x_m, y_m, _z_m, _yaw_deg in self.forward_snap_points:
            xs.append(x_m)
            ys.append(y_m)
        for x_m, y_m, _z_m, _yaw_deg in self.return_snap_points:
            xs.append(x_m)
            ys.append(y_m)
        for x_m, y_m in self.graph_nodes:
            xs.append(x_m)
            ys.append(y_m)
        for x_m, y_m in self.return_route:
            xs.append(x_m)
            ys.append(y_m)

        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        span = max(max_x - min_x, max_y - min_y, 10.0)
        center_x = (min_x + max_x) * 0.5
        center_y = (min_y + max_y) * 0.5
        half = span * 0.5
        pad = span * 0.12
        return center_x - half - pad, center_x + half + pad, center_y - half - pad, center_y + half + pad

    def _draw_grid(self, canvas, world_to_screen, bounds):
        min_x, max_x, min_y, max_y = bounds
        step = self._grid_step(max(max_x - min_x, max_y - min_y))

        start_x = math.floor(min_x / step) * step
        x = start_x
        while x <= max_x:
            sx1, sy1 = world_to_screen(x, min_y)
            sx2, sy2 = world_to_screen(x, max_y)
            color = "#9aa7b2" if abs(x) < 1e-6 else "#d8dee5"
            canvas.create_line(sx1, sy1, sx2, sy2, fill=color)
            if abs(x) > 1e-6:
                canvas.create_text(sx1 + 4, sy1 - 18, text="%.0fm" % x, anchor=tk.W, fill="#6b7785", font=CANVAS_MARK_FONT)
            x += step

        start_y = math.floor(min_y / step) * step
        y = start_y
        while y <= max_y:
            sx1, sy1 = world_to_screen(min_x, y)
            sx2, sy2 = world_to_screen(max_x, y)
            color = "#9aa7b2" if abs(y) < 1e-6 else "#d8dee5"
            canvas.create_line(sx1, sy1, sx2, sy2, fill=color)
            if abs(y) > 1e-6:
                canvas.create_text(sx1 + 4, sy1 - 6, text="%.0fm" % y, anchor=tk.W, fill="#6b7785", font=CANVAS_MARK_FONT)
            y += step

        ox, oy = world_to_screen(0.0, 0.0)
        canvas.create_oval(ox - 4, oy - 4, ox + 4, oy + 4, fill="#263238", outline="")
        canvas.create_text(ox + 8, oy + 8, text="原点", anchor=tk.NW, fill="#263238", font=CANVAS_MARK_FONT)

    def _grid_step(self, span_m):
        raw = span_m / 8.0
        for step in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0):
            if raw <= step:
                return step
        return 200.0

    def _draw_current_point(self, canvas, world_to_screen):
        x_m, y_m, _z_m, yaw_deg = self.points[-1]
        sx, sy = world_to_screen(x_m, y_m)
        canvas.create_oval(sx - 6, sy - 6, sx + 6, sy + 6, fill="#e53935", outline="")

        yaw_rad = math.radians(yaw_deg)
        arrow_len = 36
        ex = sx + math.sin(yaw_rad) * arrow_len
        ey = sy - math.cos(yaw_rad) * arrow_len
        canvas.create_line(sx, sy, ex, ey, fill="#e53935", width=3, arrow=tk.LAST)

    def _draw_graph(self, canvas, world_to_screen):
        for a_id, neighbors in self.graph_edges.items():
            ax, ay = self.graph_nodes[a_id]
            for b_id in neighbors:
                if b_id <= a_id:
                    continue
                bx, by = self.graph_nodes[b_id]
                ax_s, ay_s = world_to_screen(ax, ay)
                bx_s, by_s = world_to_screen(bx, by)
                canvas.create_line(ax_s, ay_s, bx_s, by_s, fill="#2e7d32", width=3)

    def _draw_trace_points(self, canvas, world_to_screen):
        source = self.trace_points
        if not source:
            source = [(point, "forward") for point in self.points]

        last_item = None
        for point, mode in source:
            if last_item is not None:
                last_point, last_mode = last_item
                if last_mode == mode:
                    x1, y1, _z1, _yaw1 = last_point
                    x2, y2, _z2, _yaw2 = point
                    sx1, sy1 = world_to_screen(x1, y1)
                    sx2, sy2 = world_to_screen(x2, y2)
                    if mode == "forward":
                        canvas.create_line(sx1, sy1, sx2, sy2, fill="#90caf9", width=2)
                    else:
                        color = "#ef9a9a" if self.return_snap_points else "#d32f2f"
                        canvas.create_line(sx1, sy1, sx2, sy2, fill=color, width=2, dash=(4, 4))
            last_item = (point, mode)

    def _draw_forward_snap_points(self, canvas, world_to_screen):
        if self.forward_snap_changed_count <= 0 or len(self.forward_snap_points) < 2:
            return

        coords = []
        for x_m, y_m, _z_m, _yaw_deg in self.forward_snap_points:
            coords.extend(world_to_screen(x_m, y_m))
        canvas.create_line(*coords, fill="#0d47a1", width=4)

        for x_m, y_m, _z_m, _yaw_deg in self.forward_snap_points:
            sx, sy = world_to_screen(x_m, y_m)
            canvas.create_oval(sx - 2, sy - 2, sx + 2, sy + 2, fill="#0d47a1", outline="")

    def _draw_return_snap_points(self, canvas, world_to_screen):
        if not self.return_snap_points:
            return

        if len(self.return_snap_points) >= 2:
            coords = []
            for x_m, y_m, _z_m, _yaw_deg in self.return_snap_points:
                coords.extend(world_to_screen(x_m, y_m))
            canvas.create_line(*coords, fill="#b71c1c", width=4)

        for x_m, y_m, _z_m, _yaw_deg in self.return_snap_points:
            sx, sy = world_to_screen(x_m, y_m)
            canvas.create_oval(sx - 3, sy - 3, sx + 3, sy + 3, fill="#b71c1c", outline="")

    def _draw_active_segment(self, canvas, world_to_screen):
        if self.file_return_mode or self.last_key_node_id is None or not self.points:
            return

        start_x, start_y = self.graph_nodes[self.last_key_node_id]
        end_x, end_y, _z_m, _yaw_deg = self.points[-1]
        if math.hypot(end_x - start_x, end_y - start_y) < 1e-6:
            return

        sx1, sy1 = world_to_screen(start_x, start_y)
        sx2, sy2 = world_to_screen(end_x, end_y)
        canvas.create_line(sx1, sy1, sx2, sy2, fill="#f9a825", width=2, dash=(6, 4))

    def _draw_return_route(self, canvas, world_to_screen):
        if len(self.return_route) < 2:
            return

        coords = []
        for x_m, y_m in self.return_route:
            coords.extend(world_to_screen(x_m, y_m))
        canvas.create_line(*coords, fill="#f57c00", width=5, arrow=tk.LAST)

    def _draw_key_points(self, canvas, world_to_screen):
        if not self.graph_nodes:
            return

        home_id = self.home_node_id
        current_id = self.last_key_node_id
        for index, (x_m, y_m) in enumerate(self.graph_nodes):
            sx, sy = world_to_screen(x_m, y_m)
            fill = "#ffffff"
            outline = "#2e7d32"
            radius = 5
            if index == home_id:
                fill = "#1565c0"
                outline = "#1565c0"
                radius = 7
            if index == current_id:
                fill = "#e53935"
                outline = "#e53935"
                radius = 7

            canvas.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, fill=fill, outline=outline, width=2)

        if home_id is not None:
            sx, sy = world_to_screen(*self.graph_nodes[home_id])
            canvas.create_text(sx + 10, sy - 10, text="起点/返航点", anchor=tk.W, fill="#1565c0", font=CANVAS_MARK_FONT)

    def _draw_info(self, canvas):
        pad = 10
        if self.points:
            x_m, y_m, z_m, yaw_deg = self.points[-1]
            text = "X %.2fm   Y %.2fm   Z %.2fm   航向 %.2f°" % (x_m, y_m, z_m, yaw_deg)
        else:
            text = "等待数据: x_mm,y_mm,z_mm,yaw_cdeg"

        canvas.create_text(pad, pad, text=text, anchor=tk.NW, fill="#1f2933", font=CANVAS_INFO_FONT)
        canvas.create_text(pad, pad + 32, text="最近数据: %s" % self.last_line, anchor=tk.NW, fill="#52616b", font=CANVAS_TEXT_FONT)
        canvas.create_text(
            pad,
            pad + 62,
            text="采样点: %d   缓存点: %d   关键点: %d   地图节点: %d" % (
                self.total_point_count,
                len(self.points),
                len(self.key_points),
                len(self.graph_nodes),
            ),
            anchor=tk.NW,
            fill="#52616b",
            font=CANVAS_TEXT_FONT,
        )
        canvas.create_text(
            pad,
            pad + 92,
            text="前进原始点: %d   前进校正点: %d   修正: %d" % (
                len(self.forward_raw_points),
                len(self.forward_snap_points),
                self.forward_snap_changed_count,
            ),
            anchor=tk.NW,
            fill="#52616b",
            font=CANVAS_TEXT_FONT,
        )
        canvas.create_text(
            pad,
            pad + 122,
            text="四向原始修正: %d" % self.direction_bias_changed_count,
            anchor=tk.NW,
            fill="#52616b",
            font=CANVAS_TEXT_FONT,
        )
        canvas.create_text(
            pad,
            pad + 152,
            text="返航原始点: %d   返航校正点: %d" % (
                len(self.return_raw_points),
                len(self.return_snap_points),
            ),
            anchor=tk.NW,
            fill="#52616b",
            font=CANVAS_TEXT_FONT,
        )
        if self.c_engine_var.get():
            canvas.create_text(
                pad,
                pad + 182,
                text="C算法耗时: %dus   最大: %dus   检查边: %d   交点: %d   合并: %d" % (
                    self.c_debug.get("last_process_us", 0),
                    self.c_debug.get("max_process_us", 0),
                    self.c_debug.get("checked_edge_count", 0),
                    self.c_debug.get("intersection_count", 0),
                    self.c_debug.get("merged_node_count", 0),
                ),
                anchor=tk.NW,
                fill="#7b1fa2",
                font=CANVAS_TEXT_FONT,
            )

    def close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    LoraMapViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
