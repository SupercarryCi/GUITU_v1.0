# -*- coding: gbk -*-
import math
import os
import re
import tkinter as tk
from tkinter import filedialog
from tkinter import font as tkfont
from tkinter import messagebox
from tkinter import ttk


DATA_RE = re.compile(r"^\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)")
RETURN_RE = re.compile(r"^\s*R\s*(?:\[.*\])?\s*$", re.IGNORECASE)
MIN_SCALE = 2.0
MAX_SCALE = 4000.0
ROUTE_COLOR = "#455a64"
SELECTED_COLOR = "#f57c00"


class TrackPointEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa轨迹逐点编辑器")
        self.root.geometry("1360x900")
        self._setup_fonts()

        self.entries = []
        self.selected_index = None
        self.selected_indices = set()
        self.undo_stack = []
        self.file_path = ""

        self.center_x = 0.0
        self.center_y = 0.0
        self.scale = 50.0

        self.point_drag_start = None
        self.point_drag_original_points = {}
        self.point_drag_undo = None
        self.point_drag_changed = False
        self.box_select_start = None
        self.box_select_current = None
        self.pan_start = None
        self.pan_center = None

        self.path_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="请打开轨迹文件")
        self.point_info_var = tk.StringVar(value="未选择位置点")
        self.zoom_var = tk.StringVar(value="缩放: 50 px/m")
        self.x_var = tk.StringVar(value="")
        self.y_var = tk.StringVar(value="")
        self.z_var = tk.StringVar(value="")
        self.yaw_var = tk.StringVar(value="")
        self.show_index_var = tk.BooleanVar(value=False)
        self.auto_yaw_var = tk.BooleanVar(value=True)
        self.mirror_x_var = tk.BooleanVar(value=True)
        self.selection_mode_var = tk.StringVar(value="单点")

        self._build_ui()

    def _setup_fonts(self):
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont"):
            try:
                tkfont.nametofont(name).configure(size=13)
            except tk.TclError:
                pass
        style = ttk.Style(self.root)
        style.configure("TButton", padding=(8, 5))
        style.configure("TEntry", padding=(3, 4))
        style.configure("TCheckbutton", padding=(4, 4))

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        top.pack(side=tk.TOP, fill=tk.X)

        file_row = ttk.Frame(top)
        file_row.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(file_row, text="打开", command=self.open_file).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(file_row, text="另存为", command=self.save_as).pack(side=tk.LEFT, padx=4)
        ttk.Button(file_row, text="撤销", command=self.undo).pack(side=tk.LEFT, padx=(12, 4))
        ttk.Entry(file_row, textvariable=self.path_var, state="readonly").pack(side=tk.LEFT, padx=(16, 0), fill=tk.X, expand=True)

        view_row = ttk.Frame(top)
        view_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        ttk.Button(view_row, text="－", width=3, command=lambda: self.zoom_at(0.8)).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Button(view_row, text="＋", width=3, command=lambda: self.zoom_at(1.25)).pack(side=tk.LEFT, padx=2)
        ttk.Button(view_row, text="适应窗口", command=self.fit_view).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(view_row, text="选择").pack(side=tk.LEFT)
        selection_box = ttk.Combobox(
            view_row,
            textvariable=self.selection_mode_var,
            values=("单点", "框选"),
            width=6,
            state="readonly",
        )
        selection_box.pack(side=tk.LEFT, padx=(4, 12))
        selection_box.bind("<<ComboboxSelected>>", self.on_selection_mode_changed)
        ttk.Checkbutton(view_row, text="显示序号", variable=self.show_index_var, command=self.draw).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(view_row, text="自动更新航向", variable=self.auto_yaw_var).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(view_row, text="镜像X", variable=self.mirror_x_var, command=self.draw).pack(side=tk.LEFT, padx=4)
        ttk.Label(view_row, textvariable=self.zoom_var).pack(side=tk.LEFT, padx=(12, 8))

        point_row = ttk.Frame(top)
        point_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        ttk.Label(point_row, textvariable=self.point_info_var, width=26).pack(side=tk.LEFT, padx=(0, 12))
        ttk.Label(point_row, text="X(m)").pack(side=tk.LEFT)
        ttk.Entry(point_row, textvariable=self.x_var, width=10).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(point_row, text="Y(m)").pack(side=tk.LEFT)
        ttk.Entry(point_row, textvariable=self.y_var, width=10).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(point_row, text="Z(m)").pack(side=tk.LEFT)
        ttk.Entry(point_row, textvariable=self.z_var, width=10).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(point_row, text="航向(°)").pack(side=tk.LEFT)
        ttk.Entry(point_row, textvariable=self.yaw_var, width=10).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Button(point_row, text="应用数值", command=self.apply_numeric_values).pack(side=tk.LEFT, padx=4)
        ttk.Button(point_row, text="插入点", command=self.insert_point).pack(side=tk.LEFT, padx=4)

        status_row = ttk.Frame(top)
        status_row.pack(side=tk.TOP, fill=tk.X, pady=(6, 0))
        ttk.Label(status_row, textvariable=self.status_var).pack(side=tk.LEFT)

        self.canvas = tk.Canvas(self.root, background="#fbfcfd", highlightthickness=0, takefocus=True)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)
        self.canvas.bind("<ButtonPress-1>", self.on_point_press)
        self.canvas.bind("<B1-Motion>", self.on_point_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_point_release)
        self.canvas.bind("<ButtonPress-2>", self.on_pan_press)
        self.canvas.bind("<B2-Motion>", self.on_pan_drag)
        self.canvas.bind("<ButtonRelease-2>", self.on_pan_release)
        self.canvas.bind("<ButtonPress-3>", self.on_pan_press)
        self.canvas.bind("<B3-Motion>", self.on_pan_drag)
        self.canvas.bind("<ButtonRelease-3>", self.on_pan_release)

    def open_file(self):
        path = filedialog.askopenfilename(
            title="打开轨迹文件",
            filetypes=(("文本文件", "*.txt"), ("所有文件", "*.*")),
        )
        if path:
            self.load_path(path)

    def load_path(self, path):
        try:
            with open(path, "r", encoding="gbk", errors="ignore") as file_obj:
                lines = file_obj.readlines()
        except OSError as exc:
            messagebox.showerror("读取失败", str(exc))
            return False

        entries = []
        return_mode = False
        ignored_count = 0
        for raw_line in lines:
            line = raw_line.strip()
            if not line:
                continue
            if RETURN_RE.match(line):
                entries.append(("return_marker",))
                return_mode = True
                continue
            match = DATA_RE.match(line)
            if match is None:
                ignored_count += 1
                continue
            x_mm, y_mm, z_mm, yaw_cdeg = [int(value) for value in match.groups()]
            point = [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, yaw_cdeg / 100.0]
            entries.append(("point", point, "return" if return_mode else "forward"))

        point_count = sum(1 for entry in entries if entry[0] == "point")
        if point_count == 0:
            messagebox.showerror("文件无效", "文件中没有有效位置点。")
            return False

        self.entries = entries
        self.selected_index = None
        self.selected_indices = set()
        self.undo_stack = []
        self.file_path = path
        self.path_var.set(path)
        self._clear_point_fields()
        self.status_var.set("已加载 %d 个位置点，忽略 %d 行" % (point_count, ignored_count))
        self.root.after_idle(self.fit_view)
        return True

    def save_as(self):
        if not self.entries:
            messagebox.showinfo("另存为", "请先打开轨迹文件。")
            return

        source_name = os.path.basename(self.file_path) if self.file_path else "track.txt"
        base_name, extension = os.path.splitext(source_name)
        if not extension:
            extension = ".txt"
        path = filedialog.asksaveasfilename(
            title="保存修正后的轨迹",
            initialdir=os.path.dirname(self.file_path) if self.file_path else None,
            initialfile=base_name + "_edited" + extension,
            defaultextension=".txt",
            filetypes=(("文本文件", "*.txt"), ("所有文件", "*.*")),
        )
        if not path:
            return

        try:
            with open(path, "w", encoding="gbk", newline="\n") as file_obj:
                for entry in self.entries:
                    if entry[0] == "return_marker":
                        file_obj.write("R\n")
                    else:
                        file_obj.write(self._format_point(entry[1]) + "\n")
        except OSError as exc:
            messagebox.showerror("保存失败", str(exc))
            return

        self.file_path = path
        self.path_var.set(path)
        self.status_var.set("已保存: %s" % path)

    def _format_point(self, point):
        return "%d,%d,%d,%d" % (
            round(point[0] * 1000.0),
            round(point[1] * 1000.0),
            round(point[2] * 1000.0),
            round(point[3] * 100.0),
        )

    def _point_indices(self):
        return [index for index, entry in enumerate(self.entries) if entry[0] == "point"]

    def _point_number(self, entry_index):
        number = 0
        for index, entry in enumerate(self.entries):
            if entry[0] != "point":
                continue
            number += 1
            if index == entry_index:
                return number
        return 0

    def _select_point(self, entry_index):
        self.selected_indices = {entry_index}
        self.selected_index = entry_index
        self._refresh_selection_fields()

    def _set_group_selection(self, entry_indices):
        self.selected_indices = {
            index
            for index in entry_indices
            if 0 <= index < len(self.entries) and self.entries[index][0] == "point"
        }
        self.selected_index = min(self.selected_indices) if self.selected_indices else None
        self._refresh_selection_fields()

    def _refresh_selection_fields(self):
        if not self.selected_indices or self.selected_index is None:
            self.selected_index = None
            self._clear_point_fields()
            return
        if len(self.selected_indices) > 1:
            self.point_info_var.set("已选择 %d 个位置点" % len(self.selected_indices))
            self.x_var.set("")
            self.y_var.set("")
            self.z_var.set("")
            self.yaw_var.set("")
            return

        entry = self.entries[self.selected_index]
        point = entry[1]
        total = len(self._point_indices())
        mode_text = "返航" if entry[2] == "return" else "前进"
        self.point_info_var.set("点 %d/%d  %s" % (self._point_number(self.selected_index), total, mode_text))
        self.x_var.set("%.3f" % point[0])
        self.y_var.set("%.3f" % point[1])
        self.z_var.set("%.3f" % point[2])
        self.yaw_var.set("%.2f" % point[3])

    def _clear_selection(self):
        self.selected_index = None
        self.selected_indices = set()
        self._clear_point_fields()

    def on_selection_mode_changed(self, _event=None):
        self._clear_selection()
        self.box_select_start = None
        self.box_select_current = None
        self.status_var.set("选择模式: %s" % self.selection_mode_var.get())
        self.draw()

    def _clear_point_fields(self):
        self.point_info_var.set("未选择位置点")
        self.x_var.set("")
        self.y_var.set("")
        self.z_var.set("")
        self.yaw_var.set("")

    def apply_numeric_values(self):
        if self.selected_index is None or len(self.selected_indices) != 1:
            messagebox.showinfo("应用数值", "请先选择一个位置点。")
            return
        try:
            new_point = [
                float(self.x_var.get().strip()),
                float(self.y_var.get().strip()),
                float(self.z_var.get().strip()),
                float(self.yaw_var.get().strip()),
            ]
        except ValueError:
            messagebox.showerror("数值无效", "坐标和航向必须是数字。")
            return

        undo_record = {self.selected_index: list(self.entries[self.selected_index][1])}
        self.entries[self.selected_index][1][:] = new_point
        if self.auto_yaw_var.get():
            self._update_yaws_around(self.selected_index, undo_record)
        self._push_undo(undo_record)
        self._select_point(self.selected_index)
        self.status_var.set("已应用位置点数值")
        self.draw()

    def insert_point(self):
        if self.selected_index is None or len(self.selected_indices) != 1:
            messagebox.showinfo("插入点", "请先选择一个位置点。")
            return

        selected_entry = self.entries[self.selected_index]
        selected_point = selected_entry[1]
        insert_index = self.selected_index + 1
        next_point = None
        if insert_index < len(self.entries) and self.entries[insert_index][0] == "point":
            next_point = self.entries[insert_index][1]

        if next_point is not None:
            new_point = [
                (selected_point[0] + next_point[0]) * 0.5,
                (selected_point[1] + next_point[1]) * 0.5,
                (selected_point[2] + next_point[2]) * 0.5,
                selected_point[3],
            ]
        else:
            yaw_rad = math.radians(selected_point[3])
            new_point = [
                selected_point[0] + math.sin(yaw_rad),
                selected_point[1] + math.cos(yaw_rad),
                selected_point[2],
                selected_point[3],
            ]

        self.entries.insert(insert_index, ("point", new_point, selected_entry[2]))
        restore_record = {}
        if self.auto_yaw_var.get():
            self._update_yaws_around(insert_index, restore_record)
            # 插入点会在撤销时删除，不需要再恢复其坐标。
            restore_record.pop(insert_index, None)
        self._push_undo(("insert", insert_index, restore_record))
        self._select_point(insert_index)
        self.status_var.set("已在所选点后插入新位置点")
        self.draw()

    def _push_undo(self, record):
        if not record:
            return
        self.undo_stack.append(record)
        if len(self.undo_stack) > 50:
            del self.undo_stack[0]

    def undo(self):
        if not self.undo_stack:
            self.status_var.set("没有可以撤销的修改")
            return
        record = self.undo_stack.pop()
        if isinstance(record, tuple) and record[0] == "insert":
            _kind, insert_index, restore_record = record
            if 0 <= insert_index < len(self.entries) and self.entries[insert_index][0] == "point":
                del self.entries[insert_index]
            for index, old_point in restore_record.items():
                if 0 <= index < len(self.entries) and self.entries[index][0] == "point":
                    self.entries[index][1][:] = old_point
            previous_index = insert_index - 1
            if 0 <= previous_index < len(self.entries) and self.entries[previous_index][0] == "point":
                self._select_point(previous_index)
            else:
                self._clear_selection()
            self.status_var.set("已撤销插入位置点")
            self.draw()
            return
        for index, old_point in record.items():
            if 0 <= index < len(self.entries) and self.entries[index][0] == "point":
                self.entries[index][1][:] = old_point
        self._refresh_selection_fields()
        self.status_var.set("已撤销上一次修改")
        self.draw()

    def _block_point_indices(self, entry_index):
        start = entry_index
        while start > 0 and self.entries[start - 1][0] == "point":
            start -= 1
        end = entry_index
        while end + 1 < len(self.entries) and self.entries[end + 1][0] == "point":
            end += 1
        return [index for index in range(start, end + 1) if self.entries[index][0] == "point"]

    def _update_yaws_around(self, entry_index, undo_record):
        block = self._block_point_indices(entry_index)
        if len(block) < 2:
            return
        position = block.index(entry_index)
        affected_positions = {position}
        if position > 0:
            affected_positions.add(position - 1)

        for affected_position in affected_positions:
            index = block[affected_position]
            undo_record.setdefault(index, list(self.entries[index][1]))
            if affected_position < len(block) - 1:
                next_index = block[affected_position + 1]
                dx = self.entries[next_index][1][0] - self.entries[index][1][0]
                dy = self.entries[next_index][1][1] - self.entries[index][1][1]
            else:
                previous_index = block[affected_position - 1]
                dx = self.entries[index][1][0] - self.entries[previous_index][1][0]
                dy = self.entries[index][1][1] - self.entries[previous_index][1][1]
            if math.hypot(dx, dy) > 1e-9:
                self.entries[index][1][3] = math.degrees(math.atan2(dx, dy))

    def fit_view(self):
        points = [entry[1] for entry in self.entries if entry[0] == "point"]
        if not points:
            self.center_x = 0.0
            self.center_y = 0.0
            self.scale = 50.0
            self.draw()
            return

        min_x = min(point[0] for point in points)
        max_x = max(point[0] for point in points)
        min_y = min(point[1] for point in points)
        max_y = max(point[1] for point in points)
        self.center_x = (min_x + max_x) * 0.5
        self.center_y = (min_y + max_y) * 0.5
        span_x = max(max_x - min_x, 0.5)
        span_y = max(max_y - min_y, 0.5)
        width = max(self.canvas.winfo_width() - 120, 1)
        height = max(self.canvas.winfo_height() - 120, 1)
        self.scale = max(MIN_SCALE, min(MAX_SCALE, width / span_x, height / span_y))
        self.draw()

    def zoom_at(self, factor, sx=None, sy=None):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        if sx is None:
            sx = width * 0.5
        if sy is None:
            sy = height * 0.5

        anchor_x, anchor_y = self.screen_to_world(sx, sy)
        new_scale = max(MIN_SCALE, min(MAX_SCALE, self.scale * factor))
        if abs(new_scale - self.scale) < 1e-9:
            return
        self.scale = new_scale
        sign_x = -1.0 if self.mirror_x_var.get() else 1.0
        self.center_x = anchor_x - (sx - width * 0.5) / (sign_x * self.scale)
        self.center_y = anchor_y + (sy - height * 0.5) / self.scale
        self.draw()

    def on_mouse_wheel(self, event):
        factor = 1.25 if event.delta > 0 else 0.8
        self.zoom_at(factor, event.x, event.y)

    def world_to_screen(self, x_m, y_m):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        sign_x = -1.0 if self.mirror_x_var.get() else 1.0
        sx = width * 0.5 + sign_x * (x_m - self.center_x) * self.scale
        sy = height * 0.5 - (y_m - self.center_y) * self.scale
        return sx, sy

    def screen_to_world(self, sx, sy):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        sign_x = -1.0 if self.mirror_x_var.get() else 1.0
        x_m = self.center_x + (sx - width * 0.5) / (sign_x * self.scale)
        y_m = self.center_y - (sy - height * 0.5) / self.scale
        return x_m, y_m

    def _nearest_point_index(self, sx, sy, max_distance_px=14.0):
        best_index = None
        best_distance = max_distance_px
        for index, entry in enumerate(self.entries):
            if entry[0] != "point":
                continue
            point_sx, point_sy = self.world_to_screen(entry[1][0], entry[1][1])
            distance = math.hypot(point_sx - sx, point_sy - sy)
            if distance <= best_distance:
                best_distance = distance
                best_index = index
        return best_index

    def _begin_selected_point_drag(self, event):
        self.point_drag_start = self.screen_to_world(event.x, event.y)
        self.point_drag_original_points = {
            index: list(self.entries[index][1])
            for index in self.selected_indices
        }
        self.point_drag_undo = {
            index: list(point)
            for index, point in self.point_drag_original_points.items()
        }
        self.point_drag_changed = False

    def on_point_press(self, event):
        self.canvas.focus_set()
        index = self._nearest_point_index(event.x, event.y)
        if self.selection_mode_var.get() == "框选":
            if index is not None and index in self.selected_indices:
                self._begin_selected_point_drag(event)
                self.status_var.set("正在移动 %d 个位置点" % len(self.selected_indices))
            else:
                self._clear_selection()
                self.box_select_start = (event.x, event.y)
                self.box_select_current = (event.x, event.y)
                self.status_var.set("正在框选位置点")
            self.draw()
            return

        if index is None:
            self._clear_selection()
            self.status_var.set("未选中位置点")
            self.draw()
            return

        self._select_point(index)
        self._begin_selected_point_drag(event)
        self.status_var.set("已选择位置点 %d" % self._point_number(index))
        self.draw()

    def on_point_drag(self, event):
        if self.box_select_start is not None:
            self.box_select_current = (event.x, event.y)
            self.draw()
            return
        if not self.selected_indices or self.point_drag_start is None:
            return
        current_x, current_y = self.screen_to_world(event.x, event.y)
        dx = current_x - self.point_drag_start[0]
        dy = current_y - self.point_drag_start[1]
        if math.hypot(dx, dy) <= 1e-9:
            return
        for index, original_point in self.point_drag_original_points.items():
            point = self.entries[index][1]
            point[0] = original_point[0] + dx
            point[1] = original_point[1] + dy
        self.point_drag_changed = True
        if len(self.selected_indices) == 1:
            point = self.entries[self.selected_index][1]
            self.x_var.set("%.3f" % point[0])
            self.y_var.set("%.3f" % point[1])
        self.draw()

    def on_point_release(self, event):
        if self.box_select_start is not None:
            start_x, start_y = self.box_select_start
            end_x, end_y = self.box_select_current
            min_x = min(start_x, end_x)
            max_x = max(start_x, end_x)
            min_y = min(start_y, end_y)
            max_y = max(start_y, end_y)
            selected = set()
            if max(max_x - min_x, max_y - min_y) < 4.0:
                nearest = self._nearest_point_index(event.x, event.y)
                if nearest is not None:
                    selected.add(nearest)
            else:
                for index, entry in enumerate(self.entries):
                    if entry[0] != "point":
                        continue
                    sx, sy = self.world_to_screen(entry[1][0], entry[1][1])
                    if min_x <= sx <= max_x and min_y <= sy <= max_y:
                        selected.add(index)
            self.box_select_start = None
            self.box_select_current = None
            self._set_group_selection(selected)
            self.status_var.set("框选完成: %d 个位置点" % len(selected))
            self.draw()
            return

        if self.point_drag_start is None:
            return
        if self.point_drag_changed:
            if self.auto_yaw_var.get():
                for index in sorted(self.selected_indices):
                    self._update_yaws_around(index, self.point_drag_undo)
            self._push_undo(self.point_drag_undo)
            self._refresh_selection_fields()
            self.status_var.set("已移动 %d 个位置点" % len(self.selected_indices))
        self.point_drag_start = None
        self.point_drag_original_points = {}
        self.point_drag_undo = None
        self.point_drag_changed = False
        self.draw()

    def on_pan_press(self, event):
        self.pan_start = (event.x, event.y)
        self.pan_center = (self.center_x, self.center_y)
        self.canvas.configure(cursor="fleur")

    def on_pan_drag(self, event):
        if self.pan_start is None:
            return
        dx_px = event.x - self.pan_start[0]
        dy_px = event.y - self.pan_start[1]
        sign_x = -1.0 if self.mirror_x_var.get() else 1.0
        self.center_x = self.pan_center[0] - dx_px / (sign_x * self.scale)
        self.center_y = self.pan_center[1] + dy_px / self.scale
        self.draw()

    def on_pan_release(self, _event):
        self.pan_start = None
        self.pan_center = None
        self.canvas.configure(cursor="")

    def _grid_step(self):
        target_m = 100.0 / max(self.scale, 1e-9)
        exponent = math.floor(math.log10(max(target_m, 1e-9)))
        base = 10.0 ** exponent
        normalized = target_m / base
        if normalized <= 1.0:
            factor = 1.0
        elif normalized <= 2.0:
            factor = 2.0
        elif normalized <= 5.0:
            factor = 5.0
        else:
            factor = 10.0
        return factor * base

    def _draw_grid(self):
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        corners = [
            self.screen_to_world(0, 0),
            self.screen_to_world(width, height),
        ]
        min_x = min(point[0] for point in corners)
        max_x = max(point[0] for point in corners)
        min_y = min(point[1] for point in corners)
        max_y = max(point[1] for point in corners)
        step = self._grid_step()

        x_m = math.floor(min_x / step) * step
        while x_m <= max_x + step * 0.5:
            sx1, sy1 = self.world_to_screen(x_m, min_y)
            sx2, sy2 = self.world_to_screen(x_m, max_y)
            color = "#aeb8c1" if abs(x_m) < step * 1e-6 else "#e2e7eb"
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill=color)
            self.canvas.create_text(sx1 + 4, height - 8, text="%.2g" % x_m, anchor=tk.SW, fill="#71808c")
            x_m += step

        y_m = math.floor(min_y / step) * step
        while y_m <= max_y + step * 0.5:
            sx1, sy1 = self.world_to_screen(min_x, y_m)
            sx2, sy2 = self.world_to_screen(max_x, y_m)
            color = "#aeb8c1" if abs(y_m) < step * 1e-6 else "#e2e7eb"
            self.canvas.create_line(sx1, sy1, sx2, sy2, fill=color)
            self.canvas.create_text(6, sy1 - 2, text="%.2g" % y_m, anchor=tk.SW, fill="#71808c")
            y_m += step

    def _route_segments(self):
        segments = []
        current = []
        for entry in self.entries:
            if entry[0] == "return_marker":
                if current:
                    segments.append(current)
                current = []
            elif entry[0] == "point":
                current.append(entry[1])
        if current:
            segments.append(current)
        return segments

    def draw(self):
        self.canvas.delete("all")
        self.zoom_var.set("缩放: %.0f px/m" % self.scale)
        self._draw_grid()

        for segment in self._route_segments():
            if len(segment) < 2:
                continue
            coords = []
            for point in segment:
                coords.extend(self.world_to_screen(point[0], point[1]))
            self.canvas.create_line(*coords, fill=ROUTE_COLOR, width=2)

        point_number = 0
        return_label_pending = False
        for index, entry in enumerate(self.entries):
            if entry[0] == "return_marker":
                return_label_pending = True
                continue
            point_number += 1
            point = entry[1]
            sx, sy = self.world_to_screen(point[0], point[1])
            selected = index in self.selected_indices
            radius = 7 if selected else 3
            fill = SELECTED_COLOR if selected else "#ffffff"
            outline = SELECTED_COLOR if selected else ROUTE_COLOR
            self.canvas.create_oval(
                sx - radius,
                sy - radius,
                sx + radius,
                sy + radius,
                fill=fill,
                outline=outline,
                width=2,
            )
            if self.show_index_var.get():
                self.canvas.create_text(sx + 7, sy - 7, text=str(point_number), anchor=tk.SW, fill="#263238")
            if return_label_pending:
                self.canvas.create_text(sx + 9, sy + 9, text="R", anchor=tk.NW, fill="#263238")
                return_label_pending = False

        if self.box_select_start is not None and self.box_select_current is not None:
            self.canvas.create_rectangle(
                self.box_select_start[0],
                self.box_select_start[1],
                self.box_select_current[0],
                self.box_select_current[1],
                outline=SELECTED_COLOR,
                width=2,
                dash=(5, 3),
            )


def main():
    root = tk.Tk()
    TrackPointEditor(root)
    root.mainloop()


if __name__ == "__main__":
    main()
