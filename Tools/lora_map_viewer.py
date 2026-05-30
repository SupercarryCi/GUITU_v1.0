import math
import queue
import re
import threading
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


DATA_RE = re.compile(r"^\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)")
DEFAULT_BAUD = "115200"


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
        self.root.title("LoRa Position Map")
        self.root.geometry("980x720")

        self.points = []
        self.message_queue = queue.Queue()
        self.reader = None
        self.stop_event = None
        self.last_line = ""
        self.status_var = tk.StringVar(value="idle")

        self.port_var = tk.StringVar(value="COM3")
        self.baud_var = tk.StringVar(value=DEFAULT_BAUD)

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self.process_queue)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=(8, 8, 8, 4))
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=14)
        self.port_box.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(top, text="Baud").pack(side=tk.LEFT)
        self.baud_box = ttk.Combobox(
            top,
            textvariable=self.baud_var,
            width=10,
            values=("9600", "57600", "115200", "230400", "460800", "921600"),
        )
        self.baud_box.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Button(top, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(12, 4))
        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Clear", command=self.clear_points).pack(side=tk.LEFT, padx=(12, 4))

        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(self.root, background="#f7f9fb", highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw_map())

    def refresh_ports(self):
        if list_ports is None:
            self.port_box["values"] = ()
            self.status_var.set("pyserial missing: python -m pip install pyserial")
            return

        ports = [item.device for item in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def connect(self):
        if serial is None:
            msg = "pyserial is required.\nRun: python -m pip install pyserial"
            self.status_var.set("pyserial missing")
            messagebox.showerror("Missing dependency", msg)
            return

        if self.reader is not None:
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Invalid port", "Enter a serial port, for example COM3.")
            return

        try:
            baudrate = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud rate must be a number.")
            return

        self.stop_event = threading.Event()
        self.reader = SerialReader(port, baudrate, self.message_queue, self.stop_event)
        self.reader.start()
        self.connect_btn.configure(state=tk.DISABLED)
        self.disconnect_btn.configure(state=tk.NORMAL)
        self.status_var.set("connecting")

    def disconnect(self):
        if self.stop_event is not None:
            self.stop_event.set()
        self.reader = None
        self.stop_event = None
        self.connect_btn.configure(state=tk.NORMAL)
        self.disconnect_btn.configure(state=tk.DISABLED)
        self.status_var.set("disconnected")

    def clear_points(self):
        self.points = []
        self.last_line = ""
        self.draw_map()

    def process_queue(self):
        while True:
            try:
                item = self.message_queue.get_nowait()
            except queue.Empty:
                break

            kind = item[0]
            if kind == "point":
                _kind, x_mm, y_mm, z_mm, yaw_cdeg, line = item
                self.points.append((x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0, yaw_cdeg / 100.0))
                self.last_line = line
                self.status_var.set("points: %d" % len(self.points))
                self.draw_map()
            elif kind == "ignored":
                self.last_line = item[1]
            elif kind == "error":
                self.status_var.set("serial error: %s" % item[1])
                self.connect_btn.configure(state=tk.NORMAL)
                self.disconnect_btn.configure(state=tk.DISABLED)
                self.reader = None
                self.stop_event = None
            elif kind == "status":
                if item[1] == "disconnected" and self.reader is not None:
                    self.disconnect()
                else:
                    self.status_var.set(item[1])

        self.root.after(50, self.process_queue)

    def draw_map(self):
        canvas = self.canvas
        canvas.delete("all")

        width = max(canvas.winfo_width(), 1)
        height = max(canvas.winfo_height(), 1)

        bounds = self._world_bounds()
        min_x, max_x, min_y, max_y = bounds
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        margin = 48
        scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)
        scale = max(scale, 1.0)

        def world_to_screen(x_m, y_m):
            sx = margin + (x_m - min_x) * scale
            sy = height - margin - (y_m - min_y) * scale
            return sx, sy

        self._draw_grid(canvas, world_to_screen, bounds)

        if len(self.points) >= 2:
            coords = []
            for x_m, y_m, _z_m, _yaw_deg in self.points:
                coords.extend(world_to_screen(x_m, y_m))
            canvas.create_line(*coords, fill="#1769aa", width=2, smooth=True)

        if self.points:
            self._draw_current_point(canvas, world_to_screen)

        self._draw_info(canvas)

    def _world_bounds(self):
        xs = [0.0]
        ys = [0.0]
        for x_m, y_m, _z_m, _yaw_deg in self.points:
            xs.append(x_m)
            ys.append(y_m)

        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        span = max(max_x - min_x, max_y - min_y, 10.0)
        pad = span * 0.12
        return min_x - pad, max_x + pad, min_y - pad, max_y + pad

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
                canvas.create_text(sx1 + 4, sy1 - 14, text="%.0fm" % x, anchor=tk.W, fill="#6b7785")
            x += step

        start_y = math.floor(min_y / step) * step
        y = start_y
        while y <= max_y:
            sx1, sy1 = world_to_screen(min_x, y)
            sx2, sy2 = world_to_screen(max_x, y)
            color = "#9aa7b2" if abs(y) < 1e-6 else "#d8dee5"
            canvas.create_line(sx1, sy1, sx2, sy2, fill=color)
            if abs(y) > 1e-6:
                canvas.create_text(sx1 + 4, sy1 - 4, text="%.0fm" % y, anchor=tk.W, fill="#6b7785")
            y += step

        ox, oy = world_to_screen(0.0, 0.0)
        canvas.create_oval(ox - 4, oy - 4, ox + 4, oy + 4, fill="#263238", outline="")
        canvas.create_text(ox + 8, oy + 8, text="origin", anchor=tk.NW, fill="#263238")

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

    def _draw_info(self, canvas):
        pad = 10
        if self.points:
            x_m, y_m, z_m, yaw_deg = self.points[-1]
            text = "X %.2fm   Y %.2fm   Z %.2fm   Heading %.2f deg" % (x_m, y_m, z_m, yaw_deg)
        else:
            text = "Waiting for data: x_mm,y_mm,z_mm,yaw_cdeg"

        canvas.create_text(pad, pad, text=text, anchor=tk.NW, fill="#1f2933", font=("Segoe UI", 11, "bold"))
        canvas.create_text(pad, pad + 24, text="Last: %s" % self.last_line, anchor=tk.NW, fill="#52616b")

    def close(self):
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    LoraMapViewer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
