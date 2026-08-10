import queue
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_RATE = 115200
ROBOT_BT_NAME = "Hexapod-Control"

COLORS = {
    "bg": "#0d1113",
    "panel": "#172024",
    "panel_alt": "#11181b",
    "line": "#2d3a40",
    "text": "#eef6f1",
    "muted": "#94a6a0",
    "green": "#7dff9d",
    "green_dim": "#1c4730",
    "amber": "#ffbf47",
    "blue": "#75b8ff",
    "button": "#243138",
    "button_hover": "#2d424b",
    "danger": "#e45f5f",
}


class RobotBluetoothGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Hexapod Control")
        self.geometry("1120x660")
        self.minsize(980, 580)
        self.configure(bg=COLORS["bg"])

        self.serial_port = None
        self.reader_thread = None
        self.reader_running = False
        self.rx_queue = queue.Queue()

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.cycles_var = tk.IntVar(value=1)
        self.custom_var = tk.StringVar()

        self._build_styles()
        self._build_layout()
        self.refresh_ports()
        self.after(100, self.process_rx_queue)

    def _build_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(
            "Robot.TCombobox",
            fieldbackground=COLORS["panel_alt"],
            background=COLORS["button"],
            foreground=COLORS["text"],
            arrowcolor=COLORS["text"],
            bordercolor=COLORS["line"],
            lightcolor=COLORS["line"],
            darkcolor=COLORS["line"],
            padding=6,
        )
        style.map(
            "Robot.TCombobox",
            fieldbackground=[("readonly", COLORS["panel_alt"])],
            foreground=[("readonly", COLORS["text"])],
        )

    def _build_layout(self):
        header = tk.Frame(self, bg=COLORS["bg"])
        header.pack(fill="x", padx=18, pady=(16, 10))
        tk.Label(
            header,
            text="HEXAPOD CONTROL",
            bg=COLORS["bg"],
            fg=COLORS["text"],
            font=("Segoe UI", 19, "bold"),
        ).pack(side="left")
        tk.Label(
            header,
            text="Bluetooth Serial Console",
            bg=COLORS["bg"],
            fg=COLORS["muted"],
            font=("Segoe UI", 10),
        ).pack(side="left", padx=(14, 0), pady=(6, 0))

        root = tk.Frame(self, bg=COLORS["bg"])
        root.pack(fill="both", expand=True, padx=18, pady=(0, 18))
        root.columnconfigure(0, weight=1, minsize=300)
        root.columnconfigure(1, weight=2, minsize=380)
        root.columnconfigure(2, weight=2, minsize=420)
        root.rowconfigure(0, weight=1)

        self.connection_panel = self.panel(root, "Connection")
        self.connection_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.control_panel = self.panel(root, "Maneuver")
        self.control_panel.grid(row=0, column=1, sticky="nsew", padx=10)

        self.log_panel = self.panel(root, "Command Log & Manual Override")
        self.log_panel.grid(row=0, column=2, sticky="nsew", padx=(10, 0))

        self._build_connection_panel()
        self._build_control_panel()
        self._build_log_panel()

    def panel(self, parent, title):
        frame = tk.Frame(parent, bg=COLORS["panel"], highlightthickness=1, highlightbackground=COLORS["line"])
        frame.columnconfigure(0, weight=1)
        tk.Label(
            frame,
            text=title.upper(),
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w", padx=14, pady=(14, 8))
        self.separator(frame).pack(fill="x", padx=14, pady=(0, 12))
        return frame

    def separator(self, parent):
        return tk.Frame(parent, height=1, bg=COLORS["line"])

    def label(self, parent, text, color=None, font=None):
        return tk.Label(
            parent,
            text=text,
            bg=COLORS["panel"],
            fg=color or COLORS["muted"],
            font=font or ("Segoe UI", 10),
            anchor="w",
            justify="left",
        )

    def button(self, parent, text, command, accent=False, danger=False, height=1):
        bg = COLORS["green_dim"] if accent else COLORS["button"]
        fg = COLORS["green"] if accent else COLORS["text"]
        if danger:
            fg = COLORS["danger"]

        btn = tk.Button(
            parent,
            text=text,
            command=command,
            bg=bg,
            fg=fg,
            activebackground=COLORS["button_hover"],
            activeforeground=COLORS["text"],
            relief="flat",
            bd=0,
            height=height,
            cursor="hand2",
            font=("Segoe UI", 10, "bold"),
            padx=10,
            pady=7,
        )
        btn.bind("<Enter>", lambda _event: btn.configure(bg=COLORS["button_hover"]))
        btn.bind("<Leave>", lambda _event: btn.configure(bg=bg))
        return btn

    def _build_connection_panel(self):
        body = tk.Frame(self.connection_panel, bg=COLORS["panel"])
        body.pack(fill="both", expand=True, padx=14, pady=(0, 14))

        self.label(body, "Bluetooth COM Port").pack(fill="x")
        row = tk.Frame(body, bg=COLORS["panel"])
        row.pack(fill="x", pady=(6, 10))
        self.port_combo = ttk.Combobox(row, textvariable=self.port_var, state="readonly", style="Robot.TCombobox")
        self.port_combo.pack(side="left", fill="x", expand=True)
        self.button(row, "Refresh", self.refresh_ports).pack(side="left", padx=(8, 0))

        self.connect_button = self.button(body, "Connect", self.toggle_connection, accent=True, height=2)
        self.connect_button.pack(fill="x", pady=(0, 14))

        status_box = tk.Frame(body, bg=COLORS["panel_alt"], highlightthickness=1, highlightbackground=COLORS["line"])
        status_box.pack(fill="x", pady=(0, 14))
        self.label(status_box, "Status").pack(fill="x", padx=10, pady=(8, 0))
        tk.Label(
            status_box,
            textvariable=self.status_var,
            bg=COLORS["panel_alt"],
            fg=COLORS["green"],
            font=("Segoe UI", 11, "bold"),
            anchor="w",
        ).pack(fill="x", padx=10, pady=(2, 10))

        self.button(body, "Open Windows Bluetooth Settings", self.open_bluetooth_settings).pack(fill="x", pady=(0, 8))
        self.button(body, "Show Help (?)", lambda: self.send_command("?")).pack(fill="x", pady=4)
        self.button(body, "Re-Home Robot (h)", lambda: self.send_command("h")).pack(fill="x", pady=4)

        self.separator(body).pack(fill="x", pady=16)
        self.label(
            body,
            "Tip: do not choose the USB cable port, such as Silicon Labs CP210x. Choose the port that says Standard Serial over Bluetooth link.",
            font=("Segoe UI", 9),
        ).pack(fill="x")

    def _build_control_panel(self):
        body = tk.Frame(self.control_panel, bg=COLORS["panel"])
        body.pack(fill="both", expand=True, padx=14, pady=(0, 14))

        cycle_card = tk.Frame(body, bg=COLORS["panel_alt"], highlightthickness=1, highlightbackground=COLORS["line"])
        cycle_card.pack(fill="x", pady=(0, 14))
        self.label(cycle_card, "Walk Cycles").pack(side="left", padx=12, pady=12)
        spin = tk.Spinbox(
            cycle_card,
            from_=1,
            to=20,
            textvariable=self.cycles_var,
            width=5,
            bg=COLORS["bg"],
            fg=COLORS["text"],
            buttonbackground=COLORS["button"],
            insertbackground=COLORS["text"],
            relief="flat",
            font=("Segoe UI", 11, "bold"),
            justify="center",
        )
        spin.pack(side="right", padx=12, pady=10)

        pad = tk.Frame(body, bg=COLORS["panel_alt"], highlightthickness=1, highlightbackground=COLORS["line"])
        pad.pack(fill="both", expand=True, pady=(0, 14))
        for i in range(3):
            pad.columnconfigure(i, weight=1, uniform="pad")
            pad.rowconfigure(i, weight=1, uniform="pad")

        self.button(pad, "↑\nFORWARD", self.walk_forward, accent=True, height=4).grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        self.button(pad, "↺\nROTATE LEFT", self.rotate_left, height=4).grid(row=1, column=0, sticky="nsew", padx=8, pady=8)
        self.button(pad, "HOME\nh", lambda: self.send_command("h"), height=4).grid(row=1, column=1, sticky="nsew", padx=8, pady=8)
        self.button(pad, "↻\nROTATE RIGHT", self.rotate_right, height=4).grid(row=1, column=2, sticky="nsew", padx=8, pady=8)

        info = tk.Frame(body, bg=COLORS["panel_alt"], highlightthickness=1, highlightbackground=COLORS["line"])
        info.pack(fill="x")
        self.label(info, "Firmware automatically selects the best gait from available legs.", font=("Segoe UI", 9)).pack(fill="x", padx=10, pady=10)

    def _build_log_panel(self):
        body = tk.Frame(self.log_panel, bg=COLORS["panel"])
        body.pack(fill="both", expand=True, padx=14, pady=(0, 14))

        self.log_text = tk.Text(
            body,
            height=15,
            bg="#090e10",
            fg="#bff4c8",
            insertbackground=COLORS["text"],
            relief="flat",
            bd=0,
            font=("Cascadia Mono", 10),
            wrap="word",
            padx=10,
            pady=10,
        )
        self.log_text.pack(fill="both", expand=True)

        self.label(body, "Lift Leg", color=COLORS["text"], font=("Segoe UI", 11, "bold")).pack(fill="x", pady=(14, 6))
        grid = tk.Frame(body, bg=COLORS["panel"])
        grid.pack(fill="x")
        for idx in range(6):
            leg = idx + 1
            self.button(grid, f"L{leg}", lambda leg=leg: self.send_command(str(leg))).grid(
                row=idx // 3,
                column=idx % 3,
                sticky="ew",
                padx=4,
                pady=4,
            )
        for col in range(3):
            grid.columnconfigure(col, weight=1)

        custom = tk.Frame(body, bg=COLORS["panel"])
        custom.pack(fill="x", pady=(12, 0))
        entry = tk.Entry(
            custom,
            textvariable=self.custom_var,
            bg=COLORS["bg"],
            fg=COLORS["text"],
            insertbackground=COLORS["text"],
            relief="flat",
            font=("Segoe UI", 10),
        )
        entry.pack(side="left", fill="x", expand=True, ipady=9)
        entry.bind("<Return>", lambda _event: self.send_custom())
        self.button(custom, "Send", self.send_custom, accent=True).pack(side="left", padx=(8, 0))

    def refresh_ports(self):
        if list_ports is None:
            self.log("pyserial is not installed. Run: pip install pyserial")
            return

        port_rows = []
        bluetooth_rows = []
        for port in list_ports.comports():
            label = f"{port.device} - {port.description}"
            if "Bluetooth" in port.description or "BTHENUM" in str(port.hwid).upper():
                bluetooth_rows.append(label)
            else:
                port_rows.append(label)

        ports = bluetooth_rows + port_rows
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])
            if bluetooth_rows:
                self.log(f"Bluetooth COM candidate found: {bluetooth_rows[0]}")
            else:
                self.log("No Bluetooth COM port detected. Pair Windows with Hexapod-Control first.")
        else:
            self.port_var.set("")
            self.log("No COM ports found.")

    def open_bluetooth_settings(self):
        try:
            subprocess.Popen(["cmd", "/c", "start", "ms-settings:bluetooth"])
        except OSError as exc:
            self.log(f"Could not open Bluetooth settings: {exc}")

    def selected_port_name(self):
        value = self.port_var.get()
        return value.split(" - ", 1)[0].strip()

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial first:\n\npip install pyserial")
            return

        port = self.selected_port_name()
        if not port:
            messagebox.showwarning("No port selected", "Choose the Bluetooth COM port first.")
            return

        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except serial.SerialException as exc:
            messagebox.showerror(
                "Connection failed",
                f"{exc}\n\nMake sure you selected the Bluetooth COM port, not the USB CP210x port.",
            )
            return

        self.reader_running = True
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()
        self.status_var.set(f"Connected to {port}")
        self.connect_button.configure(text="Disconnect")
        self.log(f"Connected to {port}")

    def disconnect(self):
        self.reader_running = False
        time.sleep(0.1)
        if self.serial_port:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.status_var.set("Disconnected")
        self.connect_button.configure(text="Connect")
        self.log("Disconnected")

    def read_serial_loop(self):
        while self.reader_running and self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode("utf-8", errors="replace").strip()
            except serial.SerialException as exc:
                self.rx_queue.put(f"Read error: {exc}")
                break
            if line:
                self.rx_queue.put(line)

    def process_rx_queue(self):
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self.log(f"< {line}")
        self.after(100, self.process_rx_queue)

    def send_command(self, command):
        if not self.serial_port or not self.serial_port.is_open:
            self.log("Not connected. Select the Bluetooth COM port and click Connect.")
            return

        payload = (command.strip() + "\n").encode("utf-8")
        try:
            self.serial_port.write(payload)
        except serial.SerialException as exc:
            self.log(f"Write error: {exc}")
            return
        self.log(f"> {command.strip()}")

    def send_custom(self):
        command = self.custom_var.get().strip()
        if command:
            self.send_command(command)
            self.custom_var.set("")

    def cycles(self):
        try:
            return max(1, min(20, int(self.cycles_var.get())))
        except tk.TclError:
            return 1

    def walk_forward(self):
        cycles = self.cycles()
        self.send_command("f" if cycles == 1 else f"f{cycles}")

    def rotate_left(self):
        cycles = self.cycles()
        self.send_command("l" if cycles == 1 else f"l{cycles}")

    def rotate_right(self):
        cycles = self.cycles()
        self.send_command("r" if cycles == 1 else f"r{cycles}")

    def log(self, message):
        self.log_text.insert("end", message + "\n")
        self.log_text.see("end")

    def destroy(self):
        self.disconnect()
        super().destroy()


if __name__ == "__main__":
    app = RobotBluetoothGui()
    app.mainloop()
