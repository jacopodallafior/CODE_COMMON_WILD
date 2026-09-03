"""
brake_gui_v2.py — Scalable Brake Motor Test Interface
Requires: pip install pyserial
Run:      python brake_gui_v2.py [--port COMx]

All key parameters (speed, acceleration, max turns, home origin)
are configurable at runtime without re-uploading Arduino code.
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import time
import queue
import argparse

# ─────────────────────────────────────────────
BAUD         = 115200
POLL_MS      = 100
LOG_MAX      = 300

# Colours
BG    = "#0f0f0f"
CARD  = "#1a1a1a"
CARD2 = "#141414"
ACC   = "#e8ff3c"   # yellow-green
GRN   = "#40ff80"
RED   = "#ff4444"
BLU   = "#4da8ff"
DIM   = "#3a3a3a"
FG    = "#d0d0d0"
MUTED = "#666"

# ─────────────────────────────────────────────
#  Serial worker
# ─────────────────────────────────────────────
class SerialWorker:
    def __init__(self, q):
        self.ser = None
        self.q = q
        self._stop = threading.Event()

    def connect(self, port):
        self.disconnect()
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self._stop.clear()
        threading.Thread(target=self._read, daemon=True).start()

    def disconnect(self):
        self._stop.set()
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except: pass
        self.ser = None

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            try: self.ser.write((cmd + "\n").encode())
            except Exception as e: self.q.put(("err", f"Send error: {e}"))

    def connected(self):
        return self.ser is not None and self.ser.is_open

    def _read(self):
        while not self._stop.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.q.put(("rx", line))
            except: break
            time.sleep(0.02)


# ─────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────
class BrakeGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Brake Test v2")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self.q      = queue.Queue()
        self.worker = SerialWorker(self.q)

        # Tk vars
        self.port_var    = tk.StringVar()
        self.brake_var   = tk.IntVar(value=0)
        self.speed_var   = tk.StringVar(value="600")
        self.accel_var   = tk.StringVar(value="400")
        self.turns_var   = tk.StringVar(value="0.80")
        self.jog_var     = tk.StringVar(value="50")
        self.pos_var     = tk.StringVar(value="—")
        self.pct_var     = tk.StringVar(value="—")
        self.status_var  = tk.StringVar(value="DISCONNECTED")

        self._build()
        self._refresh_ports()
        self._poll()

    # ── Build UI ──────────────────────────────

    def _build(self):
        FM = ("Courier New", 10)
        FB = ("Courier New", 10, "bold")
        FS = ("Courier New", 9)
        FL = ("Courier New", 8)

        # ── Connection bar ──
        topbar = tk.Frame(self.root, bg=CARD, pady=7, padx=10)
        topbar.pack(fill="x")

        tk.Label(topbar, text="PORT", bg=CARD, fg=MUTED, font=FL).pack(side="left", padx=(0,4))
        self.port_cb = ttk.Combobox(topbar, textvariable=self.port_var,
                                    width=16, font=FM, state="readonly")
        self.port_cb.pack(side="left", padx=2)
        tk.Button(topbar, text="⟳", bg=CARD, fg=FG, relief="flat",
                  font=("Courier New", 12), cursor="hand2",
                  command=self._refresh_ports).pack(side="left", padx=2)

        self.conn_btn = tk.Button(topbar, text="CONNECT", bg=ACC, fg="#000",
            font=FB, relief="flat", cursor="hand2", padx=10, pady=2,
            command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=8)

        self.status_lbl = tk.Label(topbar, textvariable=self.status_var,
            bg=RED, fg="#000", font=FL, padx=8, pady=2)
        self.status_lbl.pack(side="left")

        # ── Main body ──
        body = tk.Frame(self.root, bg=BG, padx=10, pady=10)
        body.pack(fill="both")

        # Column 1: Brake control
        col1 = tk.Frame(body, bg=BG)
        col1.pack(side="left", fill="both", padx=(0,8))

        # Big readout
        ro = tk.Frame(col1, bg=CARD, pady=10)
        ro.pack(fill="x", pady=(0,6))
        tk.Label(ro, text="BRAKE FORCE", bg=CARD, fg=MUTED, font=FL).pack()
        self.big_lbl = tk.Label(ro, text="0%", bg=CARD, fg=ACC,
                                font=("Courier New", 46, "bold"))
        self.big_lbl.pack()

        # Slider
        sl_frame = tk.Frame(col1, bg=BG, pady=2)
        sl_frame.pack(fill="x")
        style = ttk.Style(); style.theme_use("clam")
        style.configure("B.Horizontal.TScale",
            background=BG, troughcolor=DIM, sliderthickness=26, sliderrelief="flat")
        self.slider = ttk.Scale(sl_frame, from_=0, to=100,
            orient="horizontal", variable=self.brake_var,
            style="B.Horizontal.TScale", command=self._on_slider)
        self.slider.pack(fill="x")

        trow = tk.Frame(sl_frame, bg=BG, height=14)
        trow.pack(fill="x")
        for v in [0,25,50,75,100]:
            tk.Label(trow, text=f"{v}%", bg=BG, fg=DIM,
                     font=FL).place(relx=v/100, anchor="n", y=0)

        # Presets
        tk.Label(col1, text="PRESETS", bg=BG, fg=MUTED, font=FL).pack(
            anchor="w", pady=(8,2))
        pr = tk.Frame(col1, bg=BG)
        pr.pack(fill="x", pady=(0,6))
        for v in [0,25,50,75,100]:
            tk.Button(pr, text=f"{v}%", bg=CARD, fg=FG,
                activebackground=ACC, activeforeground="#000",
                font=FB, relief="flat", cursor="hand2",
                command=lambda x=v: self._set_brake(x)).pack(
                side="left", expand=True, fill="x", padx=2)

        # Apply button
        self.apply_btn = tk.Button(col1, text="▶  APPLY BRAKE",
            bg=ACC, fg="#000", font=FB, relief="flat",
            cursor="hand2", pady=8, command=self._send_brake)
        self.apply_btn.pack(fill="x", pady=(0,4))

        # Home / Disable
        cr = tk.Frame(col1, bg=BG)
        cr.pack(fill="x", pady=(0,6))
        self.home_btn = tk.Button(cr, text="⌂  HOME",
            bg="#1e2e1e", fg=GRN, font=FB, relief="flat",
            cursor="hand2", pady=7,
            command=lambda: self.worker.send("H"))
        self.home_btn.pack(side="left", expand=True, fill="x", padx=(0,3))
        self.off_btn = tk.Button(cr, text="■  DISABLE",
            bg="#2e1e1e", fg=RED, font=FB, relief="flat",
            cursor="hand2", pady=7,
            command=lambda: self.worker.send("OFF"))
        self.off_btn.pack(side="left", expand=True, fill="x", padx=(3,0))

        # Position readout
        pos_frame = tk.Frame(col1, bg=CARD, pady=8, padx=10)
        pos_frame.pack(fill="x")
        tk.Label(pos_frame, text="POSITION", bg=CARD, fg=MUTED, font=FL).grid(
            row=0, column=0, sticky="w")
        tk.Label(pos_frame, textvariable=self.pos_var, bg=CARD,
                 fg=FG, font=FM).grid(row=0, column=1, padx=12, sticky="e")
        tk.Label(pos_frame, text="ACTUAL %", bg=CARD, fg=MUTED, font=FL).grid(
            row=1, column=0, sticky="w")
        tk.Label(pos_frame, textvariable=self.pct_var, bg=CARD,
                 fg=ACC, font=FM).grid(row=1, column=1, padx=12, sticky="e")

        # Column 2: Config
        col2 = tk.Frame(body, bg=BG)
        col2.pack(side="left", fill="both", padx=(0,8))

        def section(parent, title):
            tk.Label(parent, text=title, bg=BG, fg=MUTED,
                     font=FL).pack(anchor="w", pady=(10,4))
            f = tk.Frame(parent, bg=CARD, padx=10, pady=8)
            f.pack(fill="x")
            return f

        def param_row(parent, label, var, unit="", row=0):
            tk.Label(parent, text=label, bg=CARD, fg=MUTED,
                     font=FL, width=10, anchor="w").grid(
                     row=row, column=0, sticky="w", pady=2)
            e = tk.Entry(parent, textvariable=var, bg=DIM, fg=FG,
                         font=FM, width=8, relief="flat",
                         insertbackground=ACC)
            e.grid(row=row, column=1, padx=4)
            if unit:
                tk.Label(parent, text=unit, bg=CARD, fg=MUTED,
                         font=FL).grid(row=row, column=2, sticky="w")
            return e

        # Motion config
        mc = section(col2, "MOTION PARAMETERS")
        param_row(mc, "Speed",  self.speed_var, "steps/s", 0)
        param_row(mc, "Accel",  self.accel_var, "steps/s²", 1)
        param_row(mc, "Turns",  self.turns_var, "rev",      2)

        send_cfg = tk.Button(mc, text="SEND CONFIG",
            bg="#1a1e10", fg=ACC, font=FB, relief="flat",
            cursor="hand2", pady=5, command=self._send_config)
        send_cfg.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6,0))

        # Jog / Home
        hc = section(col2, "JOG + SET HOME")
        tk.Label(hc, text="Jog steps", bg=CARD, fg=MUTED,
                 font=FL, width=10, anchor="w").grid(row=0, column=0, sticky="w", pady=2)
        tk.Entry(hc, textvariable=self.jog_var, bg=DIM, fg=FG,
                 font=FM, width=8, relief="flat",
                 insertbackground=ACC).grid(row=0, column=1, padx=4)

        jr = tk.Frame(hc, bg=CARD)
        jr.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(4,0))
        tk.Button(jr, text="◀ JOG −",
            bg="#1e1e2e", fg=BLU, font=FB, relief="flat",
            cursor="hand2", pady=5,
            command=self._jog_back).pack(side="left", expand=True, fill="x", padx=(0,2))
        tk.Button(jr, text="JOG + ▶",
            bg="#1e1e2e", fg=BLU, font=FB, relief="flat",
            cursor="hand2", pady=5,
            command=self._jog_fwd).pack(side="left", expand=True, fill="x", padx=(2,0))

        self.sethome_btn = tk.Button(hc, text="★  SET HOME HERE",
            bg="#1e2a10", fg=GRN, font=FB, relief="flat",
            cursor="hand2", pady=6, command=self._set_home)
        self.sethome_btn.grid(row=2, column=0, columnspan=3,
                              sticky="ew", pady=(6,0))

        tk.Label(hc, text="Jog to desired zero, then SET HOME HERE.",
                 bg=CARD, fg=MUTED, font=FL, wraplength=180,
                 justify="left").grid(row=3, column=0, columnspan=3,
                                      sticky="w", pady=(4,0))

        # Config query
        qc = section(col2, "QUERY")
        qr = tk.Frame(qc, bg=CARD)
        qr.pack(fill="x")
        tk.Button(qr, text="READ CONFIG", bg=CARD2, fg=FG,
            font=FB, relief="flat", cursor="hand2", pady=5,
            command=lambda: self.worker.send("CONFIG")).pack(
            side="left", expand=True, fill="x", padx=(0,2))
        tk.Button(qr, text="READ POS", bg=CARD2, fg=FG,
            font=FB, relief="flat", cursor="hand2", pady=5,
            command=lambda: self.worker.send("POS")).pack(
            side="left", expand=True, fill="x", padx=(2,0))

        # Column 3: Log
        col3 = tk.Frame(body, bg=BG)
        col3.pack(side="left", fill="both", expand=True)

        tk.Label(col3, text="SERIAL LOG", bg=BG, fg=MUTED,
                 font=FL).pack(anchor="w")

        lf = tk.Frame(col3, bg=CARD)
        lf.pack(fill="both", expand=True)

        self.log = tk.Text(lf, bg=CARD, fg="#666",
            font=("Courier New", 8), relief="flat",
            width=34, height=32, state="disabled",
            insertbackground=ACC, selectbackground=DIM)
        self.log.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        sb = ttk.Scrollbar(lf, command=self.log.yview)
        sb.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=sb.set)

        self.log.tag_config("rx_pos", foreground="#444")
        self.log.tag_config("rx_cmd", foreground=ACC)
        self.log.tag_config("rx_info",foreground=GRN)
        self.log.tag_config("rx_err", foreground=RED)
        self.log.tag_config("tx",     foreground=BLU)

        tk.Button(col3, text="CLEAR", bg=CARD, fg=MUTED,
            font=FL, relief="flat", cursor="hand2",
            command=self._clear_log).pack(anchor="e", pady=2)

        # hint bar
        hint = tk.Frame(self.root, bg="#0a0a0a", pady=4)
        hint.pack(fill="x")
        tk.Label(hint,
            text="JOG to find zero → SET HOME HERE   |   SEND CONFIG to update speed/turns   |   APPLY BRAKE to actuate",
            bg="#0a0a0a", fg="#2a2a2a",
            font=("Courier New", 8)).pack()

        self._set_controls_enabled(False)

    # ── Actions ──────────────────────────────

    def _toggle_connect(self):
        if self.worker.connected():
            self.worker.disconnect()
            self._log_local("Disconnected", "rx_info")
            self._set_connected(False)
        else:
            port = self.port_var.get()
            if not port:
                self._log_local("Select a port first", "rx_err"); return
            try:
                self.worker.connect(port)
                self._log_local(f"Connected → {port}", "rx_info")
                self._set_connected(True)
                time.sleep(0.1)
                self.worker.send("CONFIG")
            except Exception as e:
                self._log_local(f"Error: {e}", "rx_err")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _on_slider(self, val):
        self.big_lbl.config(text=f"{int(float(val))}%")

    def _set_brake(self, val):
        self.brake_var.set(val)
        self.big_lbl.config(text=f"{val}%")
        self._send_brake()

    def _send_brake(self):
        v = self.brake_var.get()
        self.worker.send(f"B:{v}")
        self._log_local(f"→ B:{v}", "tx")

    def _send_config(self):
        try:
            speed = float(self.speed_var.get())
            accel = float(self.accel_var.get())
            turns = float(self.turns_var.get())
        except ValueError:
            self._log_local("ERR: invalid config values", "rx_err"); return

        self.worker.send(f"SPEED:{int(speed)}")
        self.worker.send(f"ACCEL:{int(accel)}")
        self.worker.send(f"TURNS:{turns:.3f}")
        self._log_local(f"→ SPEED:{int(speed)} ACCEL:{int(accel)} TURNS:{turns:.3f}", "tx")

    def _jog_fwd(self):
        try: n = int(self.jog_var.get())
        except: n = 50
        self.worker.send(f"JOG:+{n}")
        self._log_local(f"→ JOG:+{n}", "tx")

    def _jog_back(self):
        try: n = int(self.jog_var.get())
        except: n = 50
        self.worker.send(f"JOG:-{n}")
        self._log_local(f"→ JOG:-{n}", "tx")

    def _set_home(self):
        self.worker.send("SETHOME")
        self._log_local("→ SETHOME (origin set here)", "tx")

    def _set_connected(self, conn):
        if conn:
            self.conn_btn.config(text="DISCONNECT", bg=RED)
            self.status_lbl.config(text="CONNECTED", bg=GRN)
        else:
            self.conn_btn.config(text="CONNECT", bg=ACC)
            self.status_lbl.config(text="DISCONNECTED", bg=RED)
        self._set_controls_enabled(conn)

    def _set_controls_enabled(self, on):
        s = "normal" if on else "disabled"
        for w in [self.apply_btn, self.home_btn,
                  self.off_btn, self.slider, self.sethome_btn]:
            try: w.config(state=s)
            except: pass

    # ── Log ──────────────────────────────────

    def _log_local(self, msg, tag=""):
        ts = time.strftime("%H:%M:%S")
        self.log.config(state="normal")
        self.log.insert("end", f"{ts}  {msg}\n", tag)
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > LOG_MAX:
            self.log.delete("1.0", f"{lines-LOG_MAX}.0")
        self.log.see("end")
        self.log.config(state="disabled")

    def _clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")

    # ── Poll ─────────────────────────────────

    def _poll(self):
        try:
            while True:
                kind, line = self.q.get_nowait()
                if kind == "rx":
                    self._handle_rx(line)
                elif kind == "err":
                    self._log_local(line, "rx_err")
        except queue.Empty:
            pass
        self.root.after(POLL_MS, self._poll)

    def _handle_rx(self, line):
        if line.startswith("POS:"):
            self._parse_pos(line)
            self._log_local(line, "rx_pos")
        elif line.startswith("ERR") or line.startswith("WARN"):
            self._log_local(line, "rx_err")
        elif line.startswith("---") or line.startswith("==="):
            self._log_local(line, "rx_info")
        elif line.startswith("BRAKE:") or line.startswith("JOG:") or \
             line.startswith("SPEED") or line.startswith("ACCEL") or \
             line.startswith("TURNS") or line.startswith("HOME"):
            self._log_local(line, "rx_cmd")
        else:
            self._log_local(line)

    def _parse_pos(self, line):
        try:
            parts = {k: v for k, v in (p.split(":") for p in line.split())}
            self.pos_var.set(parts.get("POS", "—") + " steps")
            self.pct_var.set(parts.get("PCT", "—") + " %")
        except: pass

    def on_close(self):
        self.worker.send("H")
        time.sleep(0.5)
        self.worker.disconnect()
        self.root.destroy()


# ─────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", help="Serial port")
    args = parser.parse_args()

    root = tk.Tk()
    app = BrakeGUI(root)
    if args.port:
        app.port_var.set(args.port)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
