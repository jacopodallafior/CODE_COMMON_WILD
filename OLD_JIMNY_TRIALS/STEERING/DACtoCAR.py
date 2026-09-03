import serial
import time
import tkinter as tk

PORT = "COM6"
BAUD = 115200

DELTA_LIMIT = 0.25
STEP_DEFAULT = 0.005   # 5 mV
STEP_MIN = 0.001
STEP_MAX = 0.050
STEP_MULT = 2.0

delta = 0.0
step = STEP_DEFAULT

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def send_delta():
    global delta
    msg = f"{delta:.4f}\n"
    ser.write(msg.encode("ascii"))

def update_label():
    label.config(
        text=(
            f"delta = {delta:+.4f} V\n"
            f"step  = {step:.4f} V\n\n"
            f"Right  = +delta\n"
            f"Left   = -delta\n"
            f"Space  = reset\n"
            f"Up     = bigger step\n"
            f"Down   = smaller step"
        )
    )

def apply_and_refresh():
    send_delta()
    update_label()

def on_key(event):
    global delta, step

    if event.keysym == "Right":
        delta = clamp(delta + step, -DELTA_LIMIT, DELTA_LIMIT)
        apply_and_refresh()

    elif event.keysym == "Left":
        delta = clamp(delta - step, -DELTA_LIMIT, DELTA_LIMIT)
        apply_and_refresh()

    elif event.keysym == "space":
        delta = 0.0
        ser.write(b"RESET\n")
        update_label()

    elif event.keysym == "Up":
        step = clamp(step * STEP_MULT, STEP_MIN, STEP_MAX)
        update_label()

    elif event.keysym == "Down":
        step = clamp(step / STEP_MULT, STEP_MIN, STEP_MAX)
        update_label()

root = tk.Tk()
root.title("DAC delta control")

label = tk.Label(root, text="", font=("Arial", 14), justify="left", width=28, height=9)
label.pack(padx=20, pady=20)

# manda subito delta zero
send_delta()
update_label()

try:
    root.bind("<KeyPress>", on_key)
    root.mainloop()
finally:
    ser.close()