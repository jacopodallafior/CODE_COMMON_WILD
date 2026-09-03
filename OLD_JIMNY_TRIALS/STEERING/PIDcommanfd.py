import serial
import time
import tkinter as tk
import csv
from datetime import datetime

PORT = "COM6"
BAUD = 115200

ANGLE_LIMIT = 600.0
STEP_DEFAULT = 1.0
STEP_MIN = 0.05
STEP_MAX = 20.0
STEP_MULT = 2.0

target_angle = 0.0
step = STEP_DEFAULT
pid_enabled = False

latest_measured_angle = 0.0
latest_error = 0.0
latest_delta = 0.0
latest_count = 0
latest_va = 0.0
latest_vb = 0.0

ser = serial.Serial(PORT, BAUD, timeout=0.05)
time.sleep(2)

log_filename = datetime.now().strftime("pid_log_%Y%m%d_%H%M%S.csv")
log_file = open(log_filename, "w", newline="")
csv_writer = csv.writer(log_file)

csv_writer.writerow([
    "pc_time",
    "arduino_t_ms",
    "target_deg",
    "angle_deg",
    "error_deg",
    "delta_v",
    "va_out",
    "vb_out",
    "codeA",
    "codeB",
    "count",
    "pid_on",
    "p_term",
    "i_term",
    "d_term",
    "isr",
    "invalid"
])
log_file.flush()


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def send_command(cmd: str):
    ser.write((cmd + "\n").encode("ascii"))


def send_target():
    send_command(f"s{target_angle:.2f}")


def update_label():
    label.config(
        text=(
            f"target angle   = {target_angle:+.2f} deg\n"
            f"measured angle = {latest_measured_angle:+.2f} deg\n"
            f"error          = {latest_error:+.2f} deg\n"
            f"delta          = {latest_delta:+.4f} V\n"
            f"va_out         = {latest_va:+.4f} V\n"
            f"vb_out         = {latest_vb:+.4f} V\n"
            f"count          = {latest_count}\n"
            f"step           = {step:.2f} deg\n"
            f"PID            = {'ON' if pid_enabled else 'OFF'}\n\n"
            f"Right  = increase target\n"
            f"Left   = decrease target\n"
            f"Space  = target to zero\n"
            f"E      = enable PID\n"
            f"D      = disable PID\n"
            f"Z      = zero encoder\n"
            f"Up     = bigger step\n"
            f"Down   = smaller step"
        )
    )


def apply_and_refresh():
    send_target()
    update_label()


def on_key(event):
    global target_angle, step, pid_enabled

    if event.keysym == "Right":
        target_angle = clamp(target_angle + step, -ANGLE_LIMIT, ANGLE_LIMIT)
        apply_and_refresh()

    elif event.keysym == "Left":
        target_angle = clamp(target_angle - step, -ANGLE_LIMIT, ANGLE_LIMIT)
        apply_and_refresh()

    elif event.keysym == "space":
        target_angle = 0.0
        send_target()
        update_label()

    elif event.keysym == "Up":
        step = clamp(step * STEP_MULT, STEP_MIN, STEP_MAX)
        update_label()

    elif event.keysym == "Down":
        step = clamp(step / STEP_MULT, STEP_MIN, STEP_MAX)
        update_label()

    elif event.keysym in ("e", "E"):
        pid_enabled = True
        send_command("e")
        update_label()

    elif event.keysym in ("d", "D"):
        pid_enabled = False
        send_command("d")
        update_label()

    elif event.keysym in ("z", "Z"):
        send_command("z")

    elif event.keysym in ("p", "P"):
        send_command("p")


def poll_serial():
    global latest_measured_angle, latest_error, latest_delta
    global latest_count, latest_va, latest_vb, pid_enabled

    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode("ascii", errors="ignore").strip()

            if not line:
                continue

            print(line)

            if line.startswith("DATA,"):
                parts = line.split(",")

                # DATA,t_ms,target_deg,angle_deg,error_deg,delta_v,va_out,vb_out,codeA,codeB,count,pid_on,p_term,i_term,d_term,isr,invalid
                if len(parts) >= 17:
                    try:
                        arduino_t_ms = int(parts[1])
                        target_deg   = float(parts[2])
                        angle_deg    = float(parts[3])
                        error_deg    = float(parts[4])
                        delta_v      = float(parts[5])
                        va_out       = float(parts[6])
                        vb_out       = float(parts[7])
                        codeA        = int(parts[8])
                        codeB        = int(parts[9])
                        count        = int(parts[10])
                        pid_on       = int(parts[11])
                        p_term       = float(parts[12])
                        i_term       = float(parts[13])
                        d_term       = float(parts[14])
                        isr_count    = int(parts[15])
                        invalid_cnt  = int(parts[16])

                        latest_measured_angle = angle_deg
                        latest_error = error_deg
                        latest_delta = delta_v
                        latest_count = count
                        latest_va = va_out
                        latest_vb = vb_out
                        pid_enabled = bool(pid_on)
                        target_angle_local = target_deg

                        csv_writer.writerow([
                            time.time(),
                            arduino_t_ms,
                            target_angle_local,
                            angle_deg,
                            error_deg,
                            delta_v,
                            va_out,
                            vb_out,
                            codeA,
                            codeB,
                            count,
                            pid_on,
                            p_term,
                            i_term,
                            d_term,
                            isr_count,
                            invalid_cnt
                        ])
                        log_file.flush()

                    except ValueError:
                        pass

    except serial.SerialException:
        pass

    update_label()
    root.after(20, poll_serial)


def on_close():
    try:
        log_file.flush()
        log_file.close()
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass

    root.destroy()


root = tk.Tk()
root.title("Steering angle PID control + logger")

label = tk.Label(root, text="", font=("Arial", 14), justify="left", width=40, height=15)
label.pack(padx=20, pady=20)

send_target()
update_label()

root.bind("<KeyPress>", on_key)
root.protocol("WM_DELETE_WINDOW", on_close)
root.after(20, poll_serial)
root.mainloop()