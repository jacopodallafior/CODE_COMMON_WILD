import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_data(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    required = [
        "arduino_t_ms",
        "target_deg",
        "angle_deg",
        "error_deg",
        "delta_v",
        "va_out",
        "vb_out",
        "count",
        "pid_on",
        "p_term",
        "i_term",
        "d_term",
        "isr",
        "invalid",
    ]

    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in CSV: {missing}")

    df = df.copy()

    # Time in seconds from Arduino timebase
    df["t_s"] = (df["arduino_t_ms"] - df["arduino_t_ms"].iloc[0]) / 1000.0

    # Some extra derived quantities
    df["abs_error_deg"] = df["error_deg"].abs()
    df["tm_plus_ts"] = df["va_out"] + df["vb_out"]
    df["tm_minus_ts"] = df["va_out"] - df["vb_out"]

    # Numerical steering speed estimate
    dt = np.diff(df["t_s"].values, prepend=df["t_s"].values[0])
    dangle = np.diff(df["angle_deg"].values, prepend=df["angle_deg"].values[0])
    speed = np.zeros(len(df))
    valid = dt > 1e-9
    speed[valid] = dangle[valid] / dt[valid]
    df["angle_speed_deg_s"] = speed

    return df


def print_summary(df: pd.DataFrame) -> None:
    duration = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    mae = df["abs_error_deg"].mean()
    rmse = np.sqrt(np.mean(df["error_deg"] ** 2))
    max_abs_error = df["abs_error_deg"].max()

    # Steady-state estimate: last 10% of data, minimum 20 samples
    n = len(df)
    tail_n = max(20, int(0.1 * n))
    tail = df.tail(tail_n)
    steady_state_error = tail["error_deg"].mean()

    # Approx overshoot relative to target when target is not zero
    overshoot = np.nan
    if np.max(np.abs(df["target_deg"])) > 1e-9:
        target_final = tail["target_deg"].mean()
        if abs(target_final) > 1e-9:
            if target_final > 0:
                overshoot = df["angle_deg"].max() - target_final
            else:
                overshoot = target_final - df["angle_deg"].min()

    print("\n===== PID POST ANALYSIS SUMMARY =====")
    print(f"Samples                : {len(df)}")
    print(f"Duration [s]           : {duration:.3f}")
    print(f"Mean abs error [deg]   : {mae:.3f}")
    print(f"RMS error [deg]        : {rmse:.3f}")
    print(f"Max abs error [deg]    : {max_abs_error:.3f}")
    print(f"Steady-state error [deg]: {steady_state_error:.3f}")
    if np.isfinite(overshoot):
        print(f"Overshoot estimate [deg]: {overshoot:.3f}")
    print(f"TM+TS mean [V]         : {df['tm_plus_ts'].mean():.4f}")
    print(f"TM+TS std [V]          : {df['tm_plus_ts'].std():.6f}")
    print(f"TM-TS min/max [V]      : {df['tm_minus_ts'].min():.4f} / {df['tm_minus_ts'].max():.4f}")
    print("=====================================\n")


def plot_time_series(df: pd.DataFrame, out_dir: str | None = None) -> None:
    t = df["t_s"]

    # 1. Target and measured angle
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["target_deg"], label="Target angle [deg]")
    plt.plot(t, df["angle_deg"], label="Measured angle [deg]")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.title("Target vs Measured Angle")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "01_target_vs_angle.png"), dpi=150)

    # 2. Error
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["error_deg"], label="Error [deg]")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [deg]")
    plt.title("Control Error")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "02_error.png"), dpi=150)

    # 3. Delta and DAC voltages
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["delta_v"], label="Delta [V]")
    plt.plot(t, df["va_out"], label="TM / VA [V]")
    plt.plot(t, df["vb_out"], label="TS / VB [V]")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.title("DAC Command and Output Voltages")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "03_delta_va_vb.png"), dpi=150)

    # 4. TM+TS and TM-TS
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["tm_plus_ts"], label="TM + TS [V]")
    plt.plot(t, df["tm_minus_ts"], label="TM - TS [V]")
    plt.xlabel("Time [s]")
    plt.ylabel("Voltage [V]")
    plt.title("Complementary Signal Check")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "04_tm_sum_diff.png"), dpi=150)

    # 5. PID terms
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["p_term"], label="P term")
    plt.plot(t, df["i_term"], label="I term")
    plt.plot(t, df["d_term"], label="D term")
    plt.xlabel("Time [s]")
    plt.ylabel("Control contribution")
    plt.title("PID Terms")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "05_pid_terms.png"), dpi=150)

    # 6. Angle speed
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["angle_speed_deg_s"], label="Steering speed [deg/s]")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [deg/s]")
    plt.title("Estimated Steering Speed")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "06_angle_speed.png"), dpi=150)

    # 7. Encoder count
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["count"], label="Encoder count")
    plt.xlabel("Time [s]")
    plt.ylabel("Count")
    plt.title("Encoder Count")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "07_encoder_count.png"), dpi=150)

    # 8. ISR and invalid transitions
    plt.figure(figsize=(10, 5))
    plt.plot(t, df["isr"], label="ISR events")
    plt.plot(t, df["invalid"], label="Invalid transitions")
    plt.xlabel("Time [s]")
    plt.ylabel("Count")
    plt.title("Encoder Diagnostic Counters")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "08_encoder_diagnostics.png"), dpi=150)

    # 9. PID enabled state
    plt.figure(figsize=(10, 3.5))
    plt.plot(t, df["pid_on"], label="PID enabled")
    plt.xlabel("Time [s]")
    plt.ylabel("State")
    plt.title("PID Enable State")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "09_pid_enabled.png"), dpi=150)


def plot_scatter_and_hist(df: pd.DataFrame, out_dir: str | None = None) -> None:
    # 10. Measured vs target scatter
    plt.figure(figsize=(6, 6))
    plt.scatter(df["target_deg"], df["angle_deg"], s=8, alpha=0.6)
    minv = min(df["target_deg"].min(), df["angle_deg"].min())
    maxv = max(df["target_deg"].max(), df["angle_deg"].max())
    plt.plot([minv, maxv], [minv, maxv], linestyle="--", label="Ideal")
    plt.xlabel("Target angle [deg]")
    plt.ylabel("Measured angle [deg]")
    plt.title("Measured vs Target")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "10_measured_vs_target_scatter.png"), dpi=150)

    # 11. Error histogram
    plt.figure(figsize=(8, 5))
    plt.hist(df["error_deg"], bins=40)
    plt.xlabel("Error [deg]")
    plt.ylabel("Samples")
    plt.title("Error Histogram")
    plt.grid(True)
    plt.tight_layout()
    if out_dir:
        plt.savefig(os.path.join(out_dir, "11_error_histogram.png"), dpi=150)


def plot_step_response_windows(df: pd.DataFrame, out_dir: str | None = None) -> None:
    # Detect target changes and plot local windows
    target = df["target_deg"].values
    t = df["t_s"].values

    change_idx = np.where(np.abs(np.diff(target)) > 1e-9)[0] + 1

    if len(change_idx) == 0:
        print("No target step changes detected.")
        return

    print(f"Detected {len(change_idx)} target changes.")

    for k, idx in enumerate(change_idx[:10], start=1):
        t0 = t[idx]
        mask = (t >= t0 - 0.5) & (t <= t0 + 3.0)

        plt.figure(figsize=(10, 5))
        plt.plot(df.loc[mask, "t_s"] - t0, df.loc[mask, "target_deg"], label="Target [deg]")
        plt.plot(df.loc[mask, "t_s"] - t0, df.loc[mask, "angle_deg"], label="Measured [deg]")
        plt.xlabel("Time from step [s]")
        plt.ylabel("Angle [deg]")
        plt.title(f"Step Response #{k}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        if out_dir:
            plt.savefig(os.path.join(out_dir, f"step_response_{k:02d}.png"), dpi=150)


def save_processed_csv(df: pd.DataFrame, csv_path: str, out_dir: str | None = None) -> None:
    if out_dir is None:
        out_dir = os.path.dirname(os.path.abspath(csv_path))
    out_csv = os.path.join(out_dir, "processed_" + os.path.basename(csv_path))
    df.to_csv(out_csv, index=False)
    print(f"Processed CSV saved to: {out_csv}")


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python pid_post_analysis.py your_log.csv")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    out_dir = os.path.splitext(csv_path)[0] + "_plots"
    os.makedirs(out_dir, exist_ok=True)

    df = load_data(csv_path)
    print_summary(df)
    save_processed_csv(df, csv_path, out_dir)
    plot_time_series(df, out_dir)
    plot_scatter_and_hist(df, out_dir)
    plot_step_response_windows(df, out_dir)

    plt.show()


if __name__ == "__main__":
    main()