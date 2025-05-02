import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- Modified Sinc Smoothing ---
def modified_sinc_smooth(signal, cutoff=0.03, window_len=101):
    if window_len % 2 == 0:
        window_len += 1
    half_len = window_len // 2
    t = np.arange(-half_len, half_len + 1)
    sinc_filter = np.sinc(2 * cutoff * t)
    window = np.hamming(window_len)
    sinc_filter *= window
    sinc_filter /= np.sum(sinc_filter)
    padded = np.pad(signal, pad_width=half_len, mode='reflect')
    return np.convolve(padded, sinc_filter, mode='valid')

# --- EMA Smoothing ---
def ema_filter(signal, alpha=0.1):
    ema = [signal[0]]
    for val in signal[1:]:
        ema.append(alpha * val + (1 - alpha) * ema[-1])
    return np.array(ema)

# --- Buffer-style Moving Average ---
def buffer_average(signal, window=5):
    return signal.rolling(window=window, min_periods=1).mean().to_numpy()

# --- Adaptive EMA ---
def adaptive_ema(signal, base_alpha=0.05, max_alpha=0.2):
    adaptive = [signal[0]]
    for i in range(1, len(signal)):
        delta = abs(signal[i] - adaptive[-1])
        alpha = min(max_alpha, base_alpha + delta * 0.1)
        smoothed = alpha * signal[i] + (1 - alpha) * adaptive[-1]
        adaptive.append(smoothed)
    return np.array(adaptive)

# --- Causal Sinc Filter ---
def causal_sinc_filter(signal, cutoff=0.03, window_len=51):
    if window_len % 2 == 0:
        window_len += 1
    t = np.arange(0, window_len)
    sinc_filter = np.sinc(2 * cutoff * (t - (window_len - 1)))
    window = np.hamming(window_len)
    sinc_filter *= window
    sinc_filter /= np.sum(sinc_filter)
    padded = np.pad(signal, (window_len - 1, 0), mode='edge')
    return np.convolve(padded, sinc_filter, mode='valid')

# --- Directories ---
log_dir = "logs"
output_dir = "figures"
os.makedirs(output_dir, exist_ok=True)

# --- Main Loop ---
for filename in os.listdir(log_dir):
    if filename.endswith(".csv") and "Gradient" in filename:
        filepath = os.path.join(log_dir, filename)
        print(f"Processing {filename}")

        try:
            df = pd.read_csv(filepath, skiprows=5, encoding="latin1")
            if df.shape[1] != 6:
                raise ValueError(f"{filename} innehåller {df.shape[1]} kolumner – förväntade 6.")
            df.columns = ["Time(s)", "Iteration", "Tilt(%)", "Speed", "Power", "Efficiency"]
            x = df["Time(s)"]

            for col in ["Tilt(%)", "Speed", "Power", "Efficiency"]:
                y = df[col]

                plt.figure(figsize=(10, 6))
                plt.plot(x, y, label=f"{col} (raw)", color="black", alpha=0.15)
                #plt.plot(x, modified_sinc_smooth(raw), label="sinc", color="red")
                plt.plot(x, ema_filter(y), label="ema", color="blue")
                #plt.plot(x, buffer_average(raw), label="buffer avg", color="green")
                #plt.plot(x, adaptive_ema(raw), label="adaptive ema", color="orange")
                #trimmed_x = x.iloc[len(x) - len(causal_sinc_filter(raw)):].reset_index(drop=True)
                #plt.plot(trimmed_x, causal_sinc_filter(raw), label="causal sinc", color="purple")

                plt.xlabel("Tid (s)")
                plt.ylabel(col)
                plt.title(f"{col} – {filename}")
                plt.grid(True)
                plt.legend()
                plt.tight_layout()
                plt.savefig(os.path.join(output_dir, filename.replace(".csv", f"_{col}_compare_time.pdf")))
                plt.close()

            # --- Dual Axis: Tilt and Efficiency ---
            fig, ax1 = plt.subplots(figsize=(12, 6))
            ax1.set_xlabel("Tid (s)")
            ax1.set_ylabel("Tilt (%)", color="blue")
            ax1.plot(x, df["Tilt(%)"], label="Tilt (%)", color="blue", alpha=0.4)
            ax1.tick_params(axis='y', labelcolor="blue")

            ax2 = ax1.twinx()
            ax2.set_ylabel("Efficiency", color="brown")
            eff = df["Efficiency"]
            ax2.plot(x, eff, label="raw", color="brown", alpha=0.15)
            ax2.plot(x, modified_sinc_smooth(eff), label="sinc", color="red")
            ax2.plot(x, ema_filter(eff), label="ema", color="blue")
            ax2.plot(x, buffer_average(eff), label="buffer", color="green")
            ax2.plot(x, adaptive_ema(eff), label="adaptive", color="orange")
            ax2.plot(trimmed_x, causal_sinc_filter(eff), label="causal sinc", color="purple")
            ax2.tick_params(axis='y', labelcolor="brown")

            plt.title(f"Tilt + Efficiency – {filename}")
            fig.tight_layout()
            plt.savefig(os.path.join(output_dir, filename.replace(".csv", "_tilt_efficiency_dual_axis_time.pdf")))
            plt.close()

        except Exception as e:
            print(f"Fel vid läsning av {filename}: {e}")
