import pandas as pd
import numpy as np
from scipy import fftpack
import matplotlib.pyplot as plt

def read_elmo_csv(filepath):
    """Read an Elmo txt chart file and return metadata and signal data."""
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Find the start of [Signal Names] and [Signals Data Group 1]
    signal_names_idx = None
    data_idx = None
    for i, line in enumerate(lines):
        if line.strip() == '[Signal Names]':
            signal_names_idx = i + 1
        if line.strip() == '[Signals Data Group 1]':
            data_idx = i + 1

    # Parse signal names
    signal_names = {}
    for i in range(signal_names_idx, len(lines)):
        line = lines[i].strip()
        if not line or line.startswith('['):
            break
        parts = line.split(',')
        sig_id = int(parts[0])
        sig_name = parts[1].strip()
        signal_names[sig_id] = sig_name

    # Parse data: first line is column IDs, rest is data
    col_ids = [int(x) for x in lines[data_idx].strip().rstrip(',').split(',')]
    col_names = [signal_names[cid] for cid in col_ids]

    data_rows = []
    for i in range(data_idx + 1, len(lines)):
        line = lines[i].strip().rstrip(',')
        if not line or line.startswith('['):
            break
        data_rows.append([float(x) for x in line.split(',')])

    df = pd.DataFrame(data_rows, columns=col_names)
    return df


if __name__ == '__main__':
    import sys
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'loadcell noise analysis.csv'
    df = read_elmo_csv(filepath)

    # Extract time info
    time_col = df.columns[0]
    dt = df[time_col].iloc[1] - df[time_col].iloc[0]
    N = len(df)
    freqs = fftpack.fftfreq(N, d=dt)
    mask = freqs > 0

    signal_cols = df.columns[1:]  # everything except time

    for col in signal_cols:
        fig, (ax_time, ax_fft) = plt.subplots(1, 2, figsize=(14, 3))

        # Time domain
        ax_time.plot(df[time_col].values, df[col].values)
        ax_time.set_xlabel('Time (s)')
        ax_time.set_ylabel(col)
        ax_time.grid(True)

        # Frequency domain
        y_fft = fftpack.fft(df[col].values)
        magnitude = 2.0 / N * np.abs(y_fft[mask])
        ax_fft.plot(freqs[mask], magnitude)
        ax_fft.set_xlabel('Frequency (Hz)')
        ax_fft.set_ylabel('Magnitude')
        ax_fft.grid(True)

        plt.tight_layout()

    plt.show()
