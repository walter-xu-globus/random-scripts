import re
import sys
import numpy as np
from numpy.fft import fft, fftfreq
import matplotlib.pyplot as plt
from datetime import datetime


def read_pmas_logs(filepath):
    """Read PMAS log file and extract timestamped trns_speed and rot_speed."""
    pattern = re.compile(
        # r'^(\S+)\s+\w+\s+(trns_speed|rot_speed)\s*=\s*([\d.]+)\s*\w+/s'
        
        r'^(\S+)\s+\w+\s+(?:wrist mode\s+)?(trns_speed|rot_speed)\s*=\s*([\d.]+)\s*\w+/s'
    )
    records = []
    with open(filepath, 'r') as f:
        for line in f:
            m = pattern.match(line.strip())
            if m:
                ts = datetime.fromisoformat(m.group(1).replace('Z', '+00:00'))
                records.append((ts, m.group(2), float(m.group(3))))

    # Pair consecutive trns/rot lines into rows with timestamps
    rows = []
    i = 0
    while i < len(records) - 1:
        if records[i][1] == 'trns_speed' and records[i + 1][1] == 'rot_speed':
            rows.append({
                'time_s': records[i][0].timestamp(),
                'trns_speed': records[i][2],
                'rot_speed': records[i + 1][2],
            })
            i += 2
        else:
            i += 1

    # Convert to numpy arrays, zero-base the time
    times = np.array([r['time_s'] for r in rows])
    times -= times[0]
    trns = np.array([r['trns_speed'] for r in rows])
    rot = np.array([r['rot_speed'] for r in rows])
    return times, trns, rot


if __name__ == '__main__':
    import os
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'normal use.txt'
    title = os.path.splitext(os.path.basename(filepath))[0]
    times, trns, rot = read_pmas_logs(filepath)

    N = len(times)
    dt = np.median(np.diff(times))  # use median dt since timing is non-uniform
    freqs = fftfreq(N, d=dt)
    mask = freqs > 0

    for signal, label in [(trns, 'trns_speed (mm/s)'), (rot, 'rot_speed (deg/s)')]:
        fig, (ax_time, ax_fft) = plt.subplots(1, 2, figsize=(14, 3))

        ax_time.plot(times, signal)
        ax_time.set_xlabel('Time (s)')
        ax_time.set_ylabel(label)
        ax_time.grid(True)

        y_fft = fft(signal)
        magnitude = 2.0 / N * np.abs(y_fft[mask])
        ax_fft.plot(freqs[mask], magnitude)
        ax_fft.set_xlabel('Frequency (Hz)')
        ax_fft.set_ylabel('Magnitude')
        ax_fft.grid(True)

        fig.suptitle(f'{title} — {label}')
        plt.tight_layout()

    plt.show()
