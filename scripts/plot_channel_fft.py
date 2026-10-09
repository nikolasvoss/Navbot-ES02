#!/usr/bin/env python3
"""Plot two channel spectra and their time signals from a comma-separated file.

Usage: python3 scripts/plot_channel_fft.py capture.txt --channels 3 6
Install dependencies with: python3 -m pip install numpy matplotlib
Channel 1 supplies timestamps in microseconds. Other channels are generic signals.
"""

import argparse
import sys


def load_channels(path, selected):
    import numpy as np

    try:
        data = np.loadtxt(path, delimiter=",", skiprows=1, encoding="utf-8-sig")
    except OSError as exc:
        raise ValueError(f"cannot read {path}: {exc}") from exc
    except ValueError as exc:
        raise ValueError(f"could not parse comma-separated numeric data: {exc}") from exc

    if data.ndim != 2 or data.shape[0] < 2:
        raise ValueError("the file must contain a header and at least two data rows")
    needed = max(selected)
    if data.shape[1] < needed:
        raise ValueError(
            f"Channel {needed} was selected, but the file has only {data.shape[1]} columns"
        )

    timestamps = data[:, 0]
    if not np.isfinite(timestamps).all():
        raise ValueError("Channel 1 timestamps must be finite numbers")
    intervals = np.diff(timestamps)
    if np.any(intervals <= 0):
        raise ValueError("Channel 1 timestamps must be strictly increasing")

    channels = {number: data[:, number - 1] for number in selected}
    for number, values in channels.items():
        if not np.isfinite(values).all():
            raise ValueError(f"Channel {number} contains non-finite values")
    return timestamps, channels


def amplitude_spectrum(timestamps, values, sample_interval_seconds):
    import numpy as np

    interval_us = sample_interval_seconds * 1e6
    count = int(np.floor((timestamps[-1] - timestamps[0]) / interval_us)) + 1
    if count < 3:
        raise ValueError("at least three uniform samples are needed for a Hann window")
    uniform_time = timestamps[0] + np.arange(count) * interval_us
    signal = np.interp(uniform_time, timestamps, values)
    signal = signal - np.mean(signal)
    window = np.hanning(count)
    spectrum = np.abs(np.fft.rfft(signal * window)) / window.sum()
    if count % 2 == 0:
        spectrum[1:-1] *= 2
    else:
        spectrum[1:] *= 2
    return np.fft.rfftfreq(count, d=sample_interval_seconds), spectrum


def main():
    parser = argparse.ArgumentParser(
        description="Plot two one-sided channel FFT spectra and their time signals."
    )
    parser.add_argument("file", help="comma-separated data file with a Channel N header")
    parser.add_argument(
        "--channels", nargs=2, type=int, default=(3, 6), metavar=("LEFT", "RIGHT"),
        help="signal channels to plot side by side (default: 3 6)",
    )
    args = parser.parse_args()
    if min(args.channels) < 2:
        parser.error("signal channels must be numbered 2 or higher; Channel 1 is time")

    try:
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.ticker import MaxNLocator
    except ImportError as exc:
        parser.error(
            f"missing plotting dependency: {exc.name}; install with: "
            "python3 -m pip install numpy matplotlib"
        )

    try:
        timestamps, channels = load_channels(args.file, args.channels)
    except ValueError as exc:
        parser.error(str(exc))

    sample_interval = float(np.median(np.diff(timestamps))) * 1e-6
    if sample_interval <= 0:
        parser.error("Channel 1 timestamps do not define a positive sample interval")

    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    grid = fig.add_gridspec(2, 2)
    axes = [fig.add_subplot(grid[0, column]) for column in range(2)]
    for axis, number in zip(axes, args.channels):
        frequency, amplitude = amplitude_spectrum(
            timestamps, channels[number], sample_interval
        )
        axis.plot(frequency, amplitude, linewidth=0.8)
        axis.set_title(f"Channel {number}")
        axis.set_xlabel("Frequency (Hz)")
        axis.set_ylabel("Amplitude")
        axis.grid(True, alpha=0.3)

    time_axis = fig.add_subplot(grid[1, :])
    elapsed_seconds = (timestamps - timestamps[0]) * 1e-6
    for number in args.channels:
        time_axis.plot(
            elapsed_seconds, channels[number], linewidth=0.8, label=f"Channel {number}"
        )
    time_axis.set_title("Selected channel time signals")
    time_axis.set_xlabel("Elapsed time (s)")
    time_axis.xaxis.set_major_locator(MaxNLocator(nbins=12))
    time_axis.set_ylabel("Amplitude")
    time_axis.legend()
    time_axis.grid(True, alpha=0.3)
    fig.suptitle("Selected channel spectra and time signals")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
