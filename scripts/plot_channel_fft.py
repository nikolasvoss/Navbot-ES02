#!/usr/bin/env python3
"""Plot channel FFT, cross-correlation, PSD, and time signals from a CSV file.

Usage: python3 scripts/plot_channel_fft.py capture.txt --channels 3 6
Install dependencies with: python3 -m pip install numpy matplotlib
Channel 1 supplies timestamps in microseconds. Other channels are generic signals.
"""

import argparse
from dataclasses import dataclass
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


@dataclass(frozen=True)
class UniformChannelPair:
    first_number: int
    second_number: int
    first: "numpy.ndarray"
    second: "numpy.ndarray"
    sample_interval_seconds: float


def resample_pair(
    timestamps, channels, first_number, second_number, sample_interval_seconds
):
    import numpy as np

    interval_us = sample_interval_seconds * 1e6
    count = int(np.floor((timestamps[-1] - timestamps[0]) / interval_us)) + 1
    if count < 3:
        raise ValueError("at least three uniform samples are needed for a Hann window")
    uniform_time = timestamps[0] + np.arange(count) * interval_us
    first = np.interp(uniform_time, timestamps, channels[first_number])
    second = np.interp(uniform_time, timestamps, channels[second_number])
    return UniformChannelPair(
        first_number, second_number, first, second, sample_interval_seconds
    )


def amplitude_spectrum(values, sample_interval_seconds):
    import numpy as np

    count = len(values)
    signal = values - np.mean(values)
    window = np.hanning(count)
    spectrum = np.abs(np.fft.rfft(signal * window)) / window.sum()
    if count % 2 == 0:
        spectrum[1:-1] *= 2
    else:
        spectrum[1:] *= 2
    return np.fft.rfftfreq(count, d=sample_interval_seconds), spectrum


def normalized_cross_correlation(pair):
    import numpy as np

    first = pair.first - np.mean(pair.first)
    second = pair.second - np.mean(pair.second)
    denominator = np.sqrt(np.sum(first**2) * np.sum(second**2))
    if denominator == 0:
        raise ValueError("cross-correlation is undefined for a constant selected channel")

    count = len(first)
    fft_length = 1 << (2 * count - 2).bit_length()
    correlation = np.fft.irfft(
        np.fft.rfft(second, fft_length) * np.conj(np.fft.rfft(first, fft_length)),
        fft_length,
    )
    correlation = np.concatenate((correlation[-(count - 1):], correlation[:count]))
    lags = np.arange(-(count - 1), count) * pair.sample_interval_seconds * 1e3
    return lags, correlation / denominator


def welch_psd(values, sample_interval_seconds):
    import numpy as np

    count = len(values)
    segment_length = min(256, count)
    hop = max(1, segment_length // 2)
    window = np.hanning(segment_length)
    scale = 1 / (1 / sample_interval_seconds * np.sum(window**2))
    periodograms = []
    for start in range(0, count - segment_length + 1, hop):
        segment = values[start:start + segment_length]
        segment = (segment - np.mean(segment)) * window
        power = np.abs(np.fft.rfft(segment))**2 * scale
        if segment_length % 2 == 0:
            power[1:-1] *= 2
        else:
            power[1:] *= 2
        periodograms.append(power)

    frequencies = np.fft.rfftfreq(segment_length, d=sample_interval_seconds)
    return frequencies, np.mean(periodograms, axis=0)


def main():
    parser = argparse.ArgumentParser(
        description="Plot channel FFT spectra, cross-correlation, Welch PSD, and time signals."
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

    try:
        pair = resample_pair(
            timestamps, channels, args.channels[0], args.channels[1], sample_interval
        )
        lags, correlation = normalized_cross_correlation(pair)
    except ValueError as exc:
        parser.error(str(exc))

    fig = plt.figure(figsize=(16, 8), constrained_layout=True)
    grid = fig.add_gridspec(2, 3, width_ratios=(1, 1, 1))
    axes = [fig.add_subplot(grid[0, column]) for column in range(2)]
    for axis, number, values in zip(axes, args.channels, (pair.first, pair.second)):
        frequency, amplitude = amplitude_spectrum(values, pair.sample_interval_seconds)
        axis.plot(frequency, amplitude, linewidth=0.8)
        axis.set_title(f"Channel {number}")
        axis.set_xlabel("Frequency (Hz)")
        axis.set_ylabel("Amplitude")
        axis.grid(True, alpha=0.3)

    time_axis = fig.add_subplot(grid[1, :2])
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

    correlation_axis = fig.add_subplot(grid[0, 2])
    correlation_axis.plot(lags, correlation, linewidth=0.8)
    correlation_axis.set_title("Cross-correlation")
    correlation_axis.set_xlabel(
        f"Lag (ms; positive means Channel {pair.second_number} lags "
        f"Channel {pair.first_number})"
    )
    correlation_axis.set_ylabel("Normalized correlation")
    correlation_axis.grid(True, alpha=0.3)

    psd_axis = fig.add_subplot(grid[1, 2])
    for number, values in zip(args.channels, (pair.first, pair.second)):
        psd_frequency, power_density = welch_psd(values, pair.sample_interval_seconds)
        psd_axis.semilogy(
            psd_frequency, power_density, linewidth=0.8, label=f"Channel {number}"
        )
    psd_axis.set_title("Welch PSD")
    psd_axis.set_xlabel("Frequency (Hz)")
    psd_axis.set_ylabel("Power density (channel units²/Hz)")
    psd_axis.legend()
    psd_axis.grid(True, alpha=0.3)
    fig.suptitle("Selected channel spectra, correlation, PSD, and time signals")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
