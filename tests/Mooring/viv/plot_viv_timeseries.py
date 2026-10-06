# @author: ryandavies19, Copilot (GPT-5.4 mini)

"""Plot the VIV test timeseries and the expected period check.

This script reads the generated test output in build/tests/Mooring/viv/
and reproduces the peak-detection logic used by tests/viv.cpp.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPECTED_FREQUENCY_HZ = 1.04
EXPECTED_PERIOD_S = 1.0 / (2.0 * EXPECTED_FREQUENCY_HZ)
PERIOD_TOLERANCE_S = 1.0e-2
CHECK_START_TIME_S = 12.0
MIN_PEAK_AMP = 0.5  # keep in sync with MIN_PEAK_AMP in tests/viv.cpp


def load_timeseries(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load the first two numeric columns from a MoorDyn output file."""

    rows: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8", errors="surrogateescape") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) < 2:
                continue
            try:
                time = float(parts[0])
                value = float(parts[1])
            except ValueError:
                continue
            rows.append((time, value))

    if not rows:
        raise ValueError(f"No numeric time series data found in {path}. Check that FairTen1 is an output in viv.txt")

    data = np.asarray(rows, dtype=float)
    return data[:, 0], data[:, 1]


def detect_test_peaks(time: np.ndarray, tension: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Reproduce the peak tracker used in tests/viv.cpp.

    The test tracks changes in the fairlead tension after 10 seconds and
    records a new peak when the absolute difference from the last peak stops
    growing.
    """

    ten_peaks = [0.0]
    peak_times: list[float] = []
    d_peak = 0.0

    for t, ten in zip(time, tension):
        if t > CHECK_START_TIME_S:
            delta = abs(ten - ten_peaks[-1])
            if delta > d_peak:
                d_peak = delta
            else:
                if d_peak >= MIN_PEAK_AMP:
                    ten_peaks.append(float(ten))
                    peak_times.append(float(t))
                d_peak = 0.0
        else:
            # seed the reference from the latest pre-window sample so the swing
            # is measured against the signal, not against the initial 0.0
            ten_peaks[-1] = float(ten)
            d_peak = 0.0

    return np.asarray(peak_times), np.asarray(ten_peaks[1:])


def compute_fft_spectrum(
    time: np.ndarray,
    values: np.ndarray,
    start_time: float,
    end_time: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute a one-sided amplitude spectrum for the portion within a time window."""

    mask = (time >= start_time) & (time <= end_time)
    segment_time = time[mask]
    segment_values = values[mask]
    if segment_time.size < 4:
        raise ValueError("Not enough samples after the FFT start time")

    # Set up time grid and interpolate, then center.
    dt = float(np.median(np.diff(segment_time)))
    uniform_time = np.arange(segment_time[0], segment_time[-1] + 0.5 * dt, dt)
    uniform_values = np.interp(uniform_time, segment_time, segment_values)
    centered_values = uniform_values - uniform_values.mean()

    # Taper the tails and apply Hanning window
    tail_length = centered_values.size // 10  # Or something else
    window_mask = np.zeros(centered_values.size, dtype=bool)
    window_mask[:tail_length] = True
    window_mask[-tail_length:] = True
    windowed_values = np.copy(centered_values)
    windowed_values[window_mask] *= np.hanning(2 * tail_length)

    # FFT and normalize
    spectrum = np.fft.rfft(windowed_values)
    frequencies = np.fft.rfftfreq(windowed_values.size, d=dt)
    window = np.ones(centered_values.size)
    window[window_mask] = np.hanning(2 * tail_length)
    normalization = window.sum() / 2.0
    amplitudes = np.abs(spectrum) / normalization
    return frequencies, amplitudes


def plot_fft_comparison(
    time: np.ndarray,
    tension: np.ndarray,
    overlay_time: np.ndarray | None,
    overlay_tension: np.ndarray | None,
    fft_end_time: float,
    output_path: Path,
) -> plt.Figure:
    """Plot the FFT comparison for the post-startup portion of the simulation."""

    fig, ax = plt.subplots(figsize=(12, 6))

    frequencies, amplitudes = compute_fft_spectrum(time, tension, CHECK_START_TIME_S, fft_end_time)
    ax.plot(frequencies, amplitudes, color="#1f4e79", lw=1.4, label="Fairlead tension")

    if overlay_time is not None and overlay_tension is not None:
        overlay_frequencies, overlay_amplitudes = compute_fft_spectrum(
            overlay_time,
            overlay_tension,
            CHECK_START_TIME_S,
            fft_end_time,
        )
        ax.plot(
            overlay_frequencies,
            overlay_amplitudes,
            color="#7a3ea1",
            lw=1.2,
            alpha=0.85,
            label="OpenFAST r-test",
        )

    ax.set_xlim(left=0.0)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Amplitude")
    ax.set_title(f"FFT comparison from {CHECK_START_TIME_S:.0f} to {fft_end_time:.3f} seconds")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    return fig


def plot_viv_timeseries(
    time: np.ndarray,
    tension: np.ndarray,
    peak_times: np.ndarray,
    overlay_time: np.ndarray | None,
    overlay_tension: np.ndarray | None,
    output_path: Path,
    show: bool,
) -> plt.Figure:
    """Plot the time series and the period check against the expected value."""

    fig, (ax_tension, ax_period) = plt.subplots(
        2,
        1,
        figsize=(12, 8),
        sharex=True,
        gridspec_kw={"height_ratios": [2.2, 1.0]},
    )

    ax_tension.plot(time, tension, color="#1f4e79", lw=1.25, label="Fairlead tension")
    if overlay_time is not None and overlay_tension is not None:
        ax_tension.plot(
            overlay_time,
            overlay_tension,
            color="#7a3ea1",
            lw=1.0,
            alpha=0.8,
            label="OpenFAST r-test",
        )
    if peak_times.size:
        peak_indices = np.searchsorted(time, peak_times)
        peak_indices = np.clip(peak_indices, 0, len(time) - 1)
        ax_tension.scatter(
            time[peak_indices],
            tension[peak_indices],
            s=18,
            color="#c23b22",
            zorder=3,
            label="Detected peaks",
        )

    ax_tension.axvline(CHECK_START_TIME_S, color="#666666", ls="--", lw=1.0, alpha=0.7)
    ax_tension.set_ylabel("Fairlead tension [N]")
    ax_tension.set_title("VIV test output and expected period")
    ax_tension.grid(True, alpha=0.25)
    ax_tension.legend(loc="best")

    if peak_times.size >= 3:
        observed_periods = peak_times[2:] - peak_times[:-2]
        period_times = peak_times[2:]
        ax_period.plot(period_times, observed_periods, "o", ms=4.5, color="#1f4e79", label="Observed period")
        out_of_tolerance = np.abs(observed_periods - EXPECTED_PERIOD_S) > PERIOD_TOLERANCE_S
        for period_time, period_value in zip(period_times[out_of_tolerance], observed_periods[out_of_tolerance]):
            ax_period.axvline(
                period_time,
                color="#c23b22",
                ls=":",
                lw=1.0,
                alpha=0.85,
            )
            ax_period.annotate(
                f"t={period_time:.3f} s",
                xy=(period_time, 0.97),
                xycoords=(ax_period.transData, ax_period.transAxes),
                xytext=(0, 0),
                textcoords="offset points",
                rotation=90,
                fontsize=8,
                color="#c23b22",
                ha="center",
                va="top",
            )
    else:
        observed_periods = np.asarray([])
        ax_period.text(0.5, 0.5, "Not enough peaks detected to compute a period", ha="center", va="center", transform=ax_period.transAxes)

    ax_period.axhline(EXPECTED_PERIOD_S, color="#c23b22", lw=1.5, label=f"Expected period = {EXPECTED_PERIOD_S:.6f} s")
    ax_period.axhspan(
        EXPECTED_PERIOD_S - PERIOD_TOLERANCE_S,
        EXPECTED_PERIOD_S + PERIOD_TOLERANCE_S,
        color="#c23b22",
        alpha=0.12,
        label=f"Test tolerance ±{PERIOD_TOLERANCE_S:.2e} s",
    )
    ax_period.set_xlabel("Time [s]")
    ax_period.set_ylabel("Period [s]")
    ax_period.grid(True, alpha=0.25)
    ax_period.legend(loc="best")

    if observed_periods.size:
        mean_period = observed_periods.mean()
        ax_period.set_title(f"Observed mean period = {mean_period:.6f} s")

    fig.tight_layout()
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    return fig


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot the VIV test fairlead tension and the expected period check."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="../../../build/tests/Mooring/viv/viv.out",
        help="Path to the viv.out file produced by the test run.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional output image path. Defaults to the input path with .png appended.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot interactively after saving it.",
    )
    parser.add_argument(
        "--overlay",
        default="<path to OpenFAST r-test/modules/moordyn/md_VIV/driver.MD.out>",
        help="Optional second output file to overlay on the main timeseries plot. Use an empty string to disable.",
    )
    parser.add_argument(
        "--fft-output",
        default=None,
        help="Optional output image path for the FFT comparison. Defaults to the main output path with _fft.png appended.",
    )
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    if args.output is None:
        output_path = input_path.with_suffix(input_path.suffix + ".png")
    else:
        output_path = Path(args.output).expanduser().resolve()

    if args.fft_output is None:
        fft_output_path = output_path.with_name(f"{output_path.stem}_fft{output_path.suffix}")
    else:
        fft_output_path = Path(args.fft_output).expanduser().resolve()

    time, tension = load_timeseries(input_path)
    peak_times, _ = detect_test_peaks(time, tension)
    fft_end_time = float(time.max())

    overlay_time = None
    overlay_tension = None
    if args.overlay:
        overlay_path = Path(args.overlay).expanduser().resolve()
        overlay_time, overlay_tension = load_timeseries(overlay_path)

    print(f"Loaded {len(time)} samples from {input_path}")
    print(f"Expected period: {EXPECTED_PERIOD_S:.6f} s")
    if overlay_time is not None:
        print(f"Loaded {len(overlay_time)} samples from {overlay_path}")
    if peak_times.size >= 3:
        observed_periods = peak_times[2:] - peak_times[:-2]
        print(f"Detected {len(peak_times)} peaks after {CHECK_START_TIME_S:.1f} s")
        print(f"Mean observed period: {observed_periods.mean():.6f} s")
    else:
        print("Not enough peaks detected to compute an observed period.")

    viv_fig = plot_viv_timeseries(
        time,
        tension,
        peak_times,
        overlay_time,
        overlay_tension,
        output_path,
        args.show,
    )
    fft_fig = plot_fft_comparison(
        time,
        tension,
        overlay_time,
        overlay_tension,
        fft_end_time,
        fft_output_path,
    )
    if args.show:
        plt.show()
    plt.close(viv_fig)
    plt.close(fft_fig)
    print(f"Saved plot to {output_path}")
    print(f"Saved FFT plot to {fft_output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())