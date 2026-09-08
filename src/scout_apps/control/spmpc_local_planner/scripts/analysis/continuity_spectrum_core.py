#!/usr/bin/env python3
"""Offline, NumPy-only spectral helpers for short, individual motion phases.

No ROS initialization, filtering across phase boundaries, or coherence estimate.
Sinusoid frequencies are descriptive fits, not independent spectral resolution.
"""

import math
import numpy as np


def polynomial_design(time, degree):
    time = np.asarray(time, dtype=float)
    span = float(np.ptp(time))
    if span <= 0:
        raise ValueError("non-positive time span")
    scaled = 2 * (time - time[0]) / span - 1
    return np.polynomial.polynomial.polyvander(scaled, int(degree))


def detrend(time, value, degree):
    time, value = np.asarray(time, float), np.asarray(value, float)
    if not np.all(np.isfinite(time)) or not np.all(np.isfinite(value)):
        raise ValueError("nonfinite input")
    if len(time) < max(12, degree + 4) or np.any(np.diff(time) <= 0):
        raise ValueError("need enough strictly increasing samples")
    if np.all(value == value[0]):
        coefficients = np.zeros(degree + 1)
        coefficients[0] = value[0]
        return np.zeros_like(value), value.copy(), coefficients
    design = polynomial_design(time, degree)
    coefficients = np.linalg.lstsq(design, value, rcond=None)[0]
    trend = design @ coefficients
    return value - trend, trend, coefficients


def sinusoid_fit(time, value, degree, frequency):
    """Jointly fit a trend and one tone on native timestamps (no interpolation)."""
    time, value = np.asarray(time, float), np.asarray(value, float)
    residual, _, _ = detrend(time, value, degree)
    if float(residual @ residual) < 1e-24:
        return {"frequency_hz": float(frequency), "peak_amplitude": 0.,
                "partial_r2": 0., "residual_rms": float(np.sqrt(np.mean(residual**2))),
                "tone": np.zeros_like(value)}
    phase = 2 * np.pi * float(frequency) * (time - time[0])
    design = np.column_stack((polynomial_design(time, degree), np.sin(phase), np.cos(phase)))
    coef = np.linalg.lstsq(design, value, rcond=None)[0]
    error = value - design @ coef
    tone = design[:, -2:] @ coef[-2:]
    base = float(residual @ residual)
    return {
        "frequency_hz": float(frequency),
        "peak_amplitude": float(np.hypot(coef[-2], coef[-1])),
        "partial_r2": max(0.0, 1 - float(error @ error) / base) if base > 1e-24 else 0.0,
        "residual_rms": float(np.sqrt(np.mean(error ** 2))),
        "tone": tone,
    }


def scan_tone(time, value, degree, low=4.5, high=5.5, step=0.01):
    residual, _, _ = detrend(time, value, degree)
    if float(residual @ residual) < 1e-24:
        return {"frequency_hz": None, "peak_amplitude": 0., "partial_r2": 0.,
                "residual_rms": float(np.sqrt(np.mean(residual**2))),
                "tone": np.zeros_like(value), "grid_step_hz": step, "at_search_boundary": False}
    best = None
    for f in np.arange(low, high + .25 * step, step):
        fit = sinusoid_fit(time, value, degree, f)
        if best is None or fit["partial_r2"] > best["partial_r2"]:
            best = fit
    best["grid_step_hz"] = step
    best["at_search_boundary"] = bool(abs(best["frequency_hz"] - low) < step / 2 or abs(best["frequency_hz"] - high) < step / 2)
    return best


def band_integral(frequency, psd, low, high):
    if high > frequency[-1] or low < frequency[0]:
        raise ValueError("band outside spectrum")
    inner = (frequency > low) & (frequency < high)
    x = np.r_[low, frequency[inner], high]
    return float(np.trapz(np.interp(x, frequency, psd), x))


def phase_spectrum(time, value, degree, sample_rate, band=(4.5, 5.5)):
    """Full-phase Hann periodogram; no zero padding, smoothing, or phase joining."""
    time, value = np.asarray(time, float), np.asarray(value, float)
    residual, trend, coefficients = detrend(time, value, degree)
    grid = np.arange(0, time[-1] - time[0] + 1e-10, 1 / sample_rate)
    uniform = np.interp(grid, time - time[0], residual)
    window = np.hanning(len(uniform))
    spectrum = np.fft.rfft(uniform * window)
    frequency = np.fft.rfftfreq(len(uniform), 1 / sample_rate)
    psd = np.abs(spectrum) ** 2 / (sample_rate * np.sum(window ** 2))
    psd[1:-1 if len(uniform) % 2 == 0 else None] *= 2
    broad = (frequency >= .5) & (frequency <= min(20, sample_rate / 2))
    peak_index = np.flatnonzero(broad)[np.argmax(psd[broad])]
    band_power = band_integral(frequency, psd, *band)
    neighboring_density = (band_integral(frequency, psd, 3, 4) + band_integral(frequency, psd, 6, 8)) / 3
    tone = scan_tone(time, value, degree, *band)
    frequency_resolution = sample_rate / len(uniform)
    result = {
        "sample_count": len(time), "uniform_sample_count": len(grid),
        "sample_span_sec": float(time[-1] - time[0]),
        "sample_rate_hz": sample_rate, "frequency_bin_hz": frequency_resolution,
        "hann_enbw_hz": float(sample_rate * np.sum(window ** 2) / np.sum(window) ** 2),
        "trend_degree": degree, "trend_coefficients_scaled_time": coefficients.tolist(),
        "raw_min": float(value.min()), "raw_max": float(value.max()),
        "residual_rms": float(np.sqrt(np.mean(residual ** 2))),
        "residual_p95_abs": float(np.percentile(abs(residual), 95)),
        "residual_max_abs": float(abs(residual).max()),
        "broad_peak_bin_hz": float(frequency[peak_index]) if float(residual @ residual) >= 1e-24 else None,
        "band_rms": math.sqrt(max(0, band_power)),
        "band_to_neighbor_density_ratio": band_power / (band[1] - band[0]) / neighboring_density if neighboring_density > 1e-30 else None,
        "native_tone": {k: v for k, v in tone.items() if k != "tone"},
    }
    arrays = {"time": time, "raw": value, "trend": trend, "residual": residual,
              "tone": tone["tone"], "frequency": frequency, "psd": psd}
    return result, arrays


def rolling_tone(time, value, degree, start, stop, frequency=5., window_sec=1., step_sec=.1):
    rows = []
    for left in np.arange(start, stop - window_sec + 1e-9, step_sec):
        mask = (time >= left) & (time < left + window_sec)
        if np.count_nonzero(mask) < 20:
            continue
        fit = sinusoid_fit(time[mask], value[mask], degree, frequency)
        rows.append({"start_sec": float(left), "stop_sec": float(left + window_sec),
                     **{k: v for k, v in fit.items() if k != "tone"}})
    return rows


def damped_tone_fit(time, value, frequency_step=.05, tau_step=.01):
    """Descriptive tail fit; does not identify a mechanical resonance or cause."""
    time, value = np.asarray(time, float), np.asarray(value, float)
    elapsed = time-time[0]
    polynomial = polynomial_design(time, 1)
    residual, _, _ = detrend(time, value, 1)
    base = float(residual @ residual)
    best = None
    for frequency in np.arange(3., 12.+frequency_step/4, frequency_step):
        sincos = np.column_stack((np.sin(2*np.pi*frequency*elapsed), np.cos(2*np.pi*frequency*elapsed)))
        for tau in np.arange(.08, .8+tau_step/4, tau_step):
            design = np.column_stack((polynomial, sincos*np.exp(-elapsed[:,None]/tau)))
            coef = np.linalg.lstsq(design,value,rcond=None)[0]
            error = value-design @ coef
            cost = float(error @ error)
            if best is None or cost < best["sse"]:
                best={"frequency_hz":float(frequency),"tau_sec":float(tau),"sse":cost,
                      "partial_r2":1-cost/base if base>1e-24 else 0.,
                      "peak_amplitude_at_fit_start":float(np.hypot(*coef[-2:])),
                      "fit":design @ coef,"time":time,"raw":value}
    return best
