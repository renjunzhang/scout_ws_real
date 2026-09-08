#!/usr/bin/env python3
"""Known-frequency and non-oscillatory controls for the offline spectral audit."""
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "analysis"))
import continuity_spectrum_core as core


class SpectrumTest(unittest.TestCase):
    def test_recovers_tone_with_jitter_and_acceleration_trend_at_both_sensor_rates(self):
        for rate in (50., 90.):
            rng = np.random.RandomState(25)
            t = np.arange(0, 3, 1/rate)
            t[1:-1] += rng.uniform(-.0005, .0005, len(t)-2)
            amplitude, frequency = .00022, 5.13
            y = 1.4 + .3*t + .15*t*t + .04*t**3 + amplitude*np.sin(2*np.pi*frequency*t+.6)
            y += rng.normal(0, amplitude*.03, len(t))
            fit = core.scan_tone(t, y, 3)
            self.assertAlmostEqual(fit["frequency_hz"], frequency, delta=.02)
            self.assertAlmostEqual(fit["peak_amplitude"], amplitude, delta=amplitude*.03)
            self.assertGreater(fit["partial_r2"], .99)

    def test_pure_polynomial_does_not_create_five_hz(self):
        t = np.arange(0, 3, .02)
        y = 8 + .8*t*t + .1*t**3
        summary, _ = core.phase_spectrum(t, y, 3, 50)
        self.assertLess(summary["native_tone"]["peak_amplitude"], 1e-11)
        self.assertLess(summary["band_rms"], 1e-11)

    def test_separate_constant_phases_do_not_turn_switch_into_spectral_power(self):
        for value, start in ((0., 0.), (.8, 3.), (0., 5.)):
            t = np.arange(start, start+2, .02)
            summary, _ = core.phase_spectrum(t, np.full(len(t), value), 1, 50)
            self.assertLess(summary["band_rms"], 1e-12)

    def test_hann_normalization_and_short_window_resolution(self):
        t = np.arange(0, 2, .02)
        amplitude = .2
        summary, arrays = core.phase_spectrum(t, amplitude*np.cos(2*np.pi*5*t), 1, 50)
        self.assertAlmostEqual(summary["frequency_bin_hz"], .5, places=8)
        self.assertGreater(summary["hann_enbw_hz"], .74)
        self.assertAlmostEqual(sum(arrays["psd"])*.5, amplitude**2/2, delta=.0005)
        self.assertAlmostEqual(summary["native_tone"]["peak_amplitude"], amplitude, delta=.002)

    def test_local_fit_distinguishes_decaying_ring_from_sustained_tone(self):
        t = np.arange(0, 4, .02)
        persistent = np.sin(2*np.pi*5*t)
        decaying = persistent*np.exp(-t/.5)
        fixed = core.rolling_tone(t, persistent, 1, 0, 4, step_sec=1)
        ring = core.rolling_tone(t, decaying, 1, 0, 4, step_sec=1)
        self.assertLess(abs(fixed[0]["peak_amplitude"]-fixed[-1]["peak_amplitude"]), 1e-10)
        self.assertGreater(ring[0]["peak_amplitude"]/ring[-1]["peak_amplitude"], 100)

    def test_refuses_duplicate_or_nonfinite_time(self):
        t = np.arange(0, 2, .02)
        t[10] = t[9]
        with self.assertRaises(ValueError):
            core.phase_spectrum(t, np.sin(t), 1, 50)
        t[10] = np.nan
        with self.assertRaises(ValueError):
            core.phase_spectrum(t, np.sin(t), 1, 50)

    def test_recovers_known_damped_tail_frequency_and_decay(self):
        t = np.arange(.18, 1.2, .02)
        elapsed = t-t[0]
        y = .04+.01*elapsed+.3*np.exp(-elapsed/.23)*np.sin(2*np.pi*6.25*elapsed+.4)
        fit = core.damped_tone_fit(t, y)
        self.assertAlmostEqual(fit["frequency_hz"], 6.25, delta=.05)
        self.assertAlmostEqual(fit["tau_sec"], .23, delta=.01)
        self.assertAlmostEqual(fit["peak_amplitude_at_fit_start"], .3, delta=.005)
        self.assertGreater(fit["partial_r2"], .999)


if __name__ == "__main__":
    unittest.main()
