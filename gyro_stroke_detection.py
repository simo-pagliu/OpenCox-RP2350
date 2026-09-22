"""SPM detection from gyro_x (boat pitch rate) peaks.

Offline analysis of on-water logs (OpenCoxUI, rowing_log001/002/003) found
that the accelerometer-magnitude signal used by PicoStrokeDetector shows two
similarly-sized deceleration events per stroke (catch and exit), which makes
telling them apart from magnitude alone unreliable. gyro_x does not have
that problem: it shows one big, sharp peak per stroke (confirmed against the
accelerometer's own deceleration dip and against manually labeled strokes)
with a much smaller bump partway through the cycle. The big/small amplitude
gap is large and consistent across two different sessions and boat speeds,
which makes it a much safer basis for stroke timing than magnitude.

This module only replaces the *timing* used for SPM -- catch/exit semantics
(which of the two per-stroke gyro features is biomechanically "catch" vs
"exit", and the shape/duration metrics PicoStrokeDetector produces for the S
rows) are a separate, not-yet-decided question and are left untouched.

Parameters were fit against 100 Hz gyro_x samples from two real sessions
(rowing_log001, ~24 min, and rowing_log003, ~70 min) by replaying them
through this exact algorithm and comparing the resulting stroke counts and
SPM distribution against an offline analysis (scipy, zero-phase filtering,
full-signal peak search) of the same data -- not guessed from first
principles, since a naive single-pole low-pass turned out to leave enough
sample noise in gyro_x to roughly quadruple the real stroke count.

Design notes:
  - Smoothing is 4 cascaded single-pole low-pass stages rather than one,
    because gyro_x noise riding on top of the real per-stroke peak was
    otherwise itself being detected as extra (spurious) peaks.
  - A peak's "prominence" is its height above the lowest point seen since
    the previous peak -- a cheap, causal stand-in for a proper prominence
    calculation (which needs the whole signal) that's good enough given how
    large and consistent the real catch-vs-noise amplitude gap is.
  - Catch classification adapts to the running peak-height levels instead
    of using one fixed amplitude cutoff (classic ECG QRS-detection trick,
    Pan-Tompkins), because raw gyro_x baseline/peak height differs between
    sessions (e.g. ~40-55 baseline / ~90-127 peaks in one log vs ~60-75 /
    ~90-130 in another) even though the big-vs-small gap itself is
    consistently large.
"""


class PicoGyroSpmDetector:
    """Detect stroke catches from gyro_x peaks and compute SPM from their period."""

    def __init__(
        self,
        min_spm=12,
        max_spm=50,   # capped below the old detector's 55: at 55, a mid-cycle peak
                      # misclassified as "big" 1.1-1.2s after a real catch (implying
                      # ~52-55 SPM) still passed the min-interval sanity check
        smoothing_stages=4,
        smoothing_alpha=0.2,        # per-stage; 4 cascaded stages ~= a steep few-Hz low-pass at 100 Hz
        min_prominence=25.0,        # deg/s; floor below which a peak is noise, not a stroke feature
        min_peak_distance_ms=600,   # two real gyro_x peaks (catch or the smaller mid-cycle one) are never closer than this
        rolling_window=5,
    ):
        self.min_spm = min_spm
        self.max_spm = max_spm
        self.min_interval_ms = int(60000 / max_spm)
        self.max_interval_ms = int(60000 / min_spm)
        self.smoothing_stages = smoothing_stages
        self.smoothing_alpha = smoothing_alpha
        self.min_prominence = min_prominence
        self.min_peak_distance_ms = min_peak_distance_ms
        self.rolling_window = rolling_window

        self._stage_values = [None] * smoothing_stages
        self.filtered = None
        self.last_delta = 0.0
        self.rising = False

        self.trough_since_peak = None
        self.last_peak_time_ms = 0

        # Pan-Tompkins style adaptive big/small peak-height tracking.
        self.big_peak_level = None
        self.small_peak_level = None

        self.last_catch_time_ms = 0
        self.stroke_intervals = []
        self.current_spm = 0

        self.last_catch_detected = False

    def update(self, gyro_x, current_time_ms):
        """Feed one gyro_x sample (deg/s) and its timestamp (ms). Returns True
        exactly when this sample completed a big (catch) peak."""
        self.last_catch_detected = False

        filtered = self._smooth(gyro_x)

        if self.trough_since_peak is None:
            self.trough_since_peak = filtered
            self.filtered = filtered
            return False

        previous = self.filtered
        delta = filtered - previous
        if filtered < self.trough_since_peak:
            self.trough_since_peak = filtered

        # A peak is the sample where the slope turns from rising to falling.
        peak_found = self.rising and delta <= 0 and self.last_delta > 0
        self.rising = delta > 0

        if peak_found:
            self._handle_peak(previous, current_time_ms)  # `previous` was the local max

        self.last_delta = delta
        self.filtered = filtered
        return self.last_catch_detected

    def _smooth(self, raw):
        x = raw
        for i in range(self.smoothing_stages):
            if self._stage_values[i] is None:
                self._stage_values[i] = x
            else:
                self._stage_values[i] += self.smoothing_alpha * (x - self._stage_values[i])
            x = self._stage_values[i]
        return x

    def _handle_peak(self, peak_value, current_time_ms):
        if self.last_peak_time_ms and current_time_ms - self.last_peak_time_ms < self.min_peak_distance_ms:
            return  # debounce: noise wiggle too soon after the last real peak

        prominence = peak_value - self.trough_since_peak
        self.trough_since_peak = peak_value  # reset the trough search for the next peak
        self.last_peak_time_ms = current_time_ms

        if prominence < self.min_prominence:
            return  # too small to be either stroke feature -- sample noise

        if self._classify_and_update(prominence):
            self._handle_catch(current_time_ms)

    def _classify_and_update(self, prominence):
        # Bootstrap: the first two qualifying peaks seed the two levels
        # directly so the detector doesn't have to guess which is "big"
        # before it has ever seen one.
        if self.big_peak_level is None:
            self.big_peak_level = prominence
            return True
        if self.small_peak_level is None:
            self.small_peak_level = prominence
            return False

        threshold = self.small_peak_level + 0.25 * (self.big_peak_level - self.small_peak_level)
        is_big = prominence >= threshold
        if is_big:
            self.big_peak_level = 0.875 * self.big_peak_level + 0.125 * prominence
        else:
            self.small_peak_level = 0.875 * self.small_peak_level + 0.125 * prominence
        return is_big

    def _handle_catch(self, current_time_ms):
        interval_ms = current_time_ms - self.last_catch_time_ms if self.last_catch_time_ms > 0 else 0

        if interval_ms and interval_ms < self.min_interval_ms:
            # Implausibly fast for a real stroke -- a noise/mid-cycle peak
            # slipped past classification. Drop it entirely rather than
            # just skipping the SPM update: if last_catch_time_ms moved to
            # this spurious peak anyway, the *next* real catch's interval
            # would be measured from the wrong point and come out wrong too.
            return

        self.last_catch_time_ms = current_time_ms
        self.last_catch_detected = True

        if interval_ms == 0:
            return
        if interval_ms > self.max_interval_ms:
            self.stroke_intervals = []
            self.current_spm = 0
            return

        self.stroke_intervals.append(interval_ms)
        if len(self.stroke_intervals) > self.rolling_window:
            self.stroke_intervals.pop(0)
        avg_interval = sum(self.stroke_intervals) / len(self.stroke_intervals)
        spm = int(round(60000.0 / avg_interval))
        self.current_spm = max(self.min_spm, min(self.max_spm, spm))

    def get_spm(self):
        return self.current_spm

    def catch_detected(self):
        return self.last_catch_detected

    def reset(self):
        self._stage_values = [None] * self.smoothing_stages
        self.filtered = None
        self.last_delta = 0.0
        self.rising = False
        self.trough_since_peak = None
        self.last_peak_time_ms = 0
        self.big_peak_level = None
        self.small_peak_level = None
        self.last_catch_time_ms = 0
        self.stroke_intervals = []
        self.current_spm = 0
        self.last_catch_detected = False
