"""SPM detection from the boat's fore-aft acceleration.

The input is the acceleration along the boat's direction of travel, in g,
sampled at 100 Hz. main.py builds it as a weighted sum of the accelerometer
axes (SPM_AXIS_WEIGHTS -- see the README, "Mounting and the SPM axis").
Every stroke shows one deep deceleration dip at the catch followed by a
sharp rise in the drive; the detector finds that rise, so its catch time
sits ~1 s after the real catch, by the same amount every stroke.

The parameters were fit on recorded sessions, replayed through this exact
algorithm and checked against an offline analysis of every catch.

Design notes:
  - Smoothing is 4 cascaded single-pole low-pass stages rather than one:
    with one, noise riding on top of the real per-stroke peak is detected
    as extra peaks, about 4x too many strokes.
  - A peak's "prominence" is its height above the lowest point seen since
    the previous peak -- a cheap, causal stand-in for a proper prominence
    calculation (which needs the whole signal) that's good enough given how
    large and consistent the real catch-vs-noise amplitude gap is.
  - Catch classification adapts to the running peak-height levels instead
    of using one fixed amplitude cutoff (classic ECG QRS-detection trick,
    Pan-Tompkins), because the baseline and peak height differ between
    sessions and mountings even though the big-vs-small gap itself is
    consistently large.
"""


def _median(values):
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


class SpmDetector:
    """Detect strokes from fore-aft acceleration peaks and compute SPM from their period."""

    def __init__(
        self,
        min_spm=12,
        max_spm=50,   # above 50, a mid-cycle peak misclassified as "big" 1.1-1.2 s
                      # after a real catch (~52-55 SPM) would pass the min-interval check
        smoothing_stages=4,
        smoothing_alpha=0.2,        # per-stage; 4 cascaded stages ~= a steep few-Hz low-pass at 100 Hz
        min_prominence=0.2,         # g; floor below which a peak is noise, not a stroke feature
        min_peak_distance_ms=600,   # two real peaks (the stroke's main one or the smaller mid-cycle one) are never closer than this
        rolling_window=3,           # SPM from the median of the last 3 intervals, see _handle_catch
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

    def update(self, sample, current_time_ms):
        """Feed one fore-aft acceleration sample (g)
        and its timestamp (ms). Returns True exactly when this sample
        completed a big (catch) peak."""
        self.last_catch_detected = False

        filtered = self._smooth(sample)

        if self.last_catch_time_ms and current_time_ms - self.last_catch_time_ms > self.max_interval_ms:
            # No catch for longer than the slowest plausible stroke: the last
            # SPM is stale, don't keep reporting it.
            self.stroke_intervals = []
            self.current_spm = 0

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

        if self._classify_and_update(prominence, current_time_ms):
            self._handle_catch(current_time_ms)

    def _classify_and_update(self, prominence, current_time_ms):
        # Bootstrap: the first two qualifying peaks seed the two levels
        # directly so the detector doesn't have to guess which is "big"
        # before it has ever seen one.
        if self.big_peak_level is None:
            self.big_peak_level = prominence
            return True
        if self.small_peak_level is None:
            self.small_peak_level = prominence
            return False

        if self.last_catch_time_ms and self._starved(current_time_ms):
            # Big level got stuck above the real catches (e.g. a burst of
            # unusually large peaks) so nothing classifies as big any more.
            # While starved, let it decay toward the small level so the
            # detector re-acquires instead of staying silent for the rest of
            # the session.
            self.big_peak_level = 0.75 * self.big_peak_level + 0.25 * self.small_peak_level

        threshold = self.small_peak_level + 0.25 * (self.big_peak_level - self.small_peak_level)
        is_big = prominence >= threshold
        if is_big:
            self.big_peak_level = 0.875 * self.big_peak_level + 0.125 * prominence
        else:
            self.small_peak_level = 0.875 * self.small_peak_level + 0.125 * prominence
        return is_big

    def _starved(self, current_time_ms):
        return current_time_ms - self.last_catch_time_ms > self.max_interval_ms

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
        # Median of the last 3: it follows a start within a few strokes,
        # and a single wrong catch can't move it.
        spm = int(round(60000.0 / _median(self.stroke_intervals)))
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
