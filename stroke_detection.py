"""Stroke detection based on acceleration-magnitude change events."""
import math


class PicoStrokeDetector:
    """Detect catch and exit events from acceleration-magnitude slope changes."""

    # Number of points used to describe the shape of one stroke's
    # acceleration-magnitude curve, sampled at even fractions of that
    # stroke's own (real) duration: 0, 1/4, 1/2, 3/4 and 1 of the period.
    # Storing exactly these fractions means the webapp can plot them
    # directly against a fixed 0..T axis without any further time-warping.
    SHAPE_FRACTIONS = (0.0, 0.25, 0.5, 0.75, 1.0)

    def __init__(
        self,
        min_spm=14,
        max_spm=55,
        min_exit_delay_ms=120,
        smoothing_alpha=0.35,
        noise_alpha=0.08,
        dynamic_threshold_scale=3.2,
        min_delta_threshold=0.03,
    ):
        self.min_spm = min_spm
        self.max_spm = max_spm

        self.min_interval_ms = int(60000 / max_spm)
        self.max_interval_ms = int(60000 / min_spm)
        self.min_exit_delay_ms = min_exit_delay_ms

        # Tuned empirically on 100 Hz MPU6050 streams to react to catch/exit transients
        # while suppressing sample-to-sample noise.
        self.smoothing_alpha = smoothing_alpha
        self.noise_alpha = noise_alpha
        self.dynamic_threshold_scale = dynamic_threshold_scale
        self.min_delta_threshold = min_delta_threshold

        self.last_magnitude = None
        self.filtered_magnitude = None
        self.last_delta = 0.0
        self.delta_noise = 0.0

        self.last_stroke_time_ms = 0
        self.last_exit_time_ms = 0
        self.stroke_count = 0
        self.stroke_intervals = []
        self.current_spm = 0
        self.awaiting_exit = False
        self.last_catch_detected = False
        self.last_exit_detected = False

        # Catch/exit transient duration tracking: a transient is "active"
        # from the threshold crossing that opens it until the slope falls
        # back inside the threshold band.
        self.catch_active = False
        self.catch_active_since_ms = 0
        self.exit_active = False
        self.exit_active_since_ms = 0

        self.last_catch_duration_ms = None
        self.last_exit_duration_ms = None
        self.catch_duration_ready = False
        self.exit_duration_ready = False

        # Buffer of (time_ms, magnitude) samples covering roughly the last
        # stroke, used to interpolate the shape points below. Bounded so it
        # can't grow unbounded if rowing stops without a final catch.
        self._shape_buffer = []
        self.last_stroke_shape = None
        self.stroke_shape_ready = False

    def detect_stroke(self, accel_x, accel_y, accel_z, current_time_ms):
        magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        self.last_catch_detected = False
        self.last_exit_detected = False
        self.catch_duration_ready = False
        self.exit_duration_ready = False
        self.stroke_shape_ready = False

        if self.last_magnitude is None:
            self.last_magnitude = magnitude
            self.filtered_magnitude = magnitude
            self._buffer_sample(current_time_ms, magnitude)
            return False

        previous_filtered = self.filtered_magnitude
        filtered = previous_filtered + self.smoothing_alpha * (magnitude - previous_filtered)
        delta = filtered - previous_filtered

        abs_delta = abs(delta)
        if self.delta_noise == 0.0:
            self.delta_noise = abs_delta
        else:
            self.delta_noise = (1.0 - self.noise_alpha) * self.delta_noise + self.noise_alpha * abs_delta
        threshold = max(self.min_delta_threshold, self.delta_noise * self.dynamic_threshold_scale)

        catch_detected_event = self.last_delta < threshold and delta >= threshold
        exit_detected_event = self.last_delta > -threshold and delta <= -threshold

        if catch_detected_event:
            self._handle_catch(current_time_ms)
        self._update_catch_duration(delta, threshold, current_time_ms)

        if self.awaiting_exit and exit_detected_event:
            self._handle_exit(current_time_ms)
        self._update_exit_duration(delta, threshold, current_time_ms)

        self.filtered_magnitude = filtered
        self.last_delta = delta
        self.last_magnitude = magnitude
        self._buffer_sample(current_time_ms, filtered)
        return self.last_catch_detected

    def _handle_catch(self, current_time_ms):
        interval_ms = current_time_ms - self.last_stroke_time_ms if self.last_stroke_time_ms > 0 else 0
        if not (self.last_stroke_time_ms == 0 or interval_ms >= self.min_interval_ms):
            return

        if self.last_stroke_time_ms > 0:
            self._update_spm_and_shape(interval_ms, current_time_ms)

        self.last_stroke_time_ms = current_time_ms
        self.stroke_count += 1
        self.awaiting_exit = True
        self.last_catch_detected = True
        self.catch_active = True
        self.catch_active_since_ms = current_time_ms

    def _update_spm_and_shape(self, interval_ms, current_time_ms):
        if interval_ms > self.max_interval_ms:
            self.stroke_intervals = []
            self.current_spm = 0
        else:
            shape = self._compute_shape(self.last_stroke_time_ms, current_time_ms)
            if shape is not None:
                self.last_stroke_shape = shape
                self.stroke_shape_ready = True

        self.stroke_intervals.append(interval_ms)
        if len(self.stroke_intervals) > 5:
            self.stroke_intervals.pop(0)
        avg_interval = sum(self.stroke_intervals) / len(self.stroke_intervals)
        spm = int(round(60000.0 / avg_interval))
        self.current_spm = max(self.min_spm, min(self.max_spm, spm))

    def _update_catch_duration(self, delta, threshold, current_time_ms):
        if self.catch_active and delta < threshold:
            self.last_catch_duration_ms = current_time_ms - self.catch_active_since_ms
            self.catch_duration_ready = True
            self.catch_active = False

    def _handle_exit(self, current_time_ms):
        if current_time_ms - self.last_stroke_time_ms < self.min_exit_delay_ms:
            return
        self.awaiting_exit = False
        self.last_exit_time_ms = current_time_ms
        self.last_exit_detected = True
        self.exit_active = True
        self.exit_active_since_ms = current_time_ms

    def _update_exit_duration(self, delta, threshold, current_time_ms):
        if self.exit_active and delta > -threshold:
            self.last_exit_duration_ms = current_time_ms - self.exit_active_since_ms
            self.exit_duration_ready = True
            self.exit_active = False

    def _buffer_sample(self, time_ms, value):
        self._shape_buffer.append((time_ms, value))
        # Never need more than one max-length stroke's worth of history.
        cutoff = time_ms - self.max_interval_ms - 500
        while self._shape_buffer and self._shape_buffer[0][0] < cutoff:
            self._shape_buffer.pop(0)

    def _compute_shape(self, start_ms, end_ms):
        duration = end_ms - start_ms
        if duration <= 0:
            return None
        return [
            self._interpolate(start_ms + frac * duration)
            for frac in self.SHAPE_FRACTIONS
        ]

    def _interpolate(self, target_ms):
        # Linear interpolation between the two buffered samples straddling
        # target_ms. Samples are ~10ms apart at 100Hz against a stroke
        # period of a second or more, so the curve is dense enough that
        # linear interpolation is indistinguishable from a higher-order fit
        # here, at a fraction of the cost.
        buf = self._shape_buffer
        if not buf:
            return 0.0
        if target_ms <= buf[0][0]:
            return buf[0][1]
        for i in range(1, len(buf)):
            t1, v1 = buf[i]
            if t1 >= target_ms:
                t0, v0 = buf[i - 1]
                if t1 == t0:
                    return v0
                ratio = (target_ms - t0) / (t1 - t0)
                return v0 + ratio * (v1 - v0)
        return buf[-1][1]

    def get_spm(self):
        return self.current_spm

    def get_stroke_count(self):
        return self.stroke_count

    def exit_detected(self):
        """Return True when the latest detect_stroke call flagged blade exit."""
        return self.last_exit_detected

    def catch_duration_available(self):
        """Return True when the latest detect_stroke call closed a catch transient."""
        return self.catch_duration_ready

    def get_catch_duration_ms(self):
        return self.last_catch_duration_ms

    def exit_duration_available(self):
        """Return True when the latest detect_stroke call closed an exit transient."""
        return self.exit_duration_ready

    def get_exit_duration_ms(self):
        return self.last_exit_duration_ms

    def stroke_shape_available(self):
        """Return True when the latest detect_stroke call completed a full stroke shape."""
        return self.stroke_shape_ready

    def get_stroke_shape(self):
        """Return the last stroke's acceleration-magnitude shape as SHAPE_FRACTIONS values."""
        return self.last_stroke_shape

    def reset(self):
        self.last_magnitude = None
        self.filtered_magnitude = None
        self.last_delta = 0.0
        self.delta_noise = 0.0
        self.last_stroke_time_ms = 0
        self.last_exit_time_ms = 0
        self.stroke_count = 0
        self.stroke_intervals = []
        self.current_spm = 0
        self.awaiting_exit = False
        self.last_catch_detected = False
        self.last_exit_detected = False
        self.catch_active = False
        self.catch_active_since_ms = 0
        self.exit_active = False
        self.exit_active_since_ms = 0
        self.last_catch_duration_ms = None
        self.last_exit_duration_ms = None
        self.catch_duration_ready = False
        self.exit_duration_ready = False
        self._shape_buffer = []
        self.last_stroke_shape = None
        self.stroke_shape_ready = False
