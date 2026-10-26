"""The LCD pace: the mean GPS speed over the last few strokes, set at each catch.

At 5 Hz the GPS speed shows the boat surging within every stroke (around
0.5 m/s peak to peak), so an average over a time window sliding with every fix
would change on nearly every LCD refresh. Averaging whole strokes, from catch
to catch, removes the surge exactly, and the value only changes once per
stroke.

Between catches the value is held. Once no catch has come for a full stroke
(a pause, the glide after stopping, the first strokes of a start), it falls
back to the plain average of the last few seconds, refreshed once a second.

No hardware here: main.py reports the catches, gps.py holds the fixes.
"""
import utime
import gps


class StrokePace:
    def __init__(self, strokes, max_stroke_ms, fallback_window_s, fallback_interval_ms):
        self.strokes = strokes
        self.max_stroke_ms = max_stroke_ms
        self.fallback_window_s = fallback_window_s
        self.fallback_interval_ms = fallback_interval_ms
        self.catches = []  # (catch ms, newest fix time then), oldest first
        self.last_fallback_ms = None
        self.speed_ms = None

    def catch(self, catch_ms):
        """At a catch: average the fixes since `strokes` catches ago, if those
        were consecutive strokes. Otherwise keep the value shown so far."""
        fix_time = gps.speed_history[-1][0] if gps.speed_history else None
        self.catches.append((catch_ms, fix_time))
        if len(self.catches) > self.strokes + 1:
            self.catches.pop(0)
        if len(self.catches) <= self.strokes:
            return
        first_ms, first_fix_time = self.catches[0]
        if first_fix_time is None or fix_time is None:
            return
        # A pause in between: not `strokes` strokes in a row.
        if utime.ticks_diff(catch_ms, first_ms) > self.strokes * self.max_stroke_ms:
            return
        speed_ms = gps.speed_average_between(first_fix_time, fix_time)
        if speed_ms is not None:
            self.speed_ms = speed_ms

    def update(self, now_ms):
        """Between catches: the fallback, once no catch has come for a stroke."""
        if self.catches and utime.ticks_diff(now_ms, self.catches[-1][0]) <= self.max_stroke_ms:
            return
        if self.last_fallback_ms is not None and \
                utime.ticks_diff(now_ms, self.last_fallback_ms) < self.fallback_interval_ms:
            return
        self.last_fallback_ms = now_ms
        self.speed_ms = gps.speed_average_ms(self.fallback_window_s)
