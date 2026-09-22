# gyro_x-based SPM detection — context for debugging

Read this before touching `gyro_stroke_detection.py` or the SPM-related
parts of `main.py`. It explains why this exists, how it was validated
(entirely offline, against recorded logs — **not yet tested on real
hardware**), and two real bugs that were already found and fixed so they
don't get rediscovered from scratch.

## Why this exists

The original stroke detector (`stroke_detection.py`, `PicoStrokeDetector`)
finds catch/exit from slope changes in accelerometer magnitude. Offline
analysis (in the sibling `OpenCoxUI` repo, working from real on-water logs
`rowing_log001.csv` ~24 min and `rowing_log003.csv` ~70 min) found that
signal shows **two similarly-sized deceleration events per stroke** (catch
and exit), which makes it unreliable for telling them apart — several
different approaches to fix it (timing-based alternation, local gap
comparison, accelerometer-axis rotation search, velocity reconstruction via
integration) were tried and none worked reliably. See that repo's
conversation history / `output/stroke_review/*.png` for the full
investigation if you need the background.

`gyro_x` (boat pitch rate) does not have that problem: it shows **one big,
sharp, dominant peak per stroke**, with a much smaller bump partway through
the cycle. This was confirmed against:
- the accelerometer's own deceleration dip (the gyro peak lines up with it)
- manually labeled strokes from a human reviewing the raw signal

The big/small peak-height gap is large and consistent across both log
files even though the raw baseline/peak height differs between them
(~40-55 baseline / ~90-127 peaks in log001 vs ~60-75 / ~90-130 in log003).

**This only replaces SPM timing.** Which of gyro_x's two per-stroke features
is biomechanically "catch" vs "exit", and the catch/exit duration + stroke
shape metrics `PicoStrokeDetector` produces for the S rows, are a separate,
not-yet-decided question. `PicoStrokeDetector` is untouched and still owns
all of that.

## What changed

- **New file**: `gyro_stroke_detection.py` — `PicoGyroSpmDetector`.
- **`main.py`**:
  - `gyro_spm_detector = PicoGyroSpmDetector(...)` instantiated alongside
    the existing `stroke_detector`.
  - `detect_stroke()` now takes `gyro_data` too and feeds `gyro_x` to
    `gyro_spm_detector.update(...)`.
  - `get_spm()` now returns `gyro_spm_detector.get_spm()` instead of
    `stroke_detector.get_spm()`. This is used for the LCD ("SPM:%d") and
    the G-row `spm` column — i.e. every SPM number a user or the log
    actually sees now comes from gyro_x, not accel magnitude.
  - `stroke_flag`, catch/exit duration, shape — still 100% from
    `stroke_detector` (`PicoStrokeDetector`), unchanged.
  - Added `GYRO_MAX_SPM = 50`, separate from the old detector's `MAX_SPM`
    (55) — see "bugs found" below for why.

## How `PicoGyroSpmDetector` works

1. **4 cascaded single-pole low-pass stages** on `gyro_x` (`smoothing_alpha`
   per stage, default 0.2). One stage wasn't enough — see bugs below.
2. **Peak detection**: a peak is the sample where the smoothed signal's
   slope flips from rising to falling.
3. **Causal "prominence"**: peak height minus the lowest value seen since
   the *previous* peak. This is a cheap, real-time stand-in for a proper
   (whole-signal) prominence calculation — good enough given how large the
   real catch-vs-noise gap is, but it is an approximation; see bugs below
   for a case where it wasn't quite enough on its own.
4. **Absolute floor** (`min_prominence`, default 25.0 deg/s): anything
   below this is noise, not a real stroke feature, full stop.
5. **Adaptive big/small classification** (classic ECG QRS-detection trick,
   Pan-Tompkins): running EMA estimates of typical "big" and "small" peak
   height (`big_peak_level` / `small_peak_level`), threshold set at
   `small + 0.25*(big - small)`. This adapts to the session instead of
   using one fixed cutoff, since raw amplitude differs log to log.
6. A **big** peak is a catch: SPM computed from a rolling average (up to 5)
   of catch-to-catch intervals, bounded by `min_spm`/`max_spm`.

## Validation methodology (do this again if you change the algorithm)

There's no way to unit-test this against ground truth directly — instead,
**replay real recorded gyro_x through the exact class** and sanity-check the
result against numbers already established by (much heavier, offline)
analysis in the `OpenCoxUI` repo:

```python
import sys, csv
sys.path.insert(0, r'path/to/OpenCox-RP2350')
from gyro_stroke_detection import PicoGyroSpmDetector

rows = list(csv.DictReader(open('rowing_log003.csv', newline='')))
a_rows = [r for r in rows if r['record_type'] == 'A']

det = PicoGyroSpmDetector()
spms = []
for r in a_rows:
    ts = int(float(r['uptime_ms']))
    gx = float(r['gyro_x'])
    if det.update(gx, ts) and det.current_spm > 0:
        spms.append(det.current_spm)
# compare len(spms) and its distribution against the expected ~740 strokes,
# median ~18 SPM established for this file in OpenCoxUI's analysis
```

`rowing_log001.csv`, `rowing_log002.csv`, `rowing_log003.csv` live in the
`OpenCoxUI` repo root. There's also an offline converter there
(`analysis_pipeline/convert_spm_gyro.py`) that replays a whole mixed log
through this exact class and writes a corrected copy with the `spm` column
overwritten — useful as a second way to eyeball results at scale, or to
regenerate plots via `output/stroke_review/` scripts in that repo's
history.

Current validated numbers (after the fixes below):
- log001 (~24 min): 254 valid SPM readings, median 19, max 34 (a real
  smooth 30->34->30 sprint-finish ramp in the last 33s of the session, not
  a bug — checked by hand).
- log003 (~70 min): 711 valid SPM readings, median 18, max 20.

## Bugs already found and fixed (don't reintroduce these)

1. **Under-smoothing caused ~4x over-detection.** A first attempt used one
   EMA stage (`alpha=0.27`, chosen to approximate a 5Hz cutoff) and no
   prominence floor. Sample-to-sample noise in gyro_x was itself detected
   as extra peaks: 3190 "catches" over 70 minutes instead of ~740, with SPM
   inflated into the 40s. Fixed by cascading 4 stages and adding
   `min_prominence`. **If you touch smoothing or the floor, replay both log
   files and check the stroke count/SPM median land in the right ballpark
   before trusting it.**

2. **A rejected catch was still corrupting the next interval.**
   `_handle_catch` used to set `self.last_catch_time_ms` and
   `self.last_catch_detected` *before* checking whether the interval since
   the last catch was even plausible. So when a spurious peak got
   classified as "big" (see bug 3), even though its own SPM update was
   correctly thrown out for being too fast, `last_catch_time_ms` had
   already moved to that spurious peak's timestamp — corrupting the *next*
   real catch's interval too. Fixed: the timing reference now only updates
   once the interval has already passed the plausibility check.

3. **A mid-cycle peak occasionally classified as "big".** The adaptive
   threshold (`small + 0.25*(big - small)`) sometimes sits close enough to
   the small-peak population that one with above-average prominence slips
   past it. Found a concrete case in log003 at t=1846.11s: a peak with
   causal-prominence 32.3 got called a catch 1.15s after a real one
   (implying ~52 SPM). Raising the 0.25 factor to fix this directly made
   things *worse* overall (629 valid readings instead of 712 for log003,
   and it shifted which peaks got classified as catches session-wide,
   since the classifier's state is sequential/adaptive — changing it
   doesn't just move one local threshold, it changes the whole history).
   Instead, fixed it from the other side: `PicoGyroSpmDetector`'s own
   `max_spm` default was tightened from 60 to 50 (so `main.py` also gets
   its own `GYRO_MAX_SPM = 50`, separate from the old detector's `MAX_SPM
   = 55`), which makes the interval-plausibility check in bug-fix #2 catch
   this specific failure mode. **This means misclassified mid-cycle peaks
   still happen occasionally — they're just prevented from producing a
   wrong SPM reading, at the cost of that stroke's interval being dropped
   rather than counted.** If you see systematically missing strokes at
   high stroke rates, this interaction is why: check whether a real fast
   stroke is getting rejected by `GYRO_MAX_SPM` before assuming the
   classifier is at fault.

## Known limitations / what's NOT done

- **Not tested on real hardware.** Everything above is a desktop Python
  replay of recorded log files. `main.py` only passed a syntax check
  (`python -m py_compile`) — it has not run on an actual Pico with a live
  MPU6050. Do a bench or on-water test before trusting this in the field.
- Catch vs exit semantics (which gyro_x feature is which, physically) is
  still open — don't infer anything about stroke phase/timing quality from
  this detector beyond the SPM number itself.
- The causal "prominence" measure is a simplification (see bug 3) that can
  still misfire occasionally; the interval-plausibility check is the
  current safety net, not a fix to the classifier itself.
