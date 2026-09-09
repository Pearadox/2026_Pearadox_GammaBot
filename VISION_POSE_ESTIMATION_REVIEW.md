# Vision / Pose Estimation Review — Recommendations

**Date:** 2026-09-09
**Branch reviewed:** `wcmp`
**Scope:** `frc.robot.subsystems.vision.*`, plus the pose-estimator consumer in
`frc.robot.subsystems.drive.Drive` and the wiring in `RobotContainer` / `Robot`.

**No code was changed.** This document is recommendations only.

---

## Files reviewed

| File | Role |
|---|---|
| [Vision.java](../src/main/java/frc/robot/subsystems/vision/Vision.java) | Filtering, "best pose" selection, std-dev computation |
| [VisionConstants.java](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java) | Thresholds and baselines |
| [VisionIO.java](../src/main/java/frc/robot/subsystems/vision/VisionIO.java) | `PoseObservation` record / IO contract |
| [VisionIOLimelight.java](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java) | MegaTag1 + MegaTag2 NT parsing, latency, throttle, rewind |
| [VisionIOPhotonVision.java](../src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java) | PhotonVision path (currently commented out in `RobotContainer`) |
| [VisionIOPhotonVisionSim.java](../src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java) | Sim camera |
| [Drive.java](../src/main/java/frc/robot/subsystems/drive/Drive.java) | `SwerveDrivePoseEstimator`, `addVisionMeasurement` |
| [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java) | Camera instantiation, rotation supplier |
| [Robot.java](../src/main/java/frc/robot/Robot.java) | Throttle / rewind lifecycle |

Architecture in one line: two Limelights publish MegaTag1 **and** MegaTag2 botposes;
`Vision.periodic()` reads every queued observation from both cameras, filters them, picks the
**single** lowest-"doubt" observation across all cameras, and pushes only that one into
`Drive`'s `SwerveDrivePoseEstimator`.

---

## Scorecard against the rules of thumb

| # | Rule of thumb | Current implementation | Verdict |
|---|---|---|---|
| 1 | Ambiguity 0.2 multi-tag | No ambiguity check at all on multi-tag | Missing |
| 2 | Ambiguity 0.4 single-tag fallback | `maxAmbiguity = 0.3`, single-tag only ([VisionConstants.java:38](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L38)) | Close; conservative end of range |
| 3 | Multi-tag always preferred over single-tag | Selection metric **penalizes** extra tags — see Finding A | **Inverted** |
| 4 | Distance cutoff 4.0 m, hard reject single-tag beyond | No distance rejection anywhere | Missing |
| 5 | Speed gate: linear > 3.5 m/s discard | No linear gate | Missing |
| 6 | Speed gate: angular > 540 deg/s discard | `maxRotsPerSecond = 2.0` rad/s = **114 deg/s** | ~4.7x stricter than rule |
| 7 | StdDev XY baseline 0.05 m multi-tag | `linearStdDevBaseline = 0.02` for everything | 2.5x more trusting |
| 8 | StdDev XY baseline 0.10 m single-tag | Same 0.02 baseline, no tag-count split | Missing |
| 9 | StdDev formula `baseline * distance^2 / tagCount` | Exactly this ([Vision.java:176-179](../src/main/java/frc/robot/subsystems/vision/Vision.java#L176-L179)) | **Matches** |
| 10 | Single-tag theta penalty | None applied | Missing (rule text is self-contradictory — see Finding E3) |
| 11 | MegaTag2 factor 0.5x | `linearStdDevMegatag2Factor = 0.5` | **Matches** |
| 12 | Camera 1280x720 / 80 deg FOV / 20 FPS | Set in LL web UI (not in repo); sim uses library defaults | Unverifiable / sim mismatch |
| 13 | Latency subtracted from FPGA timestamp immediately | Done in the IO layer ([VisionIOLimelight.java:93](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L93)) | **Matches** |
| 14 | Max latency accepted 100 ms | No staleness check | Missing |
| 15 | Kalman XY stdDev clamp [0.02, 3.0] m | No clamp | Missing |

Two additional issues fall outside the rule list but materially affect pose quality:
the single-observation-per-cycle bottleneck (Finding B) and the MegaTag2 rotation
feedback loop (Finding C).

---

## Findings, highest impact first

### Finding A — The "doubt index" prefers *fewer* tags (bug)

[Vision.java:156](../src/main/java/frc/robot/subsystems/vision/Vision.java#L156):

```java
double doubtIndex = observation.averageTagDistance() * observation.tagCount();
```

Lower `doubtIndex` wins. Multiplying by `tagCount` means a 2-tag solve at 3 m scores `6.0`
while a 1-tag solve at 3 m scores `3.0` — **the single-tag pose is selected**. This directly
inverts rule 1 ("Multi-tag results always preferred over single-tag due to lower inherent
ambiguity"), and it fights the std-dev math three lines below, which correctly *divides* by
`tagCount`.

**Recommendation.** Use the same uncertainty proxy the std-dev formula already uses, so
selection and weighting agree:

```java
double doubtIndex =
    Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
```

Nothing else about the comparison needs to change. Expect a visible behavior shift: the robot
will start preferring the wide multi-tag solutions it currently passes over.

> Also at [Vision.java:162](../src/main/java/frc/robot/subsystems/vision/Vision.java#L162): the
> comparison is `<=`, so among equal-scoring observations the *last* one wins, which makes the
> chosen camera depend on IO array order. Prefer `<` for determinism.

---

### Finding B — Only one observation per 20 ms cycle reaches the estimator

Both Limelights are read with `readQueue()`, so each `periodic()` typically yields several
observations per camera (MegaTag1 *and* MegaTag2 queues, at up to 100 fps). All of them are
filtered, but [Vision.java:185](../src/main/java/frc/robot/subsystems/vision/Vision.java#L185)
calls `consumer.accept(...)` exactly once, for the single winner. Everything else — including
perfectly good multi-tag frames from the *other* camera — is discarded.

A Kalman filter is the right tool for fusing several noisy-but-independent measurements; the
`distance^2 / tagCount` std-dev already encodes relative trust. Hand-picking one winner throws
away most of the available information and makes the estimate jumpier, not smoother, because
consecutive cycles can hop between cameras with different systematic biases.

The `TODO` at [Vision.java:101-102](../src/main/java/frc/robot/subsystems/vision/Vision.java#L101-L102)
suggests this was a known interim design.

**Recommendation.** Feed *all* surviving observations, each with its own std-devs — the
upstream AdvantageKit template behavior:

```java
for (var observation : accepted) {   // accumulated across all cameras
  double stdDevFactor =
      Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
  // ... per-observation linear/angular std devs, MT2 factor, clamps ...
  consumer.accept(observation.pose().toPose2d(), observation.timestamp(), stdDevs);
}
```

`SwerveDrivePoseEstimator.addVisionMeasurement` replays odometry from the sample timestamp, so
out-of-order timestamps across cameras are handled correctly.

If the team wants to keep a single-winner policy (it does make log analysis simpler), then at
minimum restrict it to a **tier** rather than a scalar: accept every multi-tag observation, and
fall back to the best single-tag observation only when no multi-tag observation survived
filtering. Keep the existing `TrustedPose` / `trustedCamera` logging either way — it is genuinely
useful and should not be dropped.

---

### Finding C — MegaTag2 orientation is fed the *vision-fused* yaw (feedback loop)

[RobotContainer.java:127-128](../src/main/java/frc/robot/RobotContainer.java#L127-L128) passes
`drive::getRotation` as the MegaTag2 rotation supplier. `Drive.getRotation()` returns
`poseEstimator.getEstimatedPosition().getRotation()`
([Drive.java:324-325](../src/main/java/frc/robot/subsystems/drive/Drive.java#L324-L325)) — the
*fused* heading, not raw gyro.

Meanwhile MegaTag1 observations are accepted with a finite angular std-dev
(`angularStdDevBaseline = 0.06` rad = 3.4 deg, quite trusting), so vision-derived rotation does
enter the estimate. The result is a loop:

```
MegaTag1 rotation -> poseEstimator yaw -> robot_orientation_set -> MegaTag2 pose -> poseEstimator
```

A biased MegaTag1 3D solve nudges the fused yaw, which biases the MegaTag2 solve in the same
direction, which reinforces the yaw. This is the classic cause of slow heading drift that
"only happens when a certain tag is visible."

**Recommendation.** Break the loop at one of two places:

1. **Preferred:** supply raw gyro yaw to the Limelights. Add a
   `Drive.getRawGyroRotation()` accessor returning `rawGyroRotation` (already maintained at
   [Drive.java:197-206](../src/main/java/frc/robot/subsystems/drive/Drive.java#L197-L206) and
   already falling back to kinematics when the Pigeon drops out), and pass
   `drive::getRawGyroRotation` to `VisionIOLimelight` instead. `Drive.getRotation()` stays as-is
   for the driver-facing and turret uses.
2. **Or:** stop trusting MegaTag1 rotation — treat MegaTag1 like MegaTag2 for the theta term and
   let the gyro own heading entirely. Note this only works if the pose is seeded with a correct
   heading, so it pairs with a pre-match seeding step (Finding K).

Option 1 is the smaller change and keeps MegaTag1 rotation available as a weak correction.

---

### Finding D — Missing rejection filters (distance, latency, linear speed)

`rejectPose` at [Vision.java:128-142](../src/main/java/frc/robot/subsystems/vision/Vision.java#L128-L142)
covers tag count, single-tag ambiguity, Z error, field bounds, and angular rate. Three gates
from the rule set are absent.

**D1 — Single-tag distance cutoff (rule 4).** `averageTagDistance` is read but never gated.
A single tag at 7 m produces a pose with metre-scale error that currently enters the filter with
`0.02 * 49 / 1 = 0.98 m` std-dev — not aggressive enough to be harmless.

```java
// VisionConstants
public static double maxSingleTagDistance = 4.0;   // meters, hard reject
public static double maxMultiTagDistance  = 6.0;   // meters, optional softer bound

// Vision.rejectPose
|| (observation.tagCount() == 1
    && observation.averageTagDistance() > maxSingleTagDistance)
|| observation.averageTagDistance() > maxMultiTagDistance
```

**D2 — Latency / staleness cap (rule 14).** Nothing rejects an observation whose timestamp is
far in the past. A momentarily wedged coprocessor or a network hiccup will dump a burst of stale
poses into the estimator, all of which get replayed against odometry.

```java
public static double maxObservationAgeSeconds = 0.10;   // 100 ms
...
|| (Timer.getFPGATimestamp() - observation.timestamp()) > maxObservationAgeSeconds
|| observation.timestamp() <= 0.0   // guards an unpopulated/garbage sample
```

Consider logging the observed age to `Vision/Camera*/ObservationAge` for a match or two before
committing to 100 ms — with the current disabled-throttle setting the *first* frames after enable
can legitimately be older than that.

**D3 — Linear speed gate (rule 5).** Only angular rate is gated today.

```java
double linearSpeed =
    Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
...
|| linearSpeed > maxLinearSpeedForVision
```

Sizing note: `TunerConstants.kSpeedAt12Volts = 5.12 m/s`
([TunerConstants.java:83](../src/main/java/frc/robot/generated/TunerConstants.java#L83)), so the
rule's 3.5 m/s threshold disables vision for the top ~30% of the speed range. That is the
intent of the rule (motion blur), and it is a reasonable default, but it does mean **no pose
corrections during long field-crossing sprints**. Verify against match logs before tightening
below 3.5.

---

### Finding E — Ambiguity thresholds and the single-tag theta penalty

**E1 — Split the ambiguity threshold (rules 1, 2).** Today
[Vision.java:130-131](../src/main/java/frc/robot/subsystems/vision/Vision.java#L130-L131)
applies `maxAmbiguity = 0.3` **only** when `tagCount() == 1`. Multi-tag observations get no
ambiguity check.

That is defensible — per-tag ambiguity is not meaningful for a multi-tag solve, and the
MegaTag1 parser deliberately reads only the first tag's ambiguity
([VisionIOLimelight.java:98-100](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L98-L100)) —
but it does leave rule 1 unimplemented. Recommended shape:

```java
public static double maxAmbiguityMultiTag  = 0.2;
public static double maxAmbiguitySingleTag = 0.4;
```

applied as `observation.tagCount() == 1 ? maxAmbiguitySingleTag : maxAmbiguityMultiTag`. Raising
the single-tag bound 0.3 -> 0.4 is a real loosening; pair it with the D1 distance cutoff, which
removes the far-tag cases where high ambiguity actually matters. If the team prefers to stay
conservative, keep 0.3 — it is inside the rule's stated 0.3–0.5 band, at the conservative end,
and 0.3 is explicitly called out as the conservative-team choice.

**E2 — MegaTag2 bypasses the ambiguity gate entirely.** `VisionIOLimelight` hard-codes
`ambiguity = 0.0` for MegaTag2
([VisionIOLimelight.java:124](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L124)),
correctly reasoning that MegaTag2 is already disambiguated by the supplied yaw. The consequence
is that a **single-tag MegaTag2 pose is never rejected on ambiguity**, no matter how marginal.
Given Finding C, a single-tag MegaTag2 pose is only as good as the yaw it was handed. This makes
the D1 distance cutoff the *only* real quality gate on single-tag MegaTag2 poses — another reason
to add it.

**E3 — Single-tag theta penalty (rule 10).** No theta penalty exists.

The rule text is internally inconsistent here and should be resolved before implementing:
it says *"apply 2.5x penalty to standard deviation theta"* and then *"Single-tag theta stdDev
multiplier = 0.75"*. Those point opposite ways — multiplying a std-dev by 0.75 **increases**
trust, which contradicts the accompanying gloss "reduce rotation trust." Taking the stated
*intent* (reduce rotation trust on single-tag), the multiplier must be **> 1**, i.e. the 2.5x
figure:

```java
public static double singleTagAngularStdDevPenalty = 2.5;   // >1 = trust rotation less
...
if (observation.tagCount() == 1) {
  angularStdDev *= singleTagAngularStdDevPenalty;
}
```

Make it a `LoggedTunableNumber` so it can be swept on the practice field, and flag the
0.75-vs-2.5 discrepancy back to whoever authored the rule sheet.

---

### Finding F — Std-dev baselines and the missing Kalman clamp

**F1 — Baselines (rules 7, 8).** `linearStdDevBaseline = 0.02` m
([VisionConstants.java:44](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L44))
is 2.5x more trusting than the rule's 0.05 m multi-tag figure, and there is no separate
single-tag baseline. Recommended:

```java
public static double linearStdDevBaselineMultiTag  = 0.05;  // meters
public static double linearStdDevBaselineSingleTag = 0.10;  // meters
```

This is a **significant de-trusting** of vision — at 3 m with 2 tags the linear std-dev goes
from `0.02 * 9/2 = 0.09` to `0.05 * 9/2 = 0.225`. Convergence will be slower and the pose less
twitchy. Change this one deliberately and re-tune auto-align, not as a drive-by edit: anything
downstream that assumes fast vision convergence (auto-align, the `MovingShotSolver` feed) may
need its own timing revisited.

**F2 — Clamp the result (rule 15).** Add the clamp after all multipliers, before `accept`:

```java
linearStdDev = MathUtil.clamp(linearStdDev, minLinearStdDev /* 0.02 */, maxLinearStdDev /* 3.0 */);
```

With the recommended 0.10 single-tag baseline, a single tag at 6 m yields `0.10 * 36 = 3.6`,
above the 3.0 ceiling — so the clamp does bind in practice, not just in theory. The 0.02 floor
also prevents an unrealistically confident measurement from yanking the estimate.

Leave `angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY`
([VisionConstants.java:57-58](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L57-L58))
alone — WPILib's `PoseEstimator` computes `K = q / (q + sqrt(q*r))`, which goes cleanly to 0 as
`r` grows without bound, so this correctly means "ignore MegaTag2 rotation." Do **not** apply the
F2 clamp to the angular term or you will re-enable rotation trust from MegaTag2. Clamp the linear
term only.

---

### Finding G — Trench-tag handling is per-camera, not per-observation

[Vision.java:108-124](../src/main/java/frc/robot/subsystems/vision/Vision.java#L108-L124)
computes `onlySeesTrenchTags` from `inputs[cameraIndex].tagIds`, which is the **union of all tag
IDs across every observation from that camera this cycle** — both MegaTag1 and MegaTag2 queues
(see the shared `tagIds` set at
[VisionIOLimelight.java:83](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L83)).
So the flag can be `false` because of a tag that appeared in a *different* frame than the
observation being scored.

Root cause: `PoseObservation`
([VisionIO.java:28-34](../src/main/java/frc/robot/subsystems/vision/VisionIO.java#L28-L34))
carries no tag IDs, so per-observation reasoning is impossible.

**Recommendations.**

1. Add `int[] tagIds` to the `PoseObservation` record and populate it in each IO impl (the
   Limelight parser already walks the tag block at stride 7 from index 11; collect per-sample
   instead of into the shared set). This unlocks per-observation trench detection, per-tag
   allow/deny lists, and much better logging.
2. Hoist the trench-tag IDs `{1, 6, 7, 12, 17, 22, 23, 28}` out of the `||` chain into a
   `Set<Integer>` constant in `FieldConstants` with a comment naming *why* these tags are
   distrusted. As written it is eight magic numbers in an unsorted chain.
3. Note the empty-list edge case: if `tagIds` is empty the `for` loop never runs and
   `onlySeesTrenchTags` stays `true`. Harmless today (no tags means no observations survive
   `tagCount() == 0`), but it is an accident waiting to bite. Initialize from
   `tagIds.length > 0`.
4. The trench factor currently only nudges *selection* (`doubtIndex *= 1.1`,
   [Vision.java:159](../src/main/java/frc/robot/subsystems/vision/Vision.java#L159)) — if a
   trench pose is nonetheless selected it is trusted at full weight. If trench tags are genuinely
   less reliable, apply the factor to the **std-devs** instead, which is what the commented-out
   `trenchTagStdDevFactor` at
   [Vision.java:39-40](../src/main/java/frc/robot/subsystems/vision/Vision.java#L39-L40) looks
   like it was reaching for. A 1.1x selection nudge is close to a no-op; a std-dev multiplier
   actually expresses "believe this less."

---

### Finding H — Dead and misleading code

| Location | Issue | Recommendation |
|---|---|---|
| [VisionConstants.java:49-53](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L49-L53) | `cameraStdDevFactors` is declared and never read. The upstream template multiplies it into the per-camera std-dev. | Either apply it (`stdDevFactor *= cameraStdDevFactors[trustedCamera]`) or delete it. A declared-but-ignored tuning knob is worse than none — someone will tune it and see nothing happen. |
| [VisionConstants.java:40](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L40) | `maxRotsPerSecond` is compared against `omegaRadiansPerSecond` — the name says rotations, the value is radians. | Rename to `maxAngularSpeedRadPerSec`, or store degrees and convert at the comparison. Misnamed units are how a 114 deg/s gate ends up looking like a 720 deg/s gate to the next reader. |
| [Vision.java:84-94](../src/main/java/frc/robot/subsystems/vision/Vision.java#L84-L94) | `trustedObservation` is seeded with a dummy `PoseObservation` whose infinite distance is described as guaranteeing rejection — but it is guarded by `hasConsideredPoses`, so the dummy is never used. | Use `PoseObservation trustedObservation = null` and drop the sentinel, or keep it and delete the now-inaccurate comment. |
| [Vision.java:141](../src/main/java/frc/robot/subsystems/vision/Vision.java#L141) | `"Must be rotating slower than than maxRotsPerSecond"` — doubled "than". | Trivial typo fix. |
| [Vision.java:142](../src/main/java/frc/robot/subsystems/vision/Vision.java#L142) | The angular-rate check is inside the per-observation loop but does not depend on the observation. | Hoist above the camera loop into a single `boolean robotMovingTooFast`. Cheap, and makes it obvious in logs that the whole cycle was gated rather than individual poses. |
| [VisionConstants.java:24-35](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L24-L35) | `robotToCamera0/1` are `Transform3d.kZero` with the real transforms commented out. Harmless for Limelight (configured in the web UI) but **silently wrong** for `VisionIOPhotonVision`, which uses them in the pose math. | Restore real values, or add a comment that the PhotonVision path is non-functional until they are filled in. As-is, uncommenting the PhotonVision camera in `RobotContainer` produces plausible-looking but wrong poses. |

---

### Finding I — Array-bounds robustness in the Limelight parser

[VisionIOLimelight.java:85-135](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L85-L135)
guards only `rawSample.value.length == 0`, then indexes `[6]`, `[7]`, `[9]`, and iterates from
`[11]`. A short-but-nonzero array (partial NT write, firmware change, malformed publish) throws
`ArrayIndexOutOfBoundsException` out of `updateInputs`, out of `Vision.periodic()`, and because
this runs inside `CommandScheduler.run()`, it takes down the robot code loop mid-match.

**Recommendation.** Change the guard to a minimum-length check and log the drop:

```java
if (rawSample.value.length < 11) continue;   // 11 = fixed botpose header length
```

The `>= 18` check on the ambiguity index is already correctly defensive — extend the same
discipline to the header fields. This is a cheap change with an outsized worst case avoided.

---

### Finding J — Camera configuration and simulation fidelity (rule 12)

The rule specifies 1280x720, 80 deg FOV, 20 FPS. Real Limelight pipeline settings live in the web
UI and are not in this repo, so this cannot be verified from source.

**Recommendations.**

1. Commit the Limelight pipeline `.vpr` exports (or a short markdown table of resolution / FOV /
   exposure / pipeline index per camera) under `src/main/deploy/` or `docs/`. Right now a
   reflashed Limelight silently changes robot behavior with no record in version control, and
   nobody can tell from the repo whether rule 12 is satisfied.
2. `VisionIOPhotonVisionSim` uses a bare `new SimCameraProperties()`
   ([VisionIOPhotonVisionSim.java:44](../src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java#L44)),
   which is 960x720 / 90 deg FOV / zero latency / zero noise. Sim vision is therefore *better*
   than real vision, which is the wrong direction for tuning filters. Configure it to match:

```java
var cameraProperties = new SimCameraProperties();
cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(80));
cameraProperties.setCalibError(0.25, 0.08);
cameraProperties.setFPS(20);
cameraProperties.setAvgLatencyMs(35);
cameraProperties.setLatencyStdDevMs(5);
```

3. Sim currently instantiates `Vision` with **no cameras at all**
   ([RobotContainer.java:155](../src/main/java/frc/robot/RobotContainer.java#L155)), so none of
   this filtering logic is exercised in sim or replay. Wiring up two
   `VisionIOPhotonVisionSim` instances would let the whole rejection pipeline be
   regression-tested off-robot — worth doing before re-tuning the thresholds in this document, so
   changes can be sanity-checked before they touch the field.

---

### Finding K — Disabled throttle defeats pre-match pose seeding

[VisionConstants.java:62-63](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L62-L63)
sets `DISABLED_THROTTLE = 120` and `Robot.disabledInit()` applies it
([Robot.java:143](../src/main/java/frc/robot/Robot.java#L143)). Limelight's `throttle_set` is a
**frames-to-skip count**, not a frame rate — the comment "skip 120 frames per second while
disabled" is inaccurate. At 120 the camera processes roughly one frame in 120, i.e. seconds
between pose updates.

Disabled is precisely when the robot sits on the starting line and most needs to converge on a
good pose before auto begins. `unthrottleLimelights()` is called in `autonomousInit()`
([Robot.java:160](../src/main/java/frc/robot/Robot.java#L160)), i.e. *after* auto has already
started — so the first path may run on a stale pose.

**Recommendations.**

1. Lower `DISABLED_THROTTLE` to something like 5–10 (still a large thermal/power saving, but
   several pose updates per second), **or** un-throttle in `disabledPeriodic()` once
   `DriverStation.isDSAttached()` / FMS-attached indicates a real match is imminent.
2. Also fix the comment to describe frame-skipping rather than fps.
3. Consider an explicit pre-match seeding step: while disabled, if a high-confidence multi-tag
   observation is available, call `drive.setPose(...)` rather than merely nudging the filter.
   This pairs naturally with Finding C option 2 (gyro-owns-heading), which requires a correct
   initial heading to be viable.

---

### Finding L — Optional: teleport / divergence rejection

Not in the rule set, but a common companion to the above: reject observations that disagree with
the current estimate by more than a large margin (e.g. 1.0 m while enabled), which catches
mis-IDed tags and mirrored-field solutions that pass every other filter. Gate it on
`DriverStation.isEnabled()` so it never blocks initial convergence, and log rejections so a
genuinely lost robot is diagnosable rather than silently stuck. Treat this as a later addition —
land Findings A–F first and see whether it is still needed.

---

## Suggested `VisionConstants` shape

Consolidating the numeric recommendations. Values marked **(!)** are meaningful behavior changes
that should be validated on the practice field, not merged blind.

```java
// --- Ambiguity (rules 1, 2) ---
public static double maxAmbiguityMultiTag  = 0.2;   // (!) new gate, was unchecked
public static double maxAmbiguitySingleTag = 0.4;   // (!) loosened from 0.3

// --- Distance (rule 4) ---
public static double maxSingleTagDistance = 4.0;    // (!) new hard reject
public static double maxMultiTagDistance  = 6.0;    // new, softer

// --- Speed gates (rules 5, 6) ---
public static double maxLinearSpeedForVision  = 3.5;   // (!) new; robot tops out at 5.12
public static double maxAngularSpeedRadPerSec =
    Units.degreesToRadians(540.0);                     // (!) was 2.0 rad/s
                                                       //     consider 4-6 rad/s interim

// --- Std devs (rules 7, 8, 9, 10) ---
public static double linearStdDevBaselineMultiTag  = 0.05;  // (!) was 0.02 for all
public static double linearStdDevBaselineSingleTag = 0.10;  // (!) new
public static double angularStdDevBaseline         = 0.06;  // unchanged
public static double singleTagAngularStdDevPenalty = 2.5;   // (!) new; see Finding E3 re: 0.75

// --- MegaTag2 (rule 11) --- both unchanged, both correct
public static double linearStdDevMegatag2Factor  = 0.5;
public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

// --- Kalman clamp (rule 15) --- linear term only
public static double minLinearStdDev = 0.02;
public static double maxLinearStdDev = 3.0;

// --- Latency (rule 14) ---
public static double maxObservationAgeSeconds = 0.10;

// --- Z / bounds --- unchanged
public static double maxZError = 0.1;
```

On the angular gate specifically: jumping straight from 2.0 rad/s to 9.42 rad/s (540 deg/s) is a
4.7x loosening and will admit a lot of previously-rejected data at once. Recommend an interim
value around 4–6 rad/s, with `Vision/AngularSpeedAtRejection` logged, and only go to the full
540 deg/s once the logs show the admitted poses are actually clean. The rule sheet's 540 deg/s is
a ceiling, not a target.

---

## Suggested ordering

Grouped so each stage is independently testable and independently revertible.

**Stage 1 — bug fixes, low risk, no re-tuning needed**
1. Finding A — fix the inverted `doubtIndex` (one line, wrong today)
2. Finding I — length guard in the Limelight parser (prevents a mid-match crash)
3. Finding H — rename `maxRotsPerSecond`, delete or apply `cameraStdDevFactors`, typo, hoist the
   speed check
4. Finding G.2 / G.3 — trench tag `Set` constant, empty-list init

**Stage 2 — missing filters, moderate risk**
5. Finding D1 — single-tag distance cutoff
6. Finding D2 — latency cap (log the age first, then set the threshold)
7. Finding D3 — linear speed gate
8. Finding E1 / E2 — split ambiguity thresholds

**Stage 3 — trust model, needs practice-field validation**
9. Finding F2 — Kalman clamp (do this *before* F1; it bounds the blast radius)
10. Finding F1 — std-dev baselines (!) de-trusts vision, re-check auto-align
11. Finding E3 — single-tag theta penalty (resolve the 0.75-vs-2.5 contradiction first)
12. Finding C — break the MegaTag2 rotation feedback loop

**Stage 4 — architecture**
13. Finding B — feed all accepted observations, or at least multi-tag-tier-first
14. Finding G.1 — add `tagIds` to `PoseObservation`
15. Finding J — sim camera fidelity, commit pipeline configs
16. Findings K, L — throttle policy, pre-match seeding, divergence rejection

Findings B and C are the two that most affect real pose quality, but both depend on Stage 1–3
groundwork (and C changes heading behavior), so they land later on purpose.

---

## What is already right

Worth stating explicitly, since the list above is all criticism:

- The `distance^2 / tagCount` std-dev formula
  ([Vision.java:176-177](../src/main/java/frc/robot/subsystems/vision/Vision.java#L176-L177))
  matches rule 9 exactly.
- Latency compensation is done in the IO layer against the NT sample timestamp in the FPGA
  timebase ([VisionIOLimelight.java:93](../src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java#L93)),
  which is what rule 13 asks for.
- `angularStdDevMegatag2Factor = POSITIVE_INFINITY` correctly refuses MegaTag2 rotation — better
  than the rule sheet, which is silent on it.
- `linearStdDevMegatag2Factor = 0.5` matches rule 11 on the nose.
- Zeroing MegaTag2 ambiguity is the right *reasoning* (already disambiguated), even though it
  opens the gap in E2.
- Using `readQueue()` rather than `get()` captures every frame between loops — the right
  foundation for Finding B, and a step many teams skip.
- Rejected-vs-considered pose logging plus the per-camera disconnect `Alert`s make this
  debuggable at competition, which is more than most vision code manages.
- The `LoggedTunableNumber` pattern is already in place, so the new thresholds above have an
  obvious home for field tuning.
