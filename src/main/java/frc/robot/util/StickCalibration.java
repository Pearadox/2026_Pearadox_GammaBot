package frc.robot.util;

import java.util.Arrays;
import java.util.Locale;

/**
 * Measured rim-trace calibration of a drive stick.
 *
 * <p>A real Xbox stick's rim is a rounded square, not a circle, so a full diagonal push reads up to
 * about 1.14 through hypot while a full forward push reads 1.0. The drive code clamps the magnitude
 * at 1.0, so a stick cutoff trips sooner on the diagonals than on the axes. Scaling forward and
 * left by one over the measured rim reach makes a full push read 1.0 in every direction.
 *
 * <p>This is a mirror of the driver trainer's src/input/calibration.ts (driver-trainer repo). The
 * constants, the bin convention and the behavior are identical, so a pad calibrated in either place
 * reads the same. A change to one changes the other.
 *
 * <p>The pair is (forward, left), the same pair DriveCommands.joystickDrive receives, that is
 * (-leftY, -leftX). The angle is atan2(left, forward).
 *
 * <p>This class is plain Java with no robot code in it, so it can be unit tested on a laptop.
 */
public final class StickCalibration {
  /** 15 degree bins; bin i is centered on i * 15 degrees, bin 0 is stick forward, CCW. */
  public static final int RIM_BINS = 24;

  /** A bin counts as seen once its max reach gets here; a saved value is never below it. */
  public static final double RIM_FLOOR = 0.75;

  /** A saved value is never above this. */
  public static final double RIM_CEILING = 1.5;

  /** Below this reach the stick is resting near center. */
  public static final double RIM_CENTER = 0.3;

  /** Updates at rest that finish a complete trace (0.2 s at the 20 ms loop). */
  public static final int RIM_DWELL_TICKS = 10;

  private static final double BIN_RAD = (2 * Math.PI) / RIM_BINS;

  private StickCalibration() {}

  /**
   * Nearest bin center for an angle, wrapping.
   *
   * @param angleRad angle in radians, 0 is stick forward, counterclockwise
   * @return bin index, 0 to RIM_BINS - 1
   */
  public static int binIndex(double angleRad) {
    int i = (int) (Math.round(angleRad / BIN_RAD) % RIM_BINS);
    return (i + RIM_BINS) % RIM_BINS;
  }

  /**
   * Rim reach at an angle: linear between the two neighboring bin centers, across the seam too.
   *
   * @param rim per-bin reach, length RIM_BINS
   * @param angleRad angle in radians
   * @return interpolated reach
   */
  public static double rimAt(double[] rim, double angleRad) {
    double t = (angleRad / BIN_RAD) % RIM_BINS;
    if (t < 0) {
      t += RIM_BINS;
    }
    int i0 = ((int) Math.floor(t)) % RIM_BINS;
    int i1 = (i0 + 1) % RIM_BINS;
    double f = t - Math.floor(t);
    double a = i0 < rim.length ? rim[i0] : 1.0;
    double b = i1 < rim.length ? rim[i1] : 1.0;
    return a + (b - a) * f;
  }

  /**
   * Scales forward and left so the measured rim reads 1.0. Rotation is never touched. A null rim or
   * a zero stick returns the input unchanged.
   *
   * @param rim per-bin reach, or null for no calibration
   * @param forward stick forward, -leftY
   * @param left stick left, -leftX
   * @return a new {forward, left} pair
   */
  public static double[] apply(double[] rim, double forward, double left) {
    if (rim == null || (forward == 0.0 && left == 0.0)) {
      return new double[] {forward, left};
    }
    double s = 1.0 / rimAt(rim, Math.atan2(left, forward));
    return new double[] {forward * s, left * s};
  }

  /**
   * Accepts only a rim of exactly RIM_BINS finite values inside the floor and ceiling.
   *
   * @param raw candidate rim
   * @return a copy of the rim, or null if it is not valid
   */
  public static double[] sanitize(double[] raw) {
    if (raw == null || raw.length != RIM_BINS) {
      return null;
    }
    for (double v : raw) {
      if (!Double.isFinite(v) || v < RIM_FLOOR || v > RIM_CEILING) {
        return null;
      }
    }
    return Arrays.copyOf(raw, raw.length);
  }

  /**
   * Encodes a rim for storage: comma-separated, 4 decimals.
   *
   * @param rim per-bin reach
   * @return the encoded string
   */
  public static String encode(double[] rim) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < rim.length; i++) {
      if (i > 0) {
        sb.append(',');
      }
      sb.append(String.format(Locale.ROOT, "%.4f", rim[i]));
    }
    return sb.toString();
  }

  /**
   * Decodes a stored rim. Anything malformed or out of range returns null.
   *
   * @param s the encoded string
   * @return the rim, or null
   */
  public static double[] decode(String s) {
    if (s == null || s.isEmpty()) {
      return null;
    }
    String[] parts = s.split(",", -1);
    double[] rim = new double[parts.length];
    try {
      for (int i = 0; i < parts.length; i++) {
        rim[i] = Double.parseDouble(parts[i].trim());
      }
    } catch (NumberFormatException e) {
      return null;
    }
    return sanitize(rim);
  }

  /** Whether a trace is still running or has finished. */
  public enum TraceState {
    TRACING,
    DONE
  }

  /** Records the farthest reach seen in each direction while the driver rolls the stick. */
  public static class RimTrace {
    private final double[] reach = new double[RIM_BINS];
    private int restTicks = 0;
    private boolean finished = false;

    /**
     * Feeds one stick reading into the trace. It finishes on its own once every bin is seen and the
     * stick has rested near center for the dwell. A pause at center before every bin is seen never
     * finishes it.
     *
     * @param forward stick forward, -leftY
     * @param left stick left, -leftX
     * @return TRACING or DONE
     */
    public TraceState update(double forward, double left) {
      if (finished) {
        return TraceState.DONE;
      }
      double h = Math.hypot(forward, left);
      int bin = binIndex(Math.atan2(left, forward));
      if (h > reach[bin]) {
        reach[bin] = h;
      }
      restTicks = h < RIM_CENTER ? restTicks + 1 : 0;
      if (complete() && restTicks >= RIM_DWELL_TICKS) {
        finished = true;
      }
      return finished ? TraceState.DONE : TraceState.TRACING;
    }

    /**
     * Number of directions that have reached the rim.
     *
     * @return bins at or above RIM_FLOOR
     */
    public int seen() {
      int n = 0;
      for (double r : reach) {
        if (r >= RIM_FLOOR) {
          n++;
        }
      }
      return n;
    }

    /**
     * Whether every direction has reached the rim.
     *
     * @return true once all RIM_BINS bins are seen
     */
    public boolean complete() {
      return seen() == RIM_BINS;
    }

    /**
     * Farthest reach per bin so far, for showing the partial trace.
     *
     * @return a copy of the per-bin reach
     */
    public double[] maxes() {
      return Arrays.copyOf(reach, reach.length);
    }

    /**
     * The measured rim, clamped to the floor and ceiling.
     *
     * @return the rim, or null until the trace is done
     */
    public double[] result() {
      if (!finished) {
        return null;
      }
      double[] out = new double[RIM_BINS];
      for (int i = 0; i < RIM_BINS; i++) {
        out[i] = Math.min(Math.max(reach[i], RIM_FLOOR), RIM_CEILING);
      }
      return out;
    }
  }
}
