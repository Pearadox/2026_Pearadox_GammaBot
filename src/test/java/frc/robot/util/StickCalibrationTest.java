package frc.robot.util;

import static frc.robot.util.StickCalibration.RIM_BINS;
import static frc.robot.util.StickCalibration.RIM_CEILING;
import static frc.robot.util.StickCalibration.RIM_DWELL_TICKS;
import static frc.robot.util.StickCalibration.RIM_FLOOR;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.util.StickCalibration.RimTrace;
import frc.robot.util.StickCalibration.TraceState;
import java.util.Arrays;
import java.util.function.IntPredicate;
import org.junit.jupiter.api.Test;

// Mirrors the cases in the driver trainer's src/input/calibration.test.ts (driver-trainer repo).
class StickCalibrationTest {
  private static double deg(double d) {
    return Math.toRadians(d);
  }

  /** Reach of a rounded-square rim (superellipse): 1.0 on the axes, about 1.14 on diagonals. */
  private static double superellipse(double angleRad) {
    double p = 3.2;
    double c = Math.abs(Math.cos(angleRad));
    double s = Math.abs(Math.sin(angleRad));
    return 1 / Math.pow(Math.pow(c, p) + Math.pow(s, p), 1 / p);
  }

  private static double[] squareRim() {
    double[] rim = new double[RIM_BINS];
    for (int i = 0; i < RIM_BINS; i++) {
      rim[i] = superellipse(deg(i * 15));
    }
    return rim;
  }

  private static double[] rampRim() {
    double[] rim = new double[RIM_BINS];
    for (int i = 0; i < RIM_BINS; i++) {
      rim[i] = 0.8 + i * 0.01;
    }
    return rim;
  }

  private static double[] filled(double v) {
    double[] rim = new double[RIM_BINS];
    Arrays.fill(rim, v);
    return rim;
  }

  /** Rolls the stick along the superellipse at 5 degree steps, skipping rejected angles. */
  private static void sweep(RimTrace trace, IntPredicate keep) {
    for (int d = 0; d < 360; d += 5) {
      if (!keep.test(d)) {
        continue;
      }
      double r = superellipse(deg(d));
      trace.update(r * Math.cos(deg(d)), r * Math.sin(deg(d)));
    }
  }

  private static void sweep(RimTrace trace) {
    sweep(trace, d -> true);
  }

  @Test
  void binIndexRoundsToTheNearestCenterAndWraps() {
    assertEquals(0, StickCalibration.binIndex(0));
    assertEquals(0, StickCalibration.binIndex(deg(359.9)));
    assertEquals(0, StickCalibration.binIndex(deg(7.4)));
    assertEquals(1, StickCalibration.binIndex(deg(7.6)));
    assertEquals(23, StickCalibration.binIndex(deg(-15)));
    assertEquals(6, StickCalibration.binIndex(deg(90)));
  }

  @Test
  void rimAtIsExactAtBinCenters() {
    double[] rim = rampRim();
    assertEquals(0.82, StickCalibration.rimAt(rim, deg(30)), 1e-9);
    assertEquals(1.03, StickCalibration.rimAt(rim, deg(345)), 1e-9);
  }

  @Test
  void rimAtIsHalfwayBetweenNeighborsAtTheMidpoint() {
    assertEquals(0.825, StickCalibration.rimAt(rampRim(), deg(37.5)), 1e-9);
  }

  @Test
  void rimAtIsContinuousAcrossTheSeam() {
    double[] rim = rampRim();
    assertEquals((1.03 + 0.8) / 2, StickCalibration.rimAt(rim, deg(352.5)), 1e-9);
    assertEquals((1.03 + 0.8) / 2, StickCalibration.rimAt(rim, deg(-7.5)), 1e-9);
    assertEquals(0.8, StickCalibration.rimAt(rim, deg(359.999)), 1e-3);
    assertEquals(0.8, StickCalibration.rimAt(rim, deg(0.001)), 1e-3);
  }

  @Test
  void applyMakesAFullDiagonalPushReadOne() {
    double[] out = StickCalibration.apply(squareRim(), 0.806, 0.806);
    assertTrue(Math.abs(Math.hypot(out[0], out[1]) - 1) < 0.02);
    assertEquals(deg(45), Math.atan2(out[1], out[0]), 1e-9);
  }

  @Test
  void applyLeavesAFullForwardPushAlone() {
    double[] out = StickCalibration.apply(squareRim(), 1, 0);
    assertEquals(1, out[0], 1e-9);
    assertEquals(0, out[1], 1e-9);
  }

  @Test
  void applyKeepsZeroAtZeroAndIsTheIdentityWithNoRim() {
    assertArrayEquals(new double[] {0, 0}, StickCalibration.apply(squareRim(), 0, 0));
    assertArrayEquals(new double[] {0.5, -0.7}, StickCalibration.apply(null, 0.5, -0.7));
  }

  @Test
  void traceFillsEveryBinAndFinishesOnlyAfterRestingNearCenter() {
    RimTrace t = new RimTrace();
    sweep(t);
    assertEquals(RIM_BINS, t.seen());
    assertTrue(t.complete());
    assertNull(t.result());
    for (int i = 0; i < RIM_DWELL_TICKS - 1; i++) {
      assertEquals(TraceState.TRACING, t.update(0.05, 0));
    }
    assertEquals(TraceState.DONE, t.update(0, 0));
    double[] r = t.result();
    assertNotNull(r);
    assertEquals(RIM_BINS, r.length);
    for (double v : r) {
      assertTrue(v >= RIM_FLOOR && v <= RIM_CEILING);
    }
    assertEquals(superellipse(deg(45)), r[3], 1e-6);
  }

  @Test
  void traceRestartsTheRestCountWhenTheStickLeavesCenter() {
    RimTrace t = new RimTrace();
    sweep(t);
    for (int i = 0; i < RIM_DWELL_TICKS - 1; i++) {
      t.update(0, 0);
    }
    assertEquals(TraceState.TRACING, t.update(0.9, 0));
    for (int i = 0; i < RIM_DWELL_TICKS - 1; i++) {
      assertEquals(TraceState.TRACING, t.update(0, 0));
    }
    assertEquals(TraceState.DONE, t.update(0, 0));
  }

  @Test
  void traceNeverFinishesWhenTheLeftQuadrantIsSkipped() {
    RimTrace t = new RimTrace();
    sweep(t, d -> d < 60 || d > 120);
    assertFalse(t.complete());
    assertTrue(t.seen() < RIM_BINS);
    for (int i = 0; i < 3 * RIM_DWELL_TICKS; i++) {
      assertEquals(TraceState.TRACING, t.update(0, 0));
    }
    assertNull(t.result());
  }

  @Test
  void traceDoesNotCountADirectionThatOnlyReachedHalfway() {
    RimTrace t = new RimTrace();
    sweep(t, d -> d != 90 && d != 85 && d != 95);
    t.update(0, 0.5);
    assertEquals(RIM_BINS - 1, t.seen());
    assertFalse(t.complete());
  }

  @Test
  void traceResultClampsToTheFloorAndCeiling() {
    RimTrace t = new RimTrace();
    for (int d = 0; d < 360; d += 5) {
      double r = d == 45 ? 2.0 : 0.8;
      t.update(r * Math.cos(deg(d)), r * Math.sin(deg(d)));
    }
    for (int i = 0; i < RIM_DWELL_TICKS; i++) {
      t.update(0, 0);
    }
    double[] r = t.result();
    assertNotNull(r);
    assertEquals(RIM_CEILING, r[3], 1e-9);
    assertEquals(0.8, r[0], 1e-9);
  }

  @Test
  void sanitizeAcceptsAFullRimAndRejectsBadShapesAndValues() {
    double[] good = filled(1);
    assertArrayEquals(good, StickCalibration.sanitize(good));
    assertNull(StickCalibration.sanitize(Arrays.copyOf(good, RIM_BINS - 1)));
    assertNull(StickCalibration.sanitize(Arrays.copyOf(good, RIM_BINS + 1)));
    assertNull(StickCalibration.sanitize(null));
    double[][] badLast = {
      {Double.NaN}, {Double.POSITIVE_INFINITY}, {RIM_FLOOR - 0.01}, {RIM_CEILING + 0.01}
    };
    for (double[] bad : badLast) {
      double[] rim = filled(1);
      rim[RIM_BINS - 1] = bad[0];
      assertNull(StickCalibration.sanitize(rim));
    }
  }

  @Test
  void encodeAndDecodeRoundTripAndDecodeRejectsGarbage() {
    double[] rim = squareRim();
    double[] back = StickCalibration.decode(StickCalibration.encode(rim));
    assertNotNull(back);
    assertArrayEquals(rim, back, 1e-4);
    assertNull(StickCalibration.decode(""));
    assertNull(StickCalibration.decode(null));
    assertNull(StickCalibration.decode("1,2,3"));
    assertNull(StickCalibration.decode("x" + StickCalibration.encode(rim)));
  }
}
