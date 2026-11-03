package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class LinearAccelLimiterTest {
  private static final double EPSILON = 1e-9;

  private static void assertTranslation(double x, double y, Translation2d actual) {
    assertEquals(x, actual.getX(), EPSILON);
    assertEquals(y, actual.getY(), EPSILON);
  }

  @Test
  void rampsUpFromRestWithoutOvershooting() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    Translation2d target = new Translation2d(1, 0);
    assertTranslation(0.5, 0, limiter.calculate(target, 1, 0.5));
    assertTranslation(1, 0, limiter.calculate(target, 1, 0.5));
    assertTranslation(1, 0, limiter.calculate(target, 1, 0.5));
  }

  @Test
  void lettingGoStopsImmediately() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    limiter.reset(new Translation2d(1, 0));
    assertTranslation(0, 0, limiter.calculate(new Translation2d(0, 0), 1, 0.5));
  }

  @Test
  void partialBrakingIsImmediate() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    limiter.reset(new Translation2d(1, 0));
    assertTranslation(0.3, 0, limiter.calculate(new Translation2d(0.3, 0), 1, 0.5));
  }

  @Test
  void reversalBrakesInstantlyThenRampsTheOtherWay() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    limiter.reset(new Translation2d(1, 0));
    assertTranslation(-0.5, 0, limiter.calculate(new Translation2d(-1, 0), 1, 0.5));
  }

  @Test
  void turningAtSpeedIsRateLimited() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    limiter.reset(new Translation2d(1, 0));
    assertTranslation(0, 0.5, limiter.calculate(new Translation2d(0, 1), 1, 0.5));
  }

  @Test
  void resetMakesTheNextOutputStartFromTheGivenVelocity() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    limiter.reset(new Translation2d(2, 0));
    assertTranslation(2, 0, limiter.calculate(new Translation2d(2, 0), 1, 0.02));
  }

  @Test
  void infiniteMaxAccelPassesTheTargetStraightThrough() {
    LinearAccelLimiter limiter = new LinearAccelLimiter();
    assertTranslation(
        3, 4, limiter.calculate(new Translation2d(3, 4), Double.POSITIVE_INFINITY, 0.02));
  }
}
