package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class StickCutoffLatchTest {
  @Test
  void belowTheCutoffNeverTrips() {
    StickCutoffLatch latch = new StickCutoffLatch();
    assertFalse(latch.update(0.5, 0.9, 0.1));
  }

  @Test
  void aboveTheCutoffTripsAndHoldsUntilReturnedToCenter() {
    StickCutoffLatch latch = new StickCutoffLatch();
    assertTrue(latch.update(0.95, 0.9, 0.1));
    assertTrue(latch.update(0.5, 0.9, 0.1));
    assertFalse(latch.update(0.05, 0.9, 0.1));
    assertFalse(latch.update(0.5, 0.9, 0.1));
  }

  @Test
  void exactlyAtTheCutoffDoesNotTrip() {
    StickCutoffLatch latch = new StickCutoffLatch();
    assertFalse(latch.update(0.9, 0.9, 0.1));
  }

  @Test
  void cutoffOfOneNeverTripsEvenAtFullDeflection() {
    StickCutoffLatch latch = new StickCutoffLatch();
    assertFalse(latch.update(1.0, 1.0, 0.1));
  }

  @Test
  void resetClearsATrippedLatch() {
    StickCutoffLatch latch = new StickCutoffLatch();
    assertTrue(latch.update(0.95, 0.9, 0.1));
    latch.reset();
    assertFalse(latch.isTripped());
  }
}
