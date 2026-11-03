package frc.robot.util;

/**
 * A practice-mode latch for the translation stick.
 *
 * <p>Drivers who grew up on video games tend to slam the stick all the way to the rail. On a real
 * robot that means full speed all the time and no fine control. In practice mode we "poison" the
 * top part of the stick travel: if the stick goes past the cutoff, the latch trips and the robot
 * stops moving. It stays stopped until the driver brings the stick back inside the deadband. That
 * forces the driver to learn to feather the stick instead of pinning it.
 *
 * <p>This class is plain Java with no robot code in it, so it can be unit tested on a laptop.
 */
public class StickCutoffLatch {
  private boolean tripped = false;

  /**
   * Updates the latch with the latest stick reading.
   *
   * <p>The re-arm check runs first, so the latch can never get stuck: bringing the stick back near
   * center always clears it.
   *
   * @param magnitude clamped stick magnitude, 0 to 1
   * @param cutoff trip threshold, 0 to 1; a cutoff of 1.0 or more can never trip
   * @param resetBelow re-arm threshold; the latch clears once magnitude is below this
   * @return true while the latch is tripped (the caller should command zero translation)
   */
  public boolean update(double magnitude, double cutoff, double resetBelow) {
    if (magnitude < resetBelow) {
      // Stick is back near center, so re-arm the latch
      tripped = false;
    } else if (magnitude > cutoff) {
      // Stick went past the cutoff, so stop the robot until it comes back to center
      tripped = true;
    }
    return tripped;
  }

  /**
   * Returns whether the latch is currently tripped.
   *
   * @return true if the robot should be held still
   */
  public boolean isTripped() {
    return tripped;
  }

  /** Clears the latch, for example when a drive command starts. */
  public void reset() {
    tripped = false;
  }
}
