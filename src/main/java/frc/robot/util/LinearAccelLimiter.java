package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A 2D velocity rate limiter where braking is free and only speeding up is limited.
 *
 * <p>Why braking is free: a limiter that also limited braking would let the robot coast several
 * meters after the driver lets go of the stick, which is unsafe. So letting go always stops the
 * robot immediately, and only speeding up is rate limited. That includes speeding up in a new
 * direction, for example after the driver reverses the stick or swings it sideways.
 */
public class LinearAccelLimiter {
  private Translation2d last = Translation2d.kZero;

  /**
   * Returns the velocity to command this loop, limited so speed only grows at maxAccel.
   *
   * @param target the velocity the driver is asking for (any units, same frame every call)
   * @param maxAccel maximum allowed increase in speed, in target units per second squared
   * @param dtSeconds loop period
   * @return the velocity to command this loop
   */
  public Translation2d calculate(Translation2d target, double maxAccel, double dtSeconds) {
    double lastNorm = last.getNorm();
    Translation2d start = last;

    if (lastNorm > 1e-9) {
      // Unit vector pointing the way the robot is already moving
      Translation2d u = last.div(lastNorm);
      // How much of the target points the way we are already going
      double along = target.getX() * u.getX() + target.getY() * u.getY();
      // Free braking: speed along the current direction may drop instantly, but never below zero
      // and never above the current speed
      double brakedNorm = MathUtil.clamp(along, 0.0, lastNorm);
      start = u.times(brakedNorm);
    }

    // Whatever change is left is speeding up, so cap it at maxAccel times the loop period
    Translation2d delta = target.minus(start);
    double maxStep = maxAccel * dtSeconds;
    double deltaNorm = delta.getNorm();
    if (deltaNorm > maxStep && deltaNorm > 1e-9) {
      delta = delta.times(maxStep / deltaNorm);
    }

    last = start.plus(delta);
    return last;
  }

  /**
   * Sets the limiter's memory to the given velocity, so the next output ramps from there.
   *
   * @param current the velocity the robot is moving at right now
   */
  public void reset(Translation2d current) {
    last = current;
  }

  /**
   * Returns the last velocity this limiter output.
   *
   * @return the last commanded velocity
   */
  public Translation2d getLast() {
    return last;
  }
}
