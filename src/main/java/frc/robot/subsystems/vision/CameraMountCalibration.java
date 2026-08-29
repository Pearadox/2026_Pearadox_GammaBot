package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * Turns "the two Limelights disagree about where the robot is" into "here is the mount pose to type
 * into each camera's web UI".
 *
 * <p>The trick is that a Limelight's botpose is just the camera's own field pose pushed back
 * through the mount pose configured in its web UI. That is invertible: given the botpose it
 * reported and the mount it was configured with, the camera's true field pose comes back exactly.
 * Two cameras looking at the same tags from a stationary robot therefore give two independent
 * measurements of one robot pose, and their disagreement is mount error. Pick which robot pose to
 * believe, and each camera's corrected mount follows.
 *
 * <p>Everything here is pure geometry: no NetworkTables, no HAL, no subsystem state, so it is
 * unit-testable. {@link frc.robot.commands.CalibrateCameraMounts} does the sampling and reporting.
 */
public final class CameraMountCalibration {
  private CameraMountCalibration() {}

  /** A camera's configured mount, and what it should be instead. */
  public record MountSuggestion(Pose3d currentMount, Pose3d suggestedMount) {
    /** The correction, expressed in the current mount's own frame. */
    public Transform3d correction() {
      return new Transform3d(currentMount, suggestedMount);
    }
  }

  /**
   * @param estimatedRobotPose the robot pose the corrections are built against
   * @param disagreement camera0's botpose to camera1's botpose: the raw "how bad is it" number
   */
  public record Result(
      Pose3d estimatedRobotPose,
      Transform3d disagreement,
      MountSuggestion camera0,
      MountSuggestion camera1) {}

  /** A mount pose as a transform from the robot origin. */
  public static Transform3d asTransform(Pose3d mount) {
    return new Transform3d(new Pose3d(), mount);
  }

  /** Recovers where the camera actually sat on the field from the botpose it reported. */
  public static Pose3d cameraFieldPose(Pose3d reportedBotPose, Pose3d configuredMount) {
    return reportedBotPose.transformBy(asTransform(configuredMount));
  }

  /**
   * The mount that would have made a camera at {@code cameraFieldPose} report {@code
   * trueRobotPose}.
   */
  public static Pose3d correctedMount(Pose3d trueRobotPose, Pose3d cameraFieldPose) {
    Transform3d robotToCamera = new Transform3d(trueRobotPose, cameraFieldPose);
    return new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation());
  }

  /**
   * Attributes the disagreement between the two cameras.
   *
   * @param errorShareTowardCamera0 0.0 blames camera0 entirely (camera1's answer is taken as
   *     truth), 1.0 blames camera1 entirely, 0.5 splits the difference. Splitting is the right
   *     default when neither mount is independently trusted; use 1.0 when one camera's mount is
   *     known good, for example after the other was swapped.
   */
  public static Result solve(
      Pose3d botPose0,
      Pose3d configuredMount0,
      Pose3d botPose1,
      Pose3d configuredMount1,
      double errorShareTowardCamera0) {
    // Geodesic interpolation, so rotations blend along the shortest arc rather than by averaging
    // roll/pitch/yaw numbers, which is wrong near wrap-around.
    Pose3d estimatedRobotPose = botPose1.interpolate(botPose0, errorShareTowardCamera0);

    Pose3d cameraField0 = cameraFieldPose(botPose0, configuredMount0);
    Pose3d cameraField1 = cameraFieldPose(botPose1, configuredMount1);

    return new Result(
        estimatedRobotPose,
        new Transform3d(botPose0, botPose1),
        new MountSuggestion(configuredMount0, correctedMount(estimatedRobotPose, cameraField0)),
        new MountSuggestion(configuredMount1, correctedMount(estimatedRobotPose, cameraField1)));
  }

  // ---- Limelight camerapose_robotspace array conversions ----
  // Limelight publishes and accepts [x, y, z, roll, pitch, yaw] with meters and DEGREES.

  public static Pose3d mountFromLimelightArray(double[] a) {
    if (a == null || a.length < 6) {
      return new Pose3d();
    }
    return new Pose3d(
        a[0],
        a[1],
        a[2],
        new Rotation3d(
            Units.degreesToRadians(a[3]),
            Units.degreesToRadians(a[4]),
            Units.degreesToRadians(a[5])));
  }

  public static double[] mountToLimelightArray(Pose3d mount) {
    Rotation3d r = mount.getRotation();
    return new double[] {
      mount.getX(),
      mount.getY(),
      mount.getZ(),
      Units.radiansToDegrees(r.getX()),
      Units.radiansToDegrees(r.getY()),
      Units.radiansToDegrees(r.getZ())
    };
  }

  /** Formats a mount for typing straight into the Limelight web UI. */
  public static String formatForWebUI(Pose3d mount) {
    double[] a = mountToLimelightArray(mount);
    return String.format(
        "x=%.4f m  y=%.4f m  z=%.4f m  roll=%.2f deg  pitch=%.2f deg  yaw=%.2f deg",
        a[0], a[1], a[2], a[3], a[4], a[5]);
  }

  /**
   * Running geodesic mean of poses. Each new sample is folded in by interpolating 1/n of the way
   * toward it, which is the incremental form of an average and stays on the manifold.
   */
  public static class PoseAverager {
    private Pose3d mean = new Pose3d();
    private int count = 0;

    public void add(Pose3d sample) {
      count++;
      mean = count == 1 ? sample : mean.interpolate(sample, 1.0 / count);
    }

    public Pose3d getMean() {
      return mean;
    }

    public int getCount() {
      return count;
    }
  }

  /** Spread of a set of poses about a mean, for deciding whether a sample set is trustworthy. */
  public record Spread(double translationStdDevMeters, double rotationStdDevDegrees) {}

  public static Spread spread(Iterable<Pose3d> samples, Pose3d mean) {
    double transSumSq = 0.0;
    double rotSumSq = 0.0;
    int n = 0;

    for (Pose3d sample : samples) {
      Transform3d delta = new Transform3d(mean, sample);
      transSumSq += delta.getTranslation().getNorm() * delta.getTranslation().getNorm();
      double angle = delta.getRotation().getAngle();
      rotSumSq += angle * angle;
      n++;
    }

    if (n == 0) {
      return new Spread(0.0, 0.0);
    }
    return new Spread(Math.sqrt(transSumSq / n), Units.radiansToDegrees(Math.sqrt(rotSumSq / n)));
  }
}
