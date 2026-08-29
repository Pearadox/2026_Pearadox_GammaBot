package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.CameraMountCalibration.Result;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The transform algebra in CameraMountCalibration is exactly the kind of code that looks right and
 * is silently mirrored, so these tests build a world with a KNOWN answer, perturb one camera's
 * mount, and check that the solver recovers the perturbation.
 */
class CameraMountCalibrationTest {
  private static final double EPS = 1e-9;

  // A robot somewhere unremarkable, deliberately not axis-aligned.
  private static final Pose3d TRUE_ROBOT_POSE =
      new Pose3d(4.2, 3.1, 0.0, new Rotation3d(0.0, 0.0, Units.degreesToRadians(37.0)));

  // Two plausible camera mounts, both off-center and tilted, again deliberately not symmetric.
  private static final Pose3d TRUE_MOUNT_0 =
      new Pose3d(0.31, 0.28, 0.24, new Rotation3d(0.0, Units.degreesToRadians(-20.0), Math.PI));
  private static final Pose3d TRUE_MOUNT_1 =
      new Pose3d(
          -0.27, -0.07, 0.37, new Rotation3d(0.0, Units.degreesToRadians(-15.0), -Math.PI * 0.5));

  /**
   * What a Limelight publishes: it sees the camera at cameraFieldPose and reports the robot pose
   * implied by the mount it has been CONFIGURED with, which may be wrong.
   */
  private static Pose3d reportedBotPose(Pose3d cameraFieldPose, Pose3d configuredMount) {
    return cameraFieldPose.transformBy(
        CameraMountCalibration.asTransform(configuredMount).inverse());
  }

  private static Pose3d trueCameraFieldPose(Pose3d mount) {
    return TRUE_ROBOT_POSE.transformBy(CameraMountCalibration.asTransform(mount));
  }

  private static void assertPoseEquals(Pose3d expected, Pose3d actual, double eps) {
    Transform3d delta = new Transform3d(expected, actual);
    assertEquals(0.0, delta.getTranslation().getNorm(), eps, "translation mismatch");
    assertEquals(0.0, delta.getRotation().getAngle(), eps, "rotation mismatch");
  }

  @Test
  void reportedBotPoseRoundTripsThroughCameraFieldPose() {
    // The inversion the whole approach rests on: given the botpose a camera reported and the mount
    // it used, we get its true field pose back exactly.
    Pose3d cameraField = trueCameraFieldPose(TRUE_MOUNT_0);
    Pose3d botPose = reportedBotPose(cameraField, TRUE_MOUNT_0);

    assertPoseEquals(TRUE_ROBOT_POSE, botPose, EPS);
    assertPoseEquals(
        cameraField, CameraMountCalibration.cameraFieldPose(botPose, TRUE_MOUNT_0), EPS);
  }

  @Test
  void recoversAPerturbedMountExactlyWhenTheOtherCameraIsTrusted() {
    // Camera 1's mount is wrong: 3 cm out and 2.5 degrees of yaw off, the kind of error a
    // swapped-in camera has when its extrinsics were re-entered by hand.
    Pose3d perturbedMount1 =
        TRUE_MOUNT_1.transformBy(
            new Transform3d(
                0.03, -0.01, 0.005, new Rotation3d(0.0, 0.0, Units.degreesToRadians(2.5))));

    Pose3d botPose0 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_0), TRUE_MOUNT_0);
    Pose3d botPose1 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_1), perturbedMount1);

    // errorShare 1.0 means "camera 0 is right", which it is here.
    Result result =
        CameraMountCalibration.solve(botPose0, TRUE_MOUNT_0, botPose1, perturbedMount1, 1.0);

    assertPoseEquals(TRUE_ROBOT_POSE, result.estimatedRobotPose(), EPS);
    assertPoseEquals(TRUE_MOUNT_0, result.camera0().suggestedMount(), EPS);
    assertPoseEquals(TRUE_MOUNT_1, result.camera1().suggestedMount(), EPS);
  }

  @Test
  void splittingTheErrorMakesTheTwoCamerasAgree() {
    Pose3d perturbedMount1 =
        TRUE_MOUNT_1.transformBy(
            new Transform3d(
                0.03, -0.01, 0.005, new Rotation3d(0.0, 0.0, Units.degreesToRadians(2.5))));

    Pose3d botPose0 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_0), TRUE_MOUNT_0);
    Pose3d botPose1 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_1), perturbedMount1);

    Result result =
        CameraMountCalibration.solve(botPose0, TRUE_MOUNT_0, botPose1, perturbedMount1, 0.5);

    // Neither suggestion is the truth any more (the error was split), but the point of splitting is
    // that both cameras now report the SAME robot pose, which is what stops the estimator shaking.
    Pose3d newBotPose0 =
        reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_0), result.camera0().suggestedMount());
    Pose3d newBotPose1 =
        reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_1), result.camera1().suggestedMount());

    assertPoseEquals(newBotPose0, newBotPose1, 1e-6);
  }

  @Test
  void disagreementReportsTheActualGapBetweenTheCameras() {
    Pose3d perturbedMount1 =
        TRUE_MOUNT_1.transformBy(new Transform3d(0.03, 0.0, 0.0, new Rotation3d()));

    Pose3d botPose0 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_0), TRUE_MOUNT_0);
    Pose3d botPose1 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_1), perturbedMount1);

    Result result =
        CameraMountCalibration.solve(botPose0, TRUE_MOUNT_0, botPose1, perturbedMount1, 0.5);

    // A pure 3 cm mount translation error shows up as a 3 cm botpose disagreement.
    assertEquals(0.03, result.disagreement().getTranslation().getNorm(), 1e-9);
  }

  @Test
  void aCorrectlyConfiguredPairSuggestsNoChange() {
    Pose3d botPose0 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_0), TRUE_MOUNT_0);
    Pose3d botPose1 = reportedBotPose(trueCameraFieldPose(TRUE_MOUNT_1), TRUE_MOUNT_1);

    Result result =
        CameraMountCalibration.solve(botPose0, TRUE_MOUNT_0, botPose1, TRUE_MOUNT_1, 0.5);

    assertEquals(0.0, result.disagreement().getTranslation().getNorm(), EPS);
    assertPoseEquals(TRUE_MOUNT_0, result.camera0().suggestedMount(), EPS);
    assertPoseEquals(TRUE_MOUNT_1, result.camera1().suggestedMount(), EPS);
  }

  @Test
  void limelightArrayConversionRoundTrips() {
    double[] array = CameraMountCalibration.mountToLimelightArray(TRUE_MOUNT_1);
    assertPoseEquals(TRUE_MOUNT_1, CameraMountCalibration.mountFromLimelightArray(array), 1e-12);
  }

  @Test
  void averagerAndSpreadDescribeAJitteryCamera() {
    var averager = new CameraMountCalibration.PoseAverager();
    List<Pose3d> samples =
        List.of(
            TRUE_ROBOT_POSE.transformBy(new Transform3d(0.01, 0.0, 0.0, new Rotation3d())),
            TRUE_ROBOT_POSE.transformBy(new Transform3d(-0.01, 0.0, 0.0, new Rotation3d())),
            TRUE_ROBOT_POSE);
    samples.forEach(averager::add);

    assertEquals(3, averager.getCount());
    assertPoseEquals(TRUE_ROBOT_POSE, averager.getMean(), 1e-9);

    var spread = CameraMountCalibration.spread(samples, averager.getMean());
    assertTrue(spread.translationStdDevMeters() > 0.005, "expected the 1 cm jitter to show up");
    assertTrue(spread.translationStdDevMeters() < 0.02, "jitter should not be inflated");
    assertEquals(0.0, spread.rotationStdDevDegrees(), 1e-9);
  }
}
