// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.LoggedTunableNumber;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private final Supplier<ChassisSpeeds> robotRelativeSpeedSupplier;

  // private final LoggedTunableNumber trenchTagStdDevFactor =
  //     new LoggedTunableNumber("Vision/Trench Std Dev Factor", 10.5414);

  private final LoggedTunableNumber trenchTrustFactor =
      new LoggedTunableNumber("Vision/Trench Trust Deviation", 1.1);

  public Vision(
      VisionConsumer consumer, Supplier<ChassisSpeeds> robotSpeedSupplier, VisionIO... io) {
    this.consumer = consumer;
    this.robotRelativeSpeedSupplier = robotSpeedSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    ChassisSpeeds robotRelativeSpeeds = robotRelativeSpeedSupplier.get();

    PoseObservation trustedObservation =
        new PoseObservation(
            0,
            new Pose3d(),
            1.0,
            1,
            Double.POSITIVE_INFINITY, //  high value to guarantee this pose isn't accepted
            PoseObservationType.MEGATAG_1);

    int trustedCamera = -1;
    double trustedobservationDoubtIndex = Double.POSITIVE_INFINITY;
    boolean hasConsideredPoses = false;

    // Initialize logging values
    List<Pose3d> posesRejected = new LinkedList<>();
    List<Pose3d> posesConsidered = new LinkedList<>();

    // TODO: Insert logic to pick a best camera instead of looping over cameras and processing all
    // poses.

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      boolean onlySeesTrenchTags = true;
      for (int tagId : inputs[cameraIndex].tagIds) {
        boolean isTrenchTag =
            tagId == 6
                || tagId == 7
                || tagId == 12
                || tagId == 1
                || tagId == 17
                || tagId == 28
                || tagId == 22
                || tagId == 23;

        if (!isTrenchTag) {
          onlySeesTrenchTags = false;
          break;
        }
      }

      for (var observation : inputs[cameraIndex].poseObservations) {

        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth()

                // Must be rotating slower than than maxRotsPerSecond
                || (Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) >= maxRotsPerSecond);

        // adding pose to logs

        // if the pose is rejected than don't consider it for best pose
        if (rejectPose) {
          posesRejected.add(observation.pose());
          continue;
        }

        hasConsideredPoses = true;
        posesConsidered.add(observation.pose());

        // minimize doubt index (lowest cumulative distance (and ambiguity if MT1) )
        double doubtIndex = observation.averageTagDistance() * observation.tagCount();
        if (observation.type() == PoseObservationType.MEGATAG_1)
          doubtIndex *= (1 + observation.ambiguity());
        if (onlySeesTrenchTags) doubtIndex *= trenchTrustFactor.get();

        // compare to trustedIndex to see if it's lower
        if (doubtIndex <= trustedobservationDoubtIndex) {
          trustedObservation = observation;
          trustedobservationDoubtIndex = doubtIndex;
          trustedCamera = cameraIndex;
        }
      }
    }

    Pose3d trustedPose = Pose3d.kZero;

    // only update the pose if there is a valid pose to update it with
    if (hasConsideredPoses) {
      trustedPose = trustedObservation.pose();

      double stdDevFactor =
          Math.pow(trustedObservation.averageTagDistance(), 2.0) / trustedObservation.tagCount();
      double linearStdDev = linearStdDevBaseline * stdDevFactor;
      double angularStdDev = angularStdDevBaseline * stdDevFactor;

      if (trustedObservation.type() == PoseObservationType.MEGATAG_2) {
        linearStdDev *= linearStdDevMegatag2Factor;
        angularStdDev *= angularStdDevMegatag2Factor;
      }
      consumer.accept(
          trustedPose.toPose2d(),
          trustedObservation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/ObservationsRejected", posesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/ObservationsConsidered", posesConsidered.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/TrustedPose", trustedPose);
    Logger.recordOutput("Vision/Summary/trustedCamera", trustedCamera);
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public void captureRewind(int time) {
    for (VisionIO camera : io) {
      camera.captureRewind(time);
    }
  }

  public void throttleLimelights() {
    for (VisionIO camera : io) {
      camera.setThrottle(VisionConstants.DISABLED_THROTTLE);
    }
  }

  public void unthrottleLimelights() {
    for (VisionIO camera : io) {
      camera.setThrottle(VisionConstants.ENABLED_THROTTLE);
    }
  }
}
