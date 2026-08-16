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

  // Std dev multiplier applied when a camera only sees trench tags
  // (values above 1.0 trust those observations less)
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

    // Initialize logging values
    List<Pose3d> posesRejected = new LinkedList<>();
    List<Pose3d> posesAccepted = new LinkedList<>();

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

                // Must be rotating slower than maxYawRateRadPerSec
                || (Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond) >= maxYawRateRadPerSec);

        if (rejectPose) {
          posesRejected.add(observation.pose());
          continue;
        }
        posesAccepted.add(observation.pose());

        // Every accepted observation is sent to the pose estimator, weighted by
        // standard deviations that scale with distance and tag count. The Kalman
        // filter blends all cameras and both MegaTag pipelines instead of the
        // pose snapping to a single winner each loop.
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }
        if (onlySeesTrenchTags) {
          linearStdDev *= trenchTrustFactor.get();
          angularStdDev *= trenchTrustFactor.get();
        }

        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/ObservationsRejected", posesRejected.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/ObservationsAccepted", posesAccepted.toArray(new Pose3d[0]));
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
