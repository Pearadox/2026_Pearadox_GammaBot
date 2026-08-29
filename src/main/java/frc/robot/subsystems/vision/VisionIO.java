// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void captureRewind(int time) {}

  public default void setThrottle(int throttle) {}

  /**
   * The camera's configured mount pose in robot space, as [x, y, z, roll, pitch, yaw] with meters
   * and degrees. Empty if this IO has no such concept.
   */
  public default double[] getCameraPoseRobotSpace() {
    return new double[0];
  }

  /**
   * Overrides the camera's configured mount pose. On a Limelight this lasts until the camera
   * reboots, which makes it safe for trying a calibration result out before committing it in the
   * web UI.
   */
  public default void setCameraPoseRobotSpace(double[] cameraPose) {}
}
