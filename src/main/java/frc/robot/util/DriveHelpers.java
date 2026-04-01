package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;

public class DriveHelpers {

  /**
   * rotates the robot 45 degrees given current pose
   *
   * @param robotPoseSupplier
   * @return new Rotation2d
   */
  public static Rotation2d findClosestCorner(Supplier<Pose2d> robotPoseSupplier) {
    double currentDegrees = robotPoseSupplier.get().getRotation().getDegrees();
    double[] robotCornersInDegrees = {45, 45 + 90, 45 + 180, 45 + 270};
    double minError = Double.MAX_VALUE;
    double closest = robotCornersInDegrees[0];

    for (double c : robotCornersInDegrees) {

      double error = Math.abs(Math.IEEEremainder(c - currentDegrees, 360));
      if (error < minError) {
        minError = error;
        closest = c;
      }
    }
    return new Rotation2d(Units.degreesToRadians(closest));
  }

  public static Rotation2d getCourseRotation2d(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;

    Rotation2d course = new Rotation2d(vx, vy);
    if (Math.hypot(vx, vy) < 0.05) {
      return new Rotation2d();
    }
    return course;
  }

}
