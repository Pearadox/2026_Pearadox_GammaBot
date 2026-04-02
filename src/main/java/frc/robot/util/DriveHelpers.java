package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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

  public static Rotation2d getCourseRotation2d(
      Supplier<ChassisSpeeds> chassisSpeedsSupplier, Supplier<Rotation2d> rotationSupplier) {
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsSupplier.get(), rotationSupplier.get());
    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;

    if (Math.hypot(vx, vy) < 0.05) {
      return new Rotation2d();
    }

    Logger.recordOutput("Snake/vy", vy);
    Logger.recordOutput("Snake/vx", vx);
    double angleRad = (Math.atan2(vy, vx));

    Logger.recordOutput("Snake/snake angle", Units.radiansToDegrees(angleRad));
    return new Rotation2d(angleRad);
  }
}
