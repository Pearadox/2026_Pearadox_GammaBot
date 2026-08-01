package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

public class TrashCanLayout {

  public static AprilTagFieldLayout createLayout() {
    // 1. Define physical target measurements (meters)
    double radius = 0.37465; // Distance from center axis out to a tag face
    double height = 0.19685; // 0.27305 // Vertical distance from floor to tag center

    // 2. Build the AprilTag object list matching your CCW layout configuration
    List<AprilTag> tags =
        List.of(
            // ID 1: Front Face (0 degrees Yaw Reference now)
            new AprilTag(
                1,
                new Pose3d(
                    new Translation3d(radius, 0.0, height),
                    new Rotation3d(0.0, 0.0, Math.toRadians(0.0)))),

            // ID 12: Left Face (Rotated 90 degrees CCW Yaw)
            new AprilTag(
                12,
                new Pose3d(
                    new Translation3d(0.0, radius, height),
                    new Rotation3d(0.0, 0.0, Math.toRadians(90.0)))),

            // ID 17: Back Face (Rotated 180 degrees Yaw now)
            new AprilTag(
                17,
                new Pose3d(
                    new Translation3d(-radius, 0.0, height),
                    new Rotation3d(0.0, 0.0, Math.toRadians(180.0)))),

            // ID 28: Right Face (Rotated -90 degrees / 270 degrees CCW Yaw)
            new AprilTag(
                28,
                new Pose3d(
                    new Translation3d(0.0, -radius, height),
                    new Rotation3d(0.0, 0.0, Math.toRadians(-90.0)))));

    // 3. Construct the layout object bounds (arbitrary boundary box)
    return new AprilTagFieldLayout(tags, 5.0, 5.0);
  }
}
