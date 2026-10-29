package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, 0.0);
  }

  /** Represents the angle to and area of target */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty, double ta) {}

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}

  public default void captureRewind(int time) {}

  public default void setThrottle(int throttle) {}
}
