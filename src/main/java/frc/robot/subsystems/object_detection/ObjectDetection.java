package frc.robot.subsystems.object_detection;

import static frc.robot.subsystems.object_detection.ObjectDetectionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ObjectDetection extends SubsystemBase {
  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs;
  private final Alert disconnectedAlert;

  public ObjectDetection(ObjectDetectionIO io) {
    this.io = io;

    // Initialize inputs
    this.inputs = new ObjectDetectionIOInputsAutoLogged();

    // Initialize disconnected alerts
    this.disconnectedAlert =
        new Alert("Object detection camera is disconnected.", AlertType.kWarning);
  }

  /** Returns the X angle to the target, which can be used for simple servoing with vision. */
  public Rotation2d getTargetX() {
    return inputs.latestTargetObservation.tx();
  }

  /** Returns the relative area of the target. */
  public double getTargetArea() {
    return inputs.latestTargetObservation.ta();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Object Detection", inputs);
  }

  public void captureRewind(int time) {
    io.captureRewind(time);
  }

  public void throttleLimelights() {
    io.setThrottle(ObjectDetectionConstants.DISABLED_THROTTLE);
  }

  public void unthrottleLimelights() {
    io.setThrottle(ObjectDetectionConstants.ENABLED_THROTTLE);
  }
}
