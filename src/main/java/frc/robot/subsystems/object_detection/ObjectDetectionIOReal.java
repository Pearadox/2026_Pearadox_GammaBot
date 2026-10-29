package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

/** IO implementation for real Limelight hardware. */
public class ObjectDetectionIOReal implements ObjectDetectionIO {
  private final DoubleArrayPublisher rewindPublisher;
  private final IntegerPublisher throttlePublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber taSubscriber;

  private int numRewinds = 0;

  private final double[] rewindArray = new double[2];

  /**
   * Creates a new ObjectDetectionIOReal.
   *
   * @param name The configured name of the Limelight.
   */
  public ObjectDetectionIOReal(String name) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);

    rewindPublisher = table.getDoubleArrayTopic("capture_rewind").publish();
    throttlePublisher = table.getIntegerTopic("throttle_set").publish();
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()),
            Rotation2d.fromDegrees(tySubscriber.get()),
            taSubscriber.get());
  }

  @Override
  public void captureRewind(int time) {
    rewindArray[0] = ++numRewinds;
    rewindArray[1] = time;

    rewindPublisher.set(rewindArray);
  }

  @Override
  public void setThrottle(int throttle) {
    throttlePublisher.set(throttle);
  }
}
