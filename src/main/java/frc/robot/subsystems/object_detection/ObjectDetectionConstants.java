package frc.robot.subsystems.object_detection;

public class ObjectDetectionConstants {
  // Camera names, must match names configured on coprocessor
  public static String cameraName = "limelight-pearl";

  public static final int MIN_REWIND_LENGTH_SECONDS = 10;
  public static final int MAX_REWIND_LENGTH_SECONDS = 240;
  // skip 120 frames per second while disabled
  public static final int DISABLED_THROTTLE = 120;
  // do not skip frames while robot enabled
  public static final int ENABLED_THROTTLE = 0;
}
