// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double LOOP_PERIOD = 0.02; // 20ms
  public static final double LOOP_FREQUENCY = 1.0 / LOOP_PERIOD; // 50Hz
  public static final double NOMINAL_VOLTAGE = 12; // V
  public static final double g = 9.79267; // m/s^2 in Pearland

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double UPDATE_FREQ_SEC = 0.02;

  public static final int NEWTONS_METHOD_NUM_STEPS = 15;

  public static final double BROWNOUT_VOLTAGE = 6.0; // TODO: TUNE

  public static final class AlignConstants {
    public static final double ALIGN_STRAFE_KP = 0.02;
    public static final double ALIGN_STRAFE_KI = 0.001;
    public static final double ALIGN_FORWARD_KP = 0.06; // -0.06
    public static final double ALIGN_KS = 0.009;

    // rotational PID
    public static final double ROT_kP = 24.0;
    public static final double ROT_kI = 0.0;
    public static final double ROT_kD = 0.0;
    public static final double MAX_ROT_VELOCITY = Math.PI * 2; // rad/s
    public static final double MAX_ROT_ACCELERATION = Math.PI * 4; // rad/s^2
  }

  public static final class VisualizerConstants {
    public static final Translation3d MODEL0_ZERO = new Translation3d(-0.134550, -0.143323, 0);
    public static final Translation3d Z1_ZERO = new Translation3d(-0.1235075, -0.041317, 0.519888);
    public static final Translation3d MODEL2_ZERO = new Translation3d(0.024588, 0, 0);
    public static final Translation3d MODEL3_ZERO = new Translation3d(0.205374, 0, 0.260350);
    public static final Translation3d Z4_ZERO = new Translation3d(0.302910, 0, 0.646415);

    public static final Translation3d MODEL1_OFFSET = Z1_ZERO.minus(MODEL0_ZERO);
    public static final Translation3d MODEL4_OFFSET = Z4_ZERO.minus(MODEL3_ZERO);

    public static final double TURRET_STARTING_ANGLE = -Math.PI / 2; // this is only used in sim

    public static final double HOOD_STARTING_ANGLE = Units.degreesToRadians(61.549451);
    public static final double HOOD_MIN_ANGLE = Units.degreesToRadians(24.652849);
    public static final double HOOD_MAX_ANGLE = Units.degreesToRadians(69.652849);

    public static final double INTAKE_STARTING_ANGLE = Math.PI / 2; // radians
    public static final double GRAVITY_RAMP_MAX_OFFSET_DEGS = 39; // degrees

    public static final double CLIMBER_MAX_DISPLACEMENT = Units.inchesToMeters(5.875);
  }

  public static class FieldConstants {
    // AprilTag related constants
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final int aprilTagCount = aprilTagLayout.getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);

    // Field dimensions
    public static final double fieldLength = aprilTagLayout.getFieldLength();
    public static final double fieldWidth = aprilTagLayout.getFieldWidth();

    public static class LinesVertical {
      public static final double center = fieldLength / 2.0;
      public static final double starting = aprilTagLayout.getTagPose(26).get().getX();
      public static final double allianceZone = starting;
      public static final double blueHubCenterX =
          aprilTagLayout.getTagPose(26).get().getX() + Hub.width / 2.0;
      public static final double neutralZoneNear = center - Units.inchesToMeters(120);
      public static final double neutralZoneFar = center + Units.inchesToMeters(120);
      public static final double redHubCenterX =
          aprilTagLayout.getTagPose(4).get().getX() + Hub.width / 2.0;
      public static final double oppAllianceZone = aprilTagLayout.getTagPose(10).get().getX();
    }
    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis
     * offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

      public static final double center = fieldWidth / 2.0;

      // Right of hub
      public static final double rightBumpStart = Hub.nearRightCorner.getY();
      public static final double rightBumpEnd = rightBumpStart - RightBump.width;
      public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
      public static final double rightTrenchOpenEnd = 0;

      // Left of hub
      public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
      public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
      public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
      public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {
      // Dimensions
      public static final double width = Units.inchesToMeters(47.0);
      public static final double height =
          Units.inchesToMeters(72.0); // includes the catcher at the top
      public static final double innerWidth = Units.inchesToMeters(41.7);
      public static final double innerHeight = Units.inchesToMeters(56.5);

      // Relevant reference points on alliance side
      public static final Translation3d topCenterPointBlue =
          new Translation3d(
              aprilTagLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation3d innerCenterPointBlue =
          new Translation3d(
              aprilTagLayout.getTagPose(26).get().getX() + width / 2.0,
              fieldWidth / 2.0,
              innerHeight);

      public static final Translation2d nearLeftCorner =
          new Translation2d(
              topCenterPointBlue.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d nearRightCorner =
          new Translation2d(
              topCenterPointBlue.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d farLeftCorner =
          new Translation2d(
              topCenterPointBlue.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d farRightCorner =
          new Translation2d(
              topCenterPointBlue.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Relevant reference points on the opposite side
      public static final Translation3d topCenterPointRed =
          new Translation3d(
              aprilTagLayout.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(topCenterPointRed.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppNearRightCorner =
          new Translation2d(topCenterPointRed.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(topCenterPointRed.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppFarRightCorner =
          new Translation2d(topCenterPointRed.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Hub faces
      public static final Pose2d nearFace = aprilTagLayout.getTagPose(26).get().toPose2d();
      public static final Pose2d farFace = aprilTagLayout.getTagPose(20).get().toPose2d();
      public static final Pose2d rightFace = aprilTagLayout.getTagPose(18).get().toPose2d();
      public static final Pose2d leftFace = aprilTagLayout.getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX - width / 2, Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX + width / 2, Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX - width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX + width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX + width / 2, Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX - width / 2, Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX + width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(LinesVertical.blueHubCenterX - width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft =
          new Translation3d(LinesVertical.blueHubCenterX, fieldWidth, openingHeight);
      public static final Translation3d openingTopRight =
          new Translation3d(LinesVertical.blueHubCenterX, fieldWidth - openingWidth, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft =
          new Translation3d(LinesVertical.redHubCenterX, fieldWidth, openingHeight);
      public static final Translation3d oppOpeningTopRight =
          new Translation3d(LinesVertical.redHubCenterX, fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft =
          new Translation3d(LinesVertical.blueHubCenterX, openingWidth, openingHeight);
      public static final Translation3d openingTopRight =
          new Translation3d(LinesVertical.blueHubCenterX, 0, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft =
          new Translation3d(LinesVertical.redHubCenterX, openingWidth, openingHeight);
      public static final Translation3d oppOpeningTopRight =
          new Translation3d(LinesVertical.redHubCenterX, 0, openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
      // Dimensions
      public static final double width = Units.inchesToMeters(49.25);
      public static final double depth = Units.inchesToMeters(45.0);
      public static final double height = Units.inchesToMeters(78.25);
      public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
      public static final double frontFaceX = Units.inchesToMeters(43.51);

      public static final double uprightHeight = Units.inchesToMeters(72.1);

      // Rung heights from the floor
      public static final double lowRungHeight = Units.inchesToMeters(27.0);
      public static final double midRungHeight = Units.inchesToMeters(45.0);
      public static final double highRungHeight = Units.inchesToMeters(63.0);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint =
          new Translation2d(frontFaceX, aprilTagLayout.getTagPose(31).get().getY());
      public static final Translation2d leftUpright =
          new Translation2d(
              frontFaceX,
              (aprilTagLayout.getTagPose(31).get().getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d rightUpright =
          new Translation2d(
              frontFaceX,
              (aprilTagLayout.getTagPose(31).get().getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));

      // Relevant reference points on opposing side
      public static final Translation2d oppCenterPoint =
          new Translation2d(fieldLength - frontFaceX, aprilTagLayout.getTagPose(15).get().getY());
      public static final Translation2d oppLeftUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (aprilTagLayout.getTagPose(15).get().getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d oppRightUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (aprilTagLayout.getTagPose(15).get().getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));
    }

    public static class Depot {
      // Dimensions
      public static final double width = Units.inchesToMeters(42.0);
      public static final double depth = Units.inchesToMeters(27.0);
      public static final double height = Units.inchesToMeters(1.125);
      public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

      // Relevant reference points on alliance side
      public static final Translation3d depotCenter =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
      public static final Translation3d leftCorner =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
      public static final Translation3d rightCorner =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
      // Dimensions
      public static final double width = Units.inchesToMeters(31.8);
      public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
      public static final double height = Units.inchesToMeters(7.0);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint =
          new Translation2d(0, aprilTagLayout.getTagPose(29).get().getY());
    }
  }

  public static final class LEDConstants {
    public static final int LED_PORT = 9; // TODO : change later
    public static final int NUM_LEDS = 49 + 47;

    public static final Frequency SCROLL_FREQ = Percent.per(Second).of(50);
    public static final Time BLINK_PERIOD = Seconds.of(0.5);
    public static final Time BLINKING_DURATION = BLINK_PERIOD.times(2);
    public static final Dimensionless BLINK_BRIGHTNESS = Percent.of(50);
    public static final Time BREATHE_PERIOD = Seconds.of(1);
  }
}
