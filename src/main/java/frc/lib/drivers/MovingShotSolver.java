package frc.lib.drivers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Hub;
import frc.robot.Constants.FieldConstants.LinesHorizontal;
import frc.robot.Constants.FieldConstants.LinesVertical;
import frc.robot.Robot;
import frc.robot.subsystems.launcher.LauncherConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class MovingShotSolver {
  private static MovingShotSolver INSTANCE;

  public static MovingShotSolver getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new MovingShotSolver();
    }
    return INSTANCE;
  }

  private MovingShotSolver() {}

  @Getter private static ShotSolution shotSolution = new ShotSolution(0.0, 0.0, Rotation2d.kZero);

  private enum Goal {
    HUB(Hub.topCenterPointRed, Hub.topCenterPointBlue),
    DEPOT_CORNER(
        new Translation3d(
            LinesVertical.redHubCenterX + 1, LinesHorizontal.leftBumpStart - 0.25, 2.0),
        new Translation3d(
            LinesVertical.blueHubCenterX - 1,
            FieldConstants.fieldWidth - (LinesHorizontal.leftBumpStart - 0.25),
            2.0)),
    OUTPOST_CORNER(
        new Translation3d(
            LinesVertical.redHubCenterX + 1, LinesHorizontal.rightBumpEnd + 0.25, 2.0),
        new Translation3d(
            LinesVertical.blueHubCenterX - 1,
            FieldConstants.fieldWidth - (LinesHorizontal.rightBumpEnd + 0.25),
            2.0));

    private final Translation3d redLocation;
    private final Translation3d blueLocation;

    private Goal(Translation3d red, Translation3d blue) {
      this.redLocation = red;
      this.blueLocation = blue;
    }

    public Translation3d getLocation(Alliance alliance) {
      return alliance == Alliance.Red ? redLocation : blueLocation;
    }

    public static Goal findTarget(Pose2d robotPose, Alliance alliance) {
      double robotX = robotPose.getX();
      double robotY = robotPose.getY();
      boolean isRedAlliance = alliance == Alliance.Red;

      boolean inAllianceZone =
          (!isRedAlliance && robotX < LinesVertical.allianceZone)
              || (isRedAlliance && robotX > LinesVertical.oppAllianceZone);

      if (inAllianceZone) {
        return HUB;
      }

      boolean isLowerHalf = robotY < FieldConstants.fieldWidth / 2.0;

      if (isRedAlliance) {
        return isLowerHalf ? OUTPOST_CORNER : DEPOT_CORNER;
      } else {
        return isLowerHalf ? DEPOT_CORNER : OUTPOST_CORNER;
      }
    }
  }

  private final LoggedTunableNumber rpsMultiplier =
      new LoggedTunableNumber("SOTM/Rps Multiplier", 2);
  private final LoggedTunableNumber turretDx = new LoggedTunableNumber("SOTM/Turret dx", -0.135);
  private final LoggedTunableNumber turretDy = new LoggedTunableNumber("SOTM/Turret dy", -0.14);
  private final LoggedTunableNumber shooterHeightInches =
      new LoggedTunableNumber("SOTM/Launch Height in", 22.5);
  private final LoggedTunableNumber hoodAngleDegrees =
      new LoggedTunableNumber("SOTM/Launch Angle Degs", 72);

  private static final double MPSToRPSConversion =
      LauncherConstants.LAUNCHER_GEARING / LauncherConstants.ROLLER_CIRCUMFERENCE_METERS;

  public record ShotSolution(double time, double speed, Rotation2d turretAngle) {}

  public ShotSolution solve(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> robotRelativeSpeedSupplier) {

    // hoodAngleRadians = Launcher.getState().getHoodAngleRads();

    Alliance alliance = Robot.getAlliance();

    ChassisSpeeds robotRelative = robotRelativeSpeedSupplier.get();
    Pose2d curPose = poseSupplier.get();

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, curPose.getRotation());

    Goal goal = Goal.findTarget(curPose, alliance);
    Translation3d goalTranslation = goal.getLocation(alliance);
    double goalXMeters = goalTranslation.getX();
    double goalYMeters = goalTranslation.getY();
    double goalHeightMeters = goalTranslation.getZ();

    // Robot-relative turret offset (meters)

    double dxTurretRobotRelative = turretDx.get(); // TODO: find real forward offset
    double dyTurretRobotRelative = turretDy.get(); // TODO: find real sideways offset

    // Rotate offset into field coordinates using matrix multiplication done below

    /*
        [dxTurretFieldRelative] = [cos(theta) -sin(theta)][dxTurretRelativeToRobot]
        [dyTurretFieldRelative]   [sin(theta)  cos(theta)][dyTurretRelativeToRobot]
    */

    double thetaRobot = curPose.getRotation().getRadians();
    double dxTurretFieldRelative =
        dxTurretRobotRelative * Math.cos(thetaRobot) - dyTurretRobotRelative * Math.sin(thetaRobot);
    double dyTurretFieldRelative =
        dxTurretRobotRelative * Math.sin(thetaRobot) + dyTurretRobotRelative * Math.cos(thetaRobot);

    // Actual turret position in field coordinates

    double turretXMeters = curPose.getX() + dxTurretFieldRelative;
    double turretYMeters = curPose.getY() + dyTurretFieldRelative;

    // Derive NM calculations from turret displacement instead of robot center

    double Dx = goalXMeters - turretXMeters;
    double Dy = goalYMeters - turretYMeters;
    double Dz = goalHeightMeters - Units.inchesToMeters(shooterHeightInches.get());

    double hoodAngleRadians = Math.toRadians(hoodAngleDegrees.get());

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;

    Logger.recordOutput("SOTM/distnace to target", Math.hypot(Dx, Dy));

    double ToF =
        1.0 + Math.hypot(Dx, Dy) / 15.0 * (3.0 - 1.0); // Initial guess of ToF for Newton's Method
    // (formula: ToF = t_min + dist/maxDist * (t_max - t_min))

    for (int i = 0; i < Constants.NEWTONS_METHOD_NUM_STEPS; i++) {
      // recalculating closer approximate value of ToF after each "step"

      double vxLaunch = (Dx / ToF) - robotVx;
      double vyLaunch = (Dy / ToF) - robotVy;

      double horizontalSpeed = Math.hypot(vxLaunch, vyLaunch);

      double vzLaunch = horizontalSpeed * Math.tan(hoodAngleRadians);

      double f = (vzLaunch * ToF - 0.5 * Constants.g * ToF * ToF) - Dz; // h = vt - 1/2at^2

      // Numerical derivative for Newton's method

      double dt = 1e-4; // "small" dt to estimate derivative
      double t2 = ToF + dt;

      double vx2 = (Dx / t2) - robotVx;
      double vy2 = (Dy / t2) - robotVy;
      double h2 = Math.hypot(vx2, vy2);
      double vz2 = h2 * Math.tan(hoodAngleRadians);

      double f2 = (vz2 * t2 - 0.5 * Constants.g * t2 * t2) - Dz;

      double fPrime = (f2 - f) / dt;

      if (Math.abs(fPrime) > 1e-6) {
        ToF -= f / fPrime;
      }

      // set min bound for t
      if (ToF < 0.1) ToF = 0.1;
    }

    // Compute final launch velocity (m/s) components

    double vxLaunchMPS = (Dx / ToF) - robotVx;
    double vyLaunchMPS = (Dy / ToF) - robotVy;

    double totalHorizontalSpeedMPS = Math.hypot(vxLaunchMPS, vyLaunchMPS);
    double vzLaunchMPS = totalHorizontalSpeedMPS * Math.tan(hoodAngleRadians);

    // Shooter wheel speed (m/s) magnitude:

    double shooterSpeedMPS = Math.hypot(totalHorizontalSpeedMPS, vzLaunchMPS);

    // Shooter wheel speed (rot / s) magnitude:

    double shooterSpeedRPS = shooterSpeedMPS * MPSToRPSConversion;

    double targetXOffsetMeters = goalXMeters - robotVx * ToF;
    double targetYOffsetMeters = goalYMeters - robotVy * ToF;
    Pose2d targetPose = new Pose2d(targetXOffsetMeters, targetYOffsetMeters, new Rotation2d());

    double outputtedShooterVelocity = Math.min(100, rpsMultiplier.get() * shooterSpeedRPS);

    // Compute field-relative turret angle

    Rotation2d fieldRelativeTurretAngleRot2d = new Rotation2d(Math.atan2(vyLaunchMPS, vxLaunchMPS));

    Logger.recordOutput(
        "SOTM/fieldRelativeTurretAngle", fieldRelativeTurretAngleRot2d.getDegrees());
    Logger.recordOutput("SOTM/timeOfFlight", ToF);
    Logger.recordOutput("SOTM/desiredShooterSpeed_RPS", shooterSpeedRPS * rpsMultiplier.get());
    Logger.recordOutput("SOTM/cappedShooterSpeed_RPS", outputtedShooterVelocity);
    Logger.recordOutput("SOTM/currentRotation", curPose.getRotation().getDegrees());
    Logger.recordOutput("SOTM/targetPose", targetPose);
    Logger.recordOutput("SOTM/Goal", goal.toString());

    return shotSolution =
        new ShotSolution(ToF, outputtedShooterVelocity, fieldRelativeTurretAngleRot2d);
  }
}
