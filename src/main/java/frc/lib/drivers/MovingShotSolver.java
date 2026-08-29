package frc.lib.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Hub;
import frc.robot.Constants.FieldConstants.LinesHorizontal;
import frc.robot.Constants.FieldConstants.LinesVertical;
import frc.robot.Robot;
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

  private MovingShotSolver() {
    createShotMapTunables();
  }

  @Getter
  private static ShotSolution shotSolution = new ShotSolution(0.0, 0.0, Rotation2d.kZero, 0.0);

  // Shot map breakpoints, tunable from the dashboard so practice-field tuning does not need a
  // redeploy. Defaults are exactly what used to be hardcoded here, with the old hand-added +3/+4
  // fudge terms folded into the numbers.
  //
  // Hood angles are stored in the map as (90 - hoodDeg) radians, which is what the trajectory math
  // wants, but they are exposed here in hood degrees: the same units as HOOD_MIN/MAX_ANGLE_RADS
  // and what the mechanism actually reads.
  private static final int SHOT_MAP_POINTS = 5;
  private static final double[] DEFAULT_ANGLE_DISTANCES = {1.639, 2.640, 3.638, 4.676, 11.0};
  private static final double[] DEFAULT_HOOD_DEGREES = {11.0, 17.0, 23.0, 27.0, 35.0};

  // The RPS breakpoints sit at slightly different distances than the angle breakpoints: the two
  // were measured in separate sessions at the same shooting positions. They are kept as measured
  // rather than silently averaged. ShotMap/PairKeyMismatch logs the gap, so converging them stays
  // a deliberate tuning decision made with the robot present.
  private static final double[] DEFAULT_RPS_DISTANCES = {1.678, 2.682, 3.623, 4.805, 11.0};
  private static final double[] DEFAULT_RPS_VALUES = {37.906, 41.719, 41.622, 49.0, 75.0};

  private final LoggedTunableNumber[] angleDistances = new LoggedTunableNumber[SHOT_MAP_POINTS];
  private final LoggedTunableNumber[] hoodDegrees = new LoggedTunableNumber[SHOT_MAP_POINTS];
  private final LoggedTunableNumber[] rpsDistances = new LoggedTunableNumber[SHOT_MAP_POINTS];
  private final LoggedTunableNumber[] rpsValues = new LoggedTunableNumber[SHOT_MAP_POINTS];
  private final LoggedTunableNumber[] allShotMapTunables =
      new LoggedTunableNumber[SHOT_MAP_POINTS * 4];

  private final InterpolatingDoubleTreeMap launchAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap launchRPSMap = new InterpolatingDoubleTreeMap();

  private void createShotMapTunables() {
    for (int i = 0; i < SHOT_MAP_POINTS; i++) {
      int point = i + 1;
      angleDistances[i] =
          new LoggedTunableNumber("ShotMap/Angle Dist " + point, DEFAULT_ANGLE_DISTANCES[i]);
      hoodDegrees[i] =
          new LoggedTunableNumber("ShotMap/Hood Deg " + point, DEFAULT_HOOD_DEGREES[i]);
      rpsDistances[i] =
          new LoggedTunableNumber("ShotMap/RPS Dist " + point, DEFAULT_RPS_DISTANCES[i]);
      rpsValues[i] = new LoggedTunableNumber("ShotMap/RPS " + point, DEFAULT_RPS_VALUES[i]);

      allShotMapTunables[i * 4] = angleDistances[i];
      allShotMapTunables[i * 4 + 1] = hoodDegrees[i];
      allShotMapTunables[i * 4 + 2] = rpsDistances[i];
      allShotMapTunables[i * 4 + 3] = rpsValues[i];
    }
    rebuildShotMaps();
  }

  private void rebuildShotMaps() {
    launchAngleMap.clear();
    launchRPSMap.clear();

    double worstKeyMismatch = 0.0;
    for (int i = 0; i < SHOT_MAP_POINTS; i++) {
      launchAngleMap.put(
          angleDistances[i].get(), Units.degreesToRadians(90.0 - hoodDegrees[i].get()));
      launchRPSMap.put(rpsDistances[i].get(), rpsValues[i].get());

      worstKeyMismatch =
          Math.max(worstKeyMismatch, Math.abs(angleDistances[i].get() - rpsDistances[i].get()));
    }

    Logger.recordOutput("ShotMap/PairKeyMismatch", worstKeyMismatch);
  }

  private static LoggedTunableNumber woahMultiplierAgain =
      new LoggedTunableNumber("SOTM/everywhere multiplier", 1.0);

  public enum Goal {
    HUB(Hub.topCenterPointRed, Hub.topCenterPointBlue),
    DEPOT_CORNER(
        new Translation3d(
            LinesVertical.redHubCenterX + 1.75, LinesHorizontal.leftBumpStart - 0.25, 2.0),
        new Translation3d(
            LinesVertical.blueHubCenterX - 1.75,
            FieldConstants.fieldWidth - (LinesHorizontal.leftBumpStart - 0.25),
            2.0)),
    OUTPOST_CORNER(
        new Translation3d(
            LinesVertical.redHubCenterX + 1.75, LinesHorizontal.rightBumpEnd + 0.25, 2.0),
        new Translation3d(
            LinesVertical.blueHubCenterX - 1.75,
            FieldConstants.fieldWidth - (LinesHorizontal.rightBumpEnd + 0.25),
            2.0));

    private final Translation3d redLocation;
    private final Translation3d blueLocation;

    private static Debouncer fieldZoneDebouncer = new Debouncer(0.5, DebounceType.kBoth);

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
          fieldZoneDebouncer.calculate(
              (!isRedAlliance && robotX < LinesVertical.allianceZone + 1)
                  || (isRedAlliance && robotX > LinesVertical.oppAllianceZone - 1));

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

  private final LoggedTunableNumber turretDx = new LoggedTunableNumber("SOTM/Turret dx", -0.146);
  private final LoggedTunableNumber turretDy = new LoggedTunableNumber("SOTM/Turret dy", -0.133);
  private final LoggedTunableNumber shooterHeightInches =
      new LoggedTunableNumber("SOTM/Launch Height inches", 16);

  // may need to be tuned
  private final LoggedTunableNumber shotLatency = new LoggedTunableNumber("SOTM/shot latency", 0.1);
  private final double robotLoopTimeSeconds = 0.02;
  private ChassisSpeeds prevFieldRelativeSpeeds = new ChassisSpeeds();
  private double prevAccelerationX = 0;
  private double prevAccelerationY = 0;

  @Getter private Goal goal;

  public record ShotSolution(
      double time, double speed, Rotation2d turretAngle, double hoodAngleRadians) {}

  public ShotSolution solve(
      Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> robotRelativeSpeedSupplier) {

    LoggedTunableNumber.ifChanged(hashCode(), this::rebuildShotMaps, allShotMapTunables);

    Alliance alliance = Robot.getAlliance();

    ChassisSpeeds robotRelative = robotRelativeSpeedSupplier.get();
    Pose2d curPose = poseSupplier.get();

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, curPose.getRotation());

    // translational acceleration handling for SOTM below
    // (currently doesn't support rotational acceleration handling)

    double latency = shotLatency.get();

    double robotVx = fieldRelativeSpeeds.vxMetersPerSecond;
    double robotVy = fieldRelativeSpeeds.vyMetersPerSecond;
    double robotOmega = fieldRelativeSpeeds.omegaRadiansPerSecond;

    double robotAccelerationX =
        MathUtil.clamp(
            (robotVx - prevFieldRelativeSpeeds.vxMetersPerSecond) / robotLoopTimeSeconds, -2, 2);
    double robotAccelerationY =
        MathUtil.clamp(
            (robotVy - prevFieldRelativeSpeeds.vyMetersPerSecond) / robotLoopTimeSeconds, -2, 2);

    robotAccelerationX = 0.6 * prevAccelerationX + 0.4 * robotAccelerationX;
    robotAccelerationY = 0.6 * prevAccelerationY + 0.4 * robotAccelerationY;

    prevAccelerationX = robotAccelerationX;
    prevAccelerationY = robotAccelerationY;

    Logger.recordOutput("SOTM/AccelerationX", robotAccelerationX);
    Logger.recordOutput("SOTM/AccelerationY", robotAccelerationY);

    double predictedRobotX =
        curPose.getX() + robotVx * latency + 0.5 * robotAccelerationX * latency * latency;

    double predictedRobotY =
        curPose.getY() + robotVy * latency + 0.5 * robotAccelerationY * latency * latency;

    prevFieldRelativeSpeeds = new ChassisSpeeds(robotVx, robotVy, robotOmega);

    double predictedRobotVx = robotVx + robotAccelerationX * latency;
    double predictedRobotVy = robotVy + robotAccelerationY * latency;

    Pose2d predictedRobotPose = new Pose2d(predictedRobotX, predictedRobotY, curPose.getRotation());
    // not finding predictedRotation above yet

    goal = Goal.findTarget(predictedRobotPose, alliance);

    // end of acceleration handling

    Translation3d goalTranslation = goal.getLocation(alliance);
    double goalXMeters = goalTranslation.getX();
    double goalYMeters = goalTranslation.getY();
    double goalHeightMeters = goalTranslation.getZ();

    // Robot-relative turret offset (meters)

    double dxTurretRobotRelative = turretDx.get();
    double dyTurretRobotRelative =
        turretDy.get(); // TODO: make sure the LoggedTunableNumber values are correct

    // Rotate offset into field coordinates using matrix multiplication done below

    /*
        [dxTurretFieldRelative] = [cos(theta) -sin(theta)][dxTurretRelativeToRobot]
        [dyTurretFieldRelative]   [sin(theta)  cos(theta)][dyTurretRelativeToRobot]
    */

    double thetaRobot = predictedRobotPose.getRotation().getRadians();
    double dxTurretFieldRelative =
        dxTurretRobotRelative * Math.cos(thetaRobot) - dyTurretRobotRelative * Math.sin(thetaRobot);
    double dyTurretFieldRelative =
        dxTurretRobotRelative * Math.sin(thetaRobot) + dyTurretRobotRelative * Math.cos(thetaRobot);

    // Actual turret position in field coordinates

    double turretXMeters = predictedRobotX + dxTurretFieldRelative;
    double turretYMeters = predictedRobotY + dyTurretFieldRelative;

    // Derive NM calculations from turret displacement instead of robot center

    double Dx = goalXMeters - turretXMeters;
    double Dy = goalYMeters - turretYMeters;
    double Dz = goalHeightMeters - Units.inchesToMeters(shooterHeightInches.get());

    double distanceToTarget = Math.hypot(Dx, Dy);

    // The angle map and the RPS map were tuned together as (angle, speed) pairs per distance, so
    // both must be keyed off the SAME distance or the pairs come apart whenever the robot moves.
    // The virtual (motion-compensated) target is the physically meaningful one, but it depends on
    // the time of flight, so seed it with the real distance and refine it inside the loop below.
    double distanceToVirtualTarget = distanceToTarget;
    double hoodAngleRadians = launchAngleMap.get(distanceToVirtualTarget);

    double tanHoodAngle = Math.tan(hoodAngleRadians);

    double ToF =
        1.0 + distanceToTarget / 15.0 * (3.0 - 1.0); // Initial guess of ToF for Newton's Method
    // (formula: ToF = t_min + dist/maxDist * (t_max - t_min))

    Logger.recordOutput("SOTM/distance to target (lateral)", distanceToTarget);

    for (int i = 0; i < Constants.NEWTONS_METHOD_NUM_STEPS; i++) {
      // recalculating closer approximate value of ToF after each "step"

      // Re-key the hood angle off the virtual target for the current ToF estimate, so the angle
      // the ToF solve uses is the same angle the shot will actually be taken at.
      distanceToVirtualTarget =
          Math.hypot(
              (goalXMeters - predictedRobotVx * ToF) - turretXMeters,
              (goalYMeters - predictedRobotVy * ToF) - turretYMeters);
      hoodAngleRadians = launchAngleMap.get(distanceToVirtualTarget);
      tanHoodAngle = Math.tan(hoodAngleRadians);

      double vxLaunch = (Dx / ToF) - predictedRobotVx;
      double vyLaunch = (Dy / ToF) - predictedRobotVy;

      double horizontalSpeed = Math.hypot(vxLaunch, vyLaunch);

      double vzLaunch = horizontalSpeed * tanHoodAngle;

      double f = (vzLaunch * ToF - 0.5 * Constants.g * ToF * ToF) - Dz; // h = vt - 1/2at^2

      // Numerical derivative for Newton's method

      double dt = 1e-4; // "small" dt to estimate derivative
      double t2 = ToF + dt;

      double vx2 = (Dx / t2) - predictedRobotVx;
      double vy2 = (Dy / t2) - predictedRobotVy;
      double h2 = Math.hypot(vx2, vy2);
      double vz2 = h2 * tanHoodAngle;

      double f2 = (vz2 * t2 - 0.5 * Constants.g * t2 * t2) - Dz;

      double fPrime = (f2 - f) / dt;

      if (Math.abs(fPrime) > 1e-6) {
        ToF -= f / fPrime;
      }

      // set min bound for t
      if (ToF < 0.1) ToF = 0.1;
    }

    double targetXOffsetMeters = goalXMeters - predictedRobotVx * ToF;
    double targetYOffsetMeters = goalYMeters - predictedRobotVy * ToF;
    Pose2d targetPose = new Pose2d(targetXOffsetMeters, targetYOffsetMeters, new Rotation2d());

    double distanceX = targetXOffsetMeters - turretXMeters;
    double distanceY = targetYOffsetMeters - turretYMeters;
    distanceToVirtualTarget = Math.hypot(distanceX, distanceY);

    // Final ToF moved the virtual target one last time, so re-look-up the angle here. Both maps
    // are now read at exactly the same key.
    hoodAngleRadians = launchAngleMap.get(distanceToVirtualTarget);

    double shooterSpeedRPS = launchRPSMap.get(distanceToVirtualTarget) * woahMultiplierAgain.get();

    // Compute field-relative turret angle

    Rotation2d fieldRelativeTurretAngleRot2d = new Rotation2d(Math.atan2(distanceY, distanceX));

    Logger.recordOutput(
        "SOTM/fieldRelativeTurretAngle", fieldRelativeTurretAngleRot2d.getDegrees());
    Logger.recordOutput("SOTM/timeOfFlight", ToF);
    Logger.recordOutput("SOTM/desiredShooterSpeed_RPS", shooterSpeedRPS);
    Logger.recordOutput("SOTM/currentRotation", curPose.getRotation().getDegrees());
    Logger.recordOutput("SOTM/targetPose", targetPose);
    Logger.recordOutput("SOTM/distancetoVirtualTarget", distanceToVirtualTarget);
    Logger.recordOutput("SOTM/Goal", goal.toString());
    Logger.recordOutput("SOTM/desiredHoodAngle", Units.radiansToDegrees(hoodAngleRadians));

    // Tuning capture set: these three channels, read at the moment a shot goes out, are what a
    // made or missed shot turns into a new ShotMap breakpoint. Scrub them in AdvantageScope
    // against SOTM/readyToShoot to find the shots that actually left the robot.
    Logger.recordOutput("ShotMap/Capture/Distance", distanceToVirtualTarget);
    Logger.recordOutput("ShotMap/Capture/HoodDeg", 90.0 - Units.radiansToDegrees(hoodAngleRadians));
    Logger.recordOutput("ShotMap/Capture/RPS", shooterSpeedRPS);

    return shotSolution =
        new ShotSolution(
            ToF, shooterSpeedRPS, fieldRelativeTurretAngleRot2d, Math.PI / 2 - hoodAngleRadians);
  }
}
