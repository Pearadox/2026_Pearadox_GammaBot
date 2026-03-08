package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoringMode;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Supplier<ChassisSpeeds> speedsSupplier;
  private Supplier<Rotation2d> robotRotationSupplier;

  @AutoLogOutput private boolean hasZeroed = false;

  private double turretRotationAdjust = 0;

  public void adjustRotationBy(double adj) {
    turretRotationAdjust += adj;
  }

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", 6.7); // 4
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.0); // 0
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.0);
  private final LoggedTunableNumber kOmega = new LoggedTunableNumber("Turret/kOmega", 0.0); // 0.2
  private final LoggedTunableNumber mmCruiseVel = new LoggedTunableNumber("Turret/mmCruiseVel", 85);
  private final LoggedTunableNumber mmAcceleration = new LoggedTunableNumber("Turret/mmAcc", 450);
  private final LoggedTunableNumber testSetpoint =
      new LoggedTunableNumber("Turret/test Setpoint", -45);
  private final LoggedTunableNumber fieldRelOffset =
      new LoggedTunableNumber("Turret/fieldreloffset", 0);

  private boolean shouldApplyFF = true;

  private final SysIdRoutine sysId;

  public Turret(
      TurretIO io,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> robotRotationSupplier) {
    this.io = io;
    this.speedsSupplier = chassisSpeedsSupplier;
    this.robotRotationSupplier = robotRotationSupplier;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.05).div(Seconds.of(1)), // Ramp rate (V/s)
                Volts.of(0.15), // Step voltage
                Seconds.of(10), // Timeout
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));

    io.setPID(kP.get(), kI.get(), kD.get());
    io.setFFGains(kS.get(), kV.get(), kA.get());
    io.setMotionMagicLimits(mmCruiseVel.get(), mmAcceleration.get());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // if (!hasZeroed && inputs.cancoderConnected) {
    //   requestZero();
    // }

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }

    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      io.setFFGains(kS.get(), kV.get(), kA.get());
    }

    if (mmCruiseVel.hasChanged(hashCode()) || mmAcceleration.hasChanged(hashCode())) {
      io.setMotionMagicLimits(mmCruiseVel.get(), mmAcceleration.get());
    }

    ScoringMode currentScoringMode = RobotContainer.getScoringMode();

    if (currentScoringMode != ScoringMode.FULLY_MANUAL) {

      followFieldCentricTarget(
          () ->
              RobotContainer.getShotSolution()
                  .getTurretAngleRot2d()
                  .plus(new Rotation2d(turretRotationAdjust)));

    } else if (currentScoringMode == ScoringMode.FULLY_MANUAL) {

      // followFieldCentricTarget(
      //     () -> getFieldRelativeTurretAngleRotation2d().plus(new
      // Rotation2d(turretRotationAdjust)));

      Logger.recordOutput(
          "Turret/SOTM/shotSolutionDesiredRotation",
          RobotContainer.getShotSolution().getTurretAngleRot2d());
    }
  }

  /** Follows a robot-centric angle. */
  public void followRobotCentricTarget(Supplier<Rotation2d> robotCentricAngleSupplier) {
    double setpointTurretRads = wrap(robotCentricAngleSupplier.get().getRadians());
    double setpointMotorRots = setpointTurretRads / TurretConstants.TURRET_P_COEFFICIENT;

    double ffVolts = getFF(setpointTurretRads);

    if (shouldApplyFF) {
      io.runPosition(setpointMotorRots, ffVolts);
    }

    Logger.recordOutput(
        "Turret/Setpoint Turret Degrees", Units.radiansToDegrees(setpointTurretRads));
    Logger.recordOutput("Turret/Setpoint Motor Rots", setpointMotorRots);
    Logger.recordOutput("Turret/FF Volts", ffVolts);
  }

  public void followFieldCentricTarget(Supplier<Rotation2d> fieldCentricAngleSupplier) {
    followRobotCentricTarget(
        () ->
            robotRotationSupplier
                .get()
                .minus(fieldCentricAngleSupplier.get())
                .plus(Rotation2d.fromDegrees(fieldRelOffset.get())));
  }

  public void goToZero() {
    io.runPosition(0, 0);
  }

  public void goToPlus90() {
    io.runPosition(
        Units.degreesToRadians(testSetpoint.get()) / TurretConstants.TURRET_P_COEFFICIENT, 0);
  }

  public void goToPlus180() {
    io.runPosition(
        Units.degreesToRadians(testSetpoint.get()) / TurretConstants.TURRET_P_COEFFICIENT, 0);
  }

  public void goToMinus180() {
    io.runPosition(
        -Units.degreesToRadians(testSetpoint.get()) / TurretConstants.TURRET_P_COEFFICIENT, 0);
  }

  /** Zeroes the turret */
  public void requestZero() {
    if (inputs.cancoderConnected) {
      io.setPosition(
          (inputs.cancoderPosition * TurretConstants.TURRET_TO_CANCODER_RATIO)
                  * TurretConstants.TURRET_GEAR_RATIO
              + TurretConstants.TURRET_STARTING_ANGLE / TurretConstants.TURRET_P_COEFFICIENT);
      hasZeroed = true;
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  private void runVoltage(double volts) {
    io.runVoltage(volts);
  }

  @AutoLogOutput
  public double getTurretAngleRads() {
    return inputs.turretData.position() * TurretConstants.TURRET_P_COEFFICIENT;
  }

  @AutoLogOutput
  public double getTurretAngleDegs() {
    return Units.radiansToDegrees(getTurretAngleRads());
  }

  @AutoLogOutput
  public Rotation2d getRobotRelativeTurretAngleRotation2d() {
    return new Rotation2d(getTurretAngleRads());
  }

  @AutoLogOutput
  public Rotation2d getFieldRelativeTurretAngleRotation2d() {
    return getRobotRelativeTurretAngleRotation2d().plus(robotRotationSupplier.get());
  }

  private double wrap(double target) {
    target = Math.IEEEremainder(target, 2 * Math.PI);

    double current = getTurretAngleRads();
    double[] candidates = new double[] {target - 2 * Math.PI, target, target + 2 * Math.PI};

    double bestAngle = target;
    double bestDist = Double.POSITIVE_INFINITY;

    for (double c : candidates) {
      if (c > TurretConstants.TURRET_SAFE_MIN && c < TurretConstants.TURRET_SAFE_MAX) {
        double dist = Math.abs(current - c);
        if (dist < bestDist) {
          bestAngle = c;
          bestDist = dist;
        }
      }
    }

    return bestAngle;
  }

  private double getFF(double setpointRads) {
    double chassisAngularVelocity = speedsSupplier.get().omegaRadiansPerSecond;

    shouldApplyFF =
        Math.abs(
                    Rotation2d.fromRadians(setpointRads)
                        .minus(Rotation2d.fromRadians(getTurretAngleRads()))
                        .getRadians())
                < TurretConstants.FF_ERROR_THRESHOLD
            && Math.abs(chassisAngularVelocity) < TurretConstants.FF_CHASSIS_ROT_VELOCITY_LIMIT;

    return shouldApplyFF ? chassisAngularVelocity * kOmega.get() : 0;
  }
}
