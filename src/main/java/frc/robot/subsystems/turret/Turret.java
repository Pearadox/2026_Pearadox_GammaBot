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
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private Supplier<ChassisSpeeds> speedsSupplier;
  private Supplier<Rotation2d> robotRotationSupplier;
  private BooleanSupplier turretHasClearanceSupplier;

  @AutoLogOutput @Getter private boolean hasZeroed = false;

  @AutoLogOutput private double turretRotationAdjust = 0; // -0.13

  public void adjustRotationBy(double adj) {
    turretRotationAdjust -= adj;
  }

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Turret/kP", Constants.currentMode == Mode.SIM ? 3.0 : 6.7, false);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", 0.0, false);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", 0.0, false);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", 0.0, false);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", 0.0, false);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA", 0.0, false);
  private final LoggedTunableNumber kOmega = new LoggedTunableNumber("Turret/kOmega", 0.0, false);
  private final LoggedTunableNumber mmCruiseVel =
      new LoggedTunableNumber("Turret/mmCruiseVel", 85, false); // 85
  private final LoggedTunableNumber mmAcceleration =
      new LoggedTunableNumber("Turret/mmAcc", 450, false);
  private final LoggedTunableNumber testSetpoint =
      new LoggedTunableNumber("Turret/test Setpoint", -90);
  private final LoggedTunableNumber fieldRelOffset =
      new LoggedTunableNumber("Turret/fieldreloffset", -90); // turret now zeros facing right
  @AutoLogOutput private boolean shouldApplyFF = true;

  private final SysIdRoutine sysId;

  public Turret(
      TurretIO io,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> robotRotationSupplier,
      BooleanSupplier turretHasClearanceSupplier) {
    this.io = io;
    this.speedsSupplier = chassisSpeedsSupplier;
    this.robotRotationSupplier = robotRotationSupplier;
    this.turretHasClearanceSupplier = turretHasClearanceSupplier;

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

    int hashCode = hashCode();

    if (kP.hasChanged(hashCode) || kI.hasChanged(hashCode) || kD.hasChanged(hashCode)) {
      io.setPID(kP.get(), kI.get(), kD.get());
    }
    if (kS.hasChanged(hashCode) || kV.hasChanged(hashCode) || kA.hasChanged(hashCode)) {
      io.setFFGains(kS.get(), kV.get(), kA.get());
    }
    if (mmCruiseVel.hasChanged(hashCode) || mmAcceleration.hasChanged(hashCode)) {
      io.setMotionMagicLimits(mmCruiseVel.get(), mmAcceleration.get());
    }
  }

  /** Follows a robot-centric angle. */
  public void followRobotCentricTarget(Supplier<Rotation2d> robotCentricAngleSupplier) {
    if (!hasZeroed || !turretHasClearanceSupplier.getAsBoolean()) return;

    double setpointTurretRads = wrap(robotCentricAngleSupplier.get().getRadians());
    double setpointMotorRots = setpointTurretRads / TurretConstants.TURRET_P_COEFFICIENT;

    double ffVolts = getFF(setpointTurretRads);

    io.runPosition(setpointMotorRots, ffVolts);

    Logger.recordOutput(
        "Turret/Setpoint Turret Degrees", Units.radiansToDegrees(setpointTurretRads));
    Logger.recordOutput("Turret/Setpoint Motor Rots", setpointMotorRots);
    Logger.recordOutput("Turret/FF Volts", ffVolts);
  }

  /** The operator-adjustable offset between the field frame and the turret's zero. */
  private Rotation2d getFieldRelativeOffset() {
    return Rotation2d.fromDegrees(
        fieldRelOffset.get() + Units.radiansToDegrees(turretRotationAdjust));
  }

  /**
   * Converts a field-relative target angle into the robot-relative turret setpoint. See the angle
   * convention comment in {@link TurretConstants}; {@link #getFieldRelativeTurretAngleRotation2d()}
   * is the inverse of this.
   */
  private Rotation2d fieldToTurretSetpoint(Rotation2d fieldAngle) {
    return robotRotationSupplier.get().minus(fieldAngle).plus(getFieldRelativeOffset());
  }

  public void followFieldCentricTarget(Supplier<Rotation2d> fieldCentricAngleSupplier) {
    followRobotCentricTarget(() -> fieldToTurretSetpoint(fieldCentricAngleSupplier.get()));
  }

  /**
   * Returns whether the turret is pointed at a field-relative angle, within a tolerance in degrees.
   *
   * <p>The comparison happens in robot-relative space, where Rotation2d.minus wraps to +/-180. That
   * makes it immune to which full-turn candidate {@link #wrap} picked for the commanded setpoint.
   */
  public boolean isAtFieldAngle(Rotation2d fieldAngle, double toleranceDegrees) {
    if (!hasZeroed) return false;

    double errorDegrees =
        getRobotRelativeTurretAngleRotation2d()
            .minus(fieldToTurretSetpoint(fieldAngle))
            .getDegrees();

    Logger.recordOutput("Turret/Field Angle Error Degrees", errorDegrees);

    return Math.abs(errorDegrees) < toleranceDegrees;
  }

  public void goToZero() {
    double setpointDegs = Units.radiansToDegrees(turretRotationAdjust);
    io.runPosition(Units.degreesToRadians(setpointDegs) / TurretConstants.TURRET_P_COEFFICIENT, 0);
    Logger.recordOutput("Turret/Setpoint Turret Degrees", setpointDegs);
  }

  public void goToTestSetpoint() {
    double setpointDegs = testSetpoint.get() + Units.radiansToDegrees(turretRotationAdjust);
    io.runPosition(Units.degreesToRadians(setpointDegs) / TurretConstants.TURRET_P_COEFFICIENT, 0);
    Logger.recordOutput("Turret/Setpoint Turret Degrees", setpointDegs);
  }

  /** Zeroes the turret and sets the motor to brake mode */
  public void requestZero() {
    if (inputs.cancoderConnected) {
      io.setPosition(
          (inputs.cancoderPosition
                      * TurretConstants.CANCODER_SIGN
                      * TurretConstants.TURRET_TO_CANCODER_RATIO)
                  * TurretConstants.TURRET_GEAR_RATIO
              + TurretConstants.TURRET_STARTING_ANGLE / TurretConstants.TURRET_P_COEFFICIENT);
      hasZeroed = true;
      setBrakeMode(true);
    }
  }

  /** Sets the motor back to coast mode */
  public void undoZero() {
    hasZeroed = false;
    setBrakeMode(false);
  }

  private void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
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

  /** Inverse of {@link #fieldToTurretSetpoint}: where the turret actually points on the field. */
  @AutoLogOutput
  public Rotation2d getFieldRelativeTurretAngleRotation2d() {
    return robotRotationSupplier
        .get()
        .minus(getRobotRelativeTurretAngleRotation2d())
        .plus(getFieldRelativeOffset());
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
