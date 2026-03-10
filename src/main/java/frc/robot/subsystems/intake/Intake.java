package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.StateConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  @AutoLogOutput public IntakeState intakeState = IntakeState.STOWED;
  @AutoLogOutput public static double pivotDegreesAdjust = 0.0;
  @AutoLogOutput public static double dutyAdjust = 0.0;
  @AutoLogOutput public static double voltAdjust = 0.0;

  public void adjustPivotAngleBy(double adj) {
    pivotDegreesAdjust += adj;
  }

  public void adjustVoltsBy(double volts) {
    voltAdjust += volts;
  }

  public void adjustMaxDuty(double adj) {
    dutyAdjust += adj;
  }

  public Intake(IntakeIO io) {
    this.io = io;
    io.setPIDFF(kp.get(), kv.get(), pivotkp.get(), pivotkd.get());
  }

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // private static LoggedTunableNumber loggedIntakeRollerVoltage =
  //     new LoggedTunableNumber("Intake/Voltage", 4.0);

  private static LoggedTunableNumber rps = new LoggedTunableNumber("Intake/rps", 100.0);
  private static LoggedTunableNumber ffamps = new LoggedTunableNumber("Intake/ffamps", 30.0);

  private static LoggedTunableNumber kp = new LoggedTunableNumber("Intake/roller kp", 0.05);
  private static LoggedTunableNumber kv = new LoggedTunableNumber("Intake/roller kv", 0.05);
  private static LoggedTunableNumber pivotkp = new LoggedTunableNumber("Intake/pivot kp", 0.5);
  private static LoggedTunableNumber pivotkd = new LoggedTunableNumber("Intake/pivotkd", 0.0);
  private static LoggedTunableNumber pivotkg = new LoggedTunableNumber("Intake/pivot kg", -0.5);

  // max duty 0.5
  // amps 45

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/State", intakeState.toString());
    Logger.recordOutput(
        "Intake/VoltageOut", StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage() + voltAdjust);

    // io.runRollersAmps(loggedIntakeStatorCurrent.get(), maxDuty.get());
    // io.runRollersVolts(StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage());

    // if (intakeState == IntakeState.INTAKING) {
    //   io.runRollersVelocityTorqueCurrentFOC(rps.get(), ffamps.get());

    // } else {
    io.runRollersVolts(StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage() + voltAdjust);

    // }
    io.runPositionDegrees(
        StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg() + pivotDegreesAdjust,
        getFFVolts());

    MechVisualizer.getInstance()
        .updatePositionDegrees(Units.rotationsToDegrees(inputs.pivotMotorData.position()));

    Logger.recordOutput(
        "Intake/Target Position Degrees", StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg());
    Logger.recordOutput("Intake/VoltageOut", inputs.rollerMotorData.appliedVolts());
    Logger.recordOutput(
        "Intake/Current Position Degrees",
        Units.rotationsToDegrees(inputs.pivotMotorData.position()) / IntakeConstants.GEARING);

    if (kp.hasChanged(hashCode())
        || kv.hasChanged(hashCode())
        || pivotkp.hasChanged(hashCode())
        || pivotkd.hasChanged(hashCode())) {
      io.setPIDFF(kp.get(), kv.get(), pivotkp.get(), pivotkd.get());
    }

    // UNCOMMENT WHEN TESTING INTAKE TO TUNE VOLTAGE!
    // if(loggedIntakeRollerVoltage.hasChanged(hashCode())) { inputs.rollerVoltage =
    // loggedIntakeRollerVoltage.get(); }

    // if (intakeState == IntakeState.INTAKING) {
    //     io.runRollersVolts(loggedIntakeRollerVoltage.getAsDouble());
    // } else {
    //     io.runRollersVolts(0);
    // }
  }

  private double getFFVolts() {
    return Math.sin(Math.toRadians(getAngleDegs())) * pivotkg.get();
  }

  public double getAngleDegs() {
    return Units.rotationsToDegrees(inputs.pivotMotorData.position()) / IntakeConstants.GEARING;
  }

  public void setStowed() {
    intakeState = IntakeState.STOWED;
  }

  public void setDeployed() {
    intakeState = IntakeState.DEPLOYED; // rollers aren't running; intake is deployed
  }

  public void setIntaking() {
    intakeState = IntakeState.INTAKING;
  }

  public void setOuttaking() {
    intakeState = IntakeState.OUTTAKING;
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }
}
