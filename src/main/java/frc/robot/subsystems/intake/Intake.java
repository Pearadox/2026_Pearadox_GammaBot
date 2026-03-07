package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants.StateConfig;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;

  public IntakeState intakeState = IntakeState.DEPLOYED;

  public static double pivotDegreesAdjust = 0.0;

  public void adjustPivotAngleBy(double adj) {
    pivotDegreesAdjust += adj;
  }

  public Intake(IntakeIO io) {
    this.io = io;
  }

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // private static LoggedTunableNumber loggedIntakeRollerVoltage =
  //     new LoggedTunableNumber("Intake/Voltage", 4.0);

  private static LoggedTunableNumber loggedIntakeStatorCurrent =
      new LoggedTunableNumber("Intake/StatorCurrent", 45.0);
  private static LoggedTunableNumber maxDuty = new LoggedTunableNumber("Intake/Maxduty", 0.4);

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/State", intakeState.toString());

    // io.runRollersAmps(loggedIntakeStatorCurrent.get(), maxDuty.get());
    // io.runRollersVolts(StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage());

    // io.runRollersAmps( // TODO: UNCOMMENT
    //     StateConfig.INTAKE_STATE_MAP.get(intakeState).amps(),
    //     StateConfig.INTAKE_STATE_MAP.get(intakeState).maxDuty());
    // io.runPositionDegrees(
    //     StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg() + pivotDegreesAdjust);

    MechVisualizer.getInstance()
        .updatePositionDegrees(Units.rotationsToDegrees(inputs.pivotMotorData.position()));

    Logger.recordOutput(
        "Intake/Target Position Degrees", StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg());
    Logger.recordOutput("Intake/VoltageOut", inputs.rollerMotorData.appliedVolts());
    Logger.recordOutput(
        "Intake/Current Position Degrees",
        Units.rotationsToDegrees(inputs.pivotMotorData.position()) / IntakeConstants.GEARING);

    // UNCOMMENT WHEN TESTING INTAKE TO TUNE VOLTAGE!
    // if(loggedIntakeRollerVoltage.hasChanged(hashCode())) { inputs.rollerVoltage =
    // loggedIntakeRollerVoltage.get(); }

    // if (intakeState == IntakeState.INTAKING) {
    //     io.runRollersVolts(loggedIntakeRollerVoltage.getAsDouble());
    // } else {
    //     io.runRollersVolts(0);
    // }
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
