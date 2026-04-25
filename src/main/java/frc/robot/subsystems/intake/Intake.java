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
    io.setPIDFF(rollerkP.get(), rollerkV.get(), pivotkp.get(), pivotkd.get(), pivotkg.get());
  }

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // private static LoggedTunableNumber loggedIntakeRollerVoltage =
  //     new LoggedTunableNumber("Intake/Voltage", 4.0);
  // private static LoggedTunableNumber rps = new LoggedTunableNumber("Intake/rps", 100.0);
  // private static LoggedTunableNumber ffamps = new LoggedTunableNumber("Intake/ffamps", 30.0);
  private static LoggedTunableNumber rollerkP = new LoggedTunableNumber("Intake/roller kp", 0.05);
  private static LoggedTunableNumber rollerkV = new LoggedTunableNumber("Intake/roller kv", 0.05);
  private static LoggedTunableNumber pivotkp = new LoggedTunableNumber("Intake/pivot kp", 0.4);
  private static LoggedTunableNumber pivotkd = new LoggedTunableNumber("Intake/pivotkd", 0.0);
  // might need retuning
  private static LoggedTunableNumber pivotkg =
      new LoggedTunableNumber("Intake/pivot kg", 0); // -0.45

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/State", intakeState.toString());
    Logger.recordOutput(
        "Intake/VoltageOut", StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage() + voltAdjust);

    io.runRollersVolts(StateConfig.INTAKE_STATE_MAP.get(intakeState).voltage() + voltAdjust);


    io.runPositionDegrees(
        StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg() + pivotDegreesAdjust,
        getFFVolts(), StateConfig.INTAKE_STATE_MAP.get(intakeState).slot());

    Logger.recordOutput(
        "Intake/Target Position Rots",
        Units.degreesToRotations(StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg())
            * IntakeConstants.GEARING);
    Logger.recordOutput(
        "Intake/Target Position Degrees", StateConfig.INTAKE_STATE_MAP.get(intakeState).angleDeg());
    Logger.recordOutput(
        "Intake/Current Position Degrees",
        Units.rotationsToDegrees(inputs.pivot1MotorData.position()) / IntakeConstants.GEARING);

    if (rollerkP.hasChanged(hashCode())
        || rollerkV.hasChanged(hashCode())
        || pivotkp.hasChanged(hashCode())
        || pivotkd.hasChanged(hashCode())
        || pivotkg.hasChanged(hashCode())) {
      io.setPIDFF(rollerkP.get(), rollerkV.get(), pivotkp.get(), pivotkd.get(), pivotkg.get());
    }
  }

  private double getFFVolts() {
    return Math.sin(Math.toRadians(getAngleDegs())) * pivotkg.get();
  }

  @AutoLogOutput
  public double getAngleDegs() {
    return Units.rotationsToDegrees(inputs.pivot1MotorData.position()) / IntakeConstants.GEARING;
  }

  public void setStowed() {
    intakeState = IntakeState.STOWED;
  }

  public void setFlow() {
    intakeState = IntakeState.FLOW_STATE;
  }

  public void setHold() {
    intakeState = IntakeState.HOLD_STATE;
  }

  public void setDeployed() {
    intakeState = IntakeState.DEPLOYED; // rollers aren't running; intake is deployed
  }

  public void setIntaking() {
    IntakeConstants.PIVOT_CONFIG.withSlot1(IntakeConstants.PIVOT_SLOT1_CONFIGS);
    intakeState = IntakeState.INTAKING;
  }

  public void setIntakingFast() {
    intakeState = IntakeState.INTAKING;
  }

  public void setOuttaking() {
    intakeState = IntakeState.OUTTAKING;
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }

  @AutoLogOutput
  public boolean turretHasClearance() {
    return getAngleDegs() > IntakeConstants.MIN_ANGLE_FOR_TURRET_CLEARANCE_DEGS;
  }
}
