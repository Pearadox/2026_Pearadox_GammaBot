package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import frc.lib.drivers.PearadoxTalonFX;

public abstract class IntakeIOTalonFX implements IntakeIO {
  protected final PearadoxTalonFX roller1Leader;
  protected final PearadoxTalonFX roller2Follower;
  protected final PearadoxTalonFX pivotMotor;
  protected final PositionVoltage pivotPositionVoltage;
  protected final VoltageOut rollerVoltage;
  protected final Follower followerRequest;
  protected final TorqueCurrentFOC torqueCurrentFOC;
  protected final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

  // private TalonFXConfiguration rollerConfigs;
  // private TalonFXConfiguration pivotConfigs;

  protected IntakeIOTalonFX() {

    // pivotConfigs = IntakeConstants.getPivotConfigTalonFX();
    // rollerConfigs = IntakeConstants.getRollerConfigTalonFX();

    roller1Leader =
        new PearadoxTalonFX(
            IntakeConstants.ROLLER_1_LEADER_ID, IntakeConstants.getRollerConfigTalonFX());
    roller2Follower =
        new PearadoxTalonFX(
            IntakeConstants.ROLLER_2_FOLLOWER_ID, IntakeConstants.getRollerConfigTalonFX());
    pivotMotor =
        new PearadoxTalonFX(IntakeConstants.PIVOT_ID, IntakeConstants.getPivotConfigTalonFX());
    pivotPositionVoltage = new PositionVoltage(0);
    torqueCurrentFOC = new TorqueCurrentFOC(0);
    velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

    rollerVoltage = new VoltageOut(0);
    followerRequest = new Follower(IntakeConstants.ROLLER_1_LEADER_ID, MotorAlignmentValue.Opposed);
    roller2Follower.setControl(followerRequest);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorData = roller1Leader.getData();
    inputs.roller2MotorData = roller2Follower.getData();

    inputs.pivotMotorData = pivotMotor.getData();
  }

  @Override
  public void runRollersAmps(double amps, double maxDutyOut) {
    // roller1Leader.setControl(rollerVoltage.withOutput(volts));
    roller1Leader.setControl(torqueCurrentFOC.withOutput(amps).withMaxAbsDutyCycle(maxDutyOut));

    roller2Follower.setControl(followerRequest);
  }

  @Override
  public void runRollersVelocityTorqueCurrentFOC(double velocity) {
    // roller1Leader.setControl(rollerVoltage.withOutput(volts));
    roller1Leader.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));

    roller2Follower.setControl(followerRequest);
  }

  @Override
  public void runRollersVolts(double volts) {
    roller1Leader.setControl(rollerVoltage.withOutput(volts));
  }

  @Override
  public void runPositionDegrees(double degrees) {
    pivotMotor.setControl(
        pivotPositionVoltage.withPosition(
            Units.degreesToRotations(degrees) * IntakeConstants.GEARING));
    // pivotMotor.setControl(new PositionVoltage(Units.degreesToRotations(degrees)));
  }
}
