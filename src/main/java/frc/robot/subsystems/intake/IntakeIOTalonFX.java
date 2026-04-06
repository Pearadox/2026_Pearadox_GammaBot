package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;
import frc.robot.util.PhoenixUtil;

public abstract class IntakeIOTalonFX implements IntakeIO {
  protected final PearadoxTalonFX roller1Leader;
  protected final PearadoxTalonFX roller2Follower;
  protected final PearadoxTalonFX pivot1Leader;
  protected final PearadoxTalonFX pivot2Follower;
  protected final PositionVoltage pivotPositionVoltage;
  protected final VoltageOut rollerVoltage;
  protected final Follower rollerFollowerRequest;
  protected final Follower pivotFollowerRequest;
  protected final TorqueCurrentFOC torqueCurrentFOC;
  protected final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;

  private TalonFXConfiguration rollerConfigs;
  private TalonFXConfiguration pivotConfigs;

  protected IntakeIOTalonFX() {

    pivotConfigs = IntakeConstants.getPivotConfigTalonFX();
    rollerConfigs = IntakeConstants.getRollerConfigTalonFX();

    roller1Leader =
        new PearadoxTalonFX(
            IntakeConstants.ROLLER_1_LEADER_ID, rollerConfigs, Compeartment.INTAKE_ROLLERS);
    roller2Follower =
        new PearadoxTalonFX(
            IntakeConstants.ROLLER_2_FOLLOWER_ID, rollerConfigs, Compeartment.INTAKE_ROLLERS);
    pivot1Leader =
        new PearadoxTalonFX(
            IntakeConstants.PIVOT_1_LEADER_ID, pivotConfigs, Compeartment.INTAKE_PIVOT);
    pivot2Follower =
        new PearadoxTalonFX(
            IntakeConstants.PIVOT_2_FOLLOWER_ID, pivotConfigs, Compeartment.INTAKE_PIVOT);

    pivotPositionVoltage = new PositionVoltage(0);
    torqueCurrentFOC = new TorqueCurrentFOC(0);
    velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);

    pivotFollowerRequest =
        new Follower(IntakeConstants.PIVOT_1_LEADER_ID, MotorAlignmentValue.Opposed);
    pivot2Follower.setControl(pivotFollowerRequest);

    rollerVoltage = new VoltageOut(0);
    rollerFollowerRequest =
        new Follower(IntakeConstants.ROLLER_1_LEADER_ID, MotorAlignmentValue.Opposed);
    roller2Follower.setControl(rollerFollowerRequest);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.roller1MotorData = roller1Leader.getData();
    inputs.roller2MotorData = roller2Follower.getData();

    inputs.pivot1MotorData = pivot1Leader.getData();
    inputs.pivot2MotorData = pivot2Follower.getData();
  }

  @Override
  public void runRollersAmps(double amps, double maxDutyOut) {
    roller1Leader.setControl(torqueCurrentFOC.withOutput(amps).withMaxAbsDutyCycle(maxDutyOut));

    roller2Follower.setControl(rollerFollowerRequest);
  }

  @Override
  public void runRollersVelocityTorqueCurrentFOC(double velocity, double ffAmps) {
    roller1Leader.setControl(velocityTorqueCurrentFOC.withVelocity(velocity));

    roller2Follower.setControl(rollerFollowerRequest);
  }

  @Override
  public void runRollersVolts(double volts) {
    roller1Leader.setControl(rollerVoltage.withOutput(volts).withEnableFOC(false));
  }

  @Override
  public void runPositionDegrees(double degrees, double ffvolts) {
    pivot1Leader.setControl(
        pivotPositionVoltage
            .withPosition(Units.degreesToRotations(degrees) * IntakeConstants.GEARING)
            .withFeedForward(ffvolts));
    // pivotMotor.setControl(new PositionVoltage(Units.degreesToRotations(degrees)));

    pivot2Follower.setControl(pivotFollowerRequest);
  }

  @Override
  public void setPIDFF(double kp, double kv, double pivotkp, double pivotkd) {
    rollerConfigs.Slot0.kP = kp;
    rollerConfigs.Slot0.kV = kv;

    pivotConfigs.Slot0.kP = pivotkp;
    pivotConfigs.Slot0.kD = pivotkd;

    PhoenixUtil.tryUntilOk(5, () -> roller1Leader.getConfigurator().apply(rollerConfigs));
    PhoenixUtil.tryUntilOk(5, () -> roller2Follower.getConfigurator().apply(rollerConfigs));
    PhoenixUtil.tryUntilOk(5, () -> pivot1Leader.getConfigurator().apply(pivotConfigs));
    PhoenixUtil.tryUntilOk(5, () -> pivot2Follower.getConfigurator().apply(pivotConfigs));
  }
}
