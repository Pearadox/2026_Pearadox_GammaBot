package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class IntakeIOSim extends IntakeIOTalonFX {

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          IntakeConstants.GEARING,
          SingleJointedArmSim.estimateMOI(IntakeConstants.LENGTH_METERS, IntakeConstants.MASS_KG),
          IntakeConstants.LENGTH_METERS,
          IntakeConstants.SIM_MIN_ANGLE_RADS,
          IntakeConstants.SIM_MAX_ANGLE_RADS,
          false,
          IntakeConstants.SIM_STARTING_ANGLE_RADS);

  private TalonFXSimState pivotSimState;

  public IntakeIOSim() {
    pivotSimState = pivot1Leader.getSimState();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);

    pivotSimState.setSupplyVoltage(12);

    pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSim.update(Constants.UPDATE_FREQ_SEC);

    pivotSimState.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads()));
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * IntakeConstants.GEARING);
  }
}
