package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.FeederConstants;

public class IntakeIOSim extends IntakeIOTalonFX {

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(2),
          IntakeConstants.GEARING,
          SingleJointedArmSim.estimateMOI(IntakeConstants.LENGTH_METERS, IntakeConstants.MASS_KG),
          IntakeConstants.LENGTH_METERS,
          IntakeConstants.SIM_MIN_ANGLE_RADS,
          IntakeConstants.SIM_MAX_ANGLE_RADS,
          false,
          IntakeConstants.SIM_STARTING_ANGLE_RADS);

  private DCMotorSim rollerPhysicsSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              FeederConstants.FEEDER_MOTOR, 0.00001, FeederConstants.FEEDER_GEARING),
          FeederConstants.FEEDER_MOTOR);

  private TalonFXSimState pivotSimState;
  private TalonFXSimState rollerSimState;

  public IntakeIOSim() {
    pivotSimState = pivot1Leader.getSimState();
    rollerSimState = roller1Leader.getSimState();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);

    pivotSimState.setSupplyVoltage(12);

    pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
    pivotSim.update(Constants.UPDATE_FREQ_SEC);

    pivotSimState.setRawRotorPosition(
        (Units.radiansToRotations(pivotSim.getAngleRads()) * IntakeConstants.GEARING));
    pivotSimState.setRotorVelocity(
        Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * IntakeConstants.GEARING);

    rollerSimState.setSupplyVoltage(12);
    rollerPhysicsSim.setInputVoltage(rollerSimState.getMotorVoltage());
    rollerPhysicsSim.update(0.02);

    rollerSimState.setRawRotorPosition(
        rollerPhysicsSim.getAngularPositionRotations() * FeederConstants.FEEDER_GEARING);
    rollerSimState.setRotorVelocity(
        rollerPhysicsSim.getAngularVelocityRPM() * FeederConstants.FEEDER_GEARING / 60.0);
  }
}
