package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;

public abstract class ClimberIOTalonFX implements ClimberIO {
  protected final PearadoxTalonFX climber;
  protected final PositionVoltage climberControl;

  public ClimberIOTalonFX() {
    climber =
        new PearadoxTalonFX(
            ClimberConstants.CLIMBER_MOTOR_ID,
            ClimberConstants.getClimberConfigTalonFX(),
            Compeartment.CLIMBER);

    climberControl = new PositionVoltage(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberMotorData = climber.getData();
  }

  @Override
  public void runPosition(double setpointRots) {
    climber.setControl(climberControl.withPosition(setpointRots));
  }

  @Override
  public void zeroClimber() {
    climber.setPosition(0.0);
  }
}
