package frc.robot.subsystems.chrisintake;

import com.ctre.phoenix6.controls.PositionVoltage;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.subsystems.chrisintake.ChrisIntakeConstants;
import frc.robot.util.EnergyTracker.Compeartment;

public abstract class ChrisIntakeIOTalonFX implements ChrisIntakeIO {
  protected final PearadoxTalonFX intakeRollers;
  protected final PositionVoltage intakeControl;

public ChrisIntakeIOTalonFX() {
    intakeRollers =
        new PearadoxTalonFX(
            ChrisIntakeConstants.LEFT_ROLLER_INTAKE_MOTOR_ID,
            ChrisIntakeConstants.getIntakeConfigTalonFX(),
            Compeartment.INTAKE_ROLLERS);

    intakeControl = new PositionVoltage(0);
  }

  public void rollForward() {
    intakeRollers.setVoltage(12);
  }

}
