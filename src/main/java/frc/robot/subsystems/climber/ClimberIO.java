package frc.robot.subsystems.climber;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public MotorData climberMotorData = new MotorData();
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void runPosition(double setpointRots) {}

  public default void zeroClimber() {}
}
