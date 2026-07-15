package frc.robot.subsystems.chrisintake;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface ChrisIntakeIO {
  @AutoLog
  public static class ChrisIntakeIOInputs {
    public MotorData chrisIntakeMotorData = new MotorData();
  }

  public default void updateInputs(ChrisIntakeIOInputs inputs) {}

  public default void runPosition(double setpointRots) {}

  public default void zeroIntake() {}
}
