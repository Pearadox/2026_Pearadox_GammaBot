package frc.robot.subsystems.intake;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public MotorData rollerMotorData = new MotorData();
    public MotorData roller2MotorData = new MotorData();
    public MotorData pivotMotorData = new MotorData();
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runRollersAmps(double amps, double maxDutyOut) {}

  public default void runRollersVelocityTorqueCurrentFOC(double velocity, double ffAmps) {}

  public default void runRollersVolts(double volts) {}

  public default void runPositionDegrees(double degrees, double ffvolts) {}

  public default void setPIDFF(double rollerkp, double kv, double pivotkp, double pivotkd) {}
}
