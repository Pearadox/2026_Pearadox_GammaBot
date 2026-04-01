package frc.robot.subsystems.spindexer;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public MotorData spindexerMotorData = new MotorData();
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void runSpindexerTorqueCurrent(double amps, double maxDutyCycle) {}
}
