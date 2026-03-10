package frc.robot.subsystems.turret;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  static class TurretIOInputs {
    public MotorData turretData = new MotorData();

    public double cancoderPosition = 0.0;
    public boolean cancoderConnected = false;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void runPosition(double setpointRots, double ffVolts) {}

  default void runVoltage(double volts) {}

  default void setPosition(double newPositionRots) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setFFGains(double kS, double kV, double kA) {}

  default void setMotionMagicLimits(double mmCruiseVel, double mmAcceleration) {}

  default void setBrakeMode(boolean brake) {}
}
