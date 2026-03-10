package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.PhoenixUtil;

public abstract class TurretIOTalonFX implements TurretIO {
  protected final PearadoxTalonFX turretMotor;
  protected final MotionMagicVoltage turretMMRequest = new MotionMagicVoltage(0);
  protected final VoltageOut voltageRequest = new VoltageOut(0);

  protected final TalonFXConfiguration configs = TurretConstants.getTurretConfig();

  protected TurretIOTalonFX() {
    turretMotor = new PearadoxTalonFX(TurretConstants.TURRET_ID, configs);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretData = turretMotor.getData();
  }

  @Override
  public void runPosition(double setpointRots, double ffVolts) {
    turretMotor.setControl(
        turretMMRequest.withPosition(setpointRots).withFeedForward(ffVolts).withEnableFOC(true));
  }

  @Override
  public void runVoltage(double volts) {
    turretMotor.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void setPosition(double newPositionRots) {
    turretMotor.setPosition(newPositionRots);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(configs));
  }

  @Override
  public void setFFGains(double kS, double kV, double kA) {
    configs.Slot0.kS = kS;
    configs.Slot0.kV = kV;
    configs.Slot0.kA = kA;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(configs));
  }

  @Override
  public void setMotionMagicLimits(double mmCruiseVel, double mmAcceleration) {
    configs.MotionMagic.MotionMagicCruiseVelocity = mmCruiseVel;
    configs.MotionMagic.MotionMagicAcceleration = mmAcceleration;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(configs));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    configs.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(configs));
  }
}
