// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/** Launcher IO's base class for TalonFX motors */
public abstract class LauncherIOTalonFX implements LauncherIO {

  protected final PearadoxTalonFX launcher1Leader;
  protected final PearadoxTalonFX launcher2Follower;

  protected final VelocityTorqueCurrentFOC launcher1Control;
  protected final VelocityVoltage velocityVoltageRequest;
  protected final VoltageOut voltageRequest;
  protected final Follower launcher2Control;

  private final TalonFXConfiguration launcherConfigs;

  protected final PearadoxTalonFX hood;
  private final TalonFXConfiguration hoodConfigs;

  protected final PositionVoltage hoodControl;

  public LauncherIOTalonFX() {
    launcherConfigs = LauncherConstants.LAUNCHER_MOTOR_CONFIG();

    launcher1Leader =
        new PearadoxTalonFX(
            LauncherConstants.LAUNCHER_1_CAN_ID, launcherConfigs, Compeartment.LAUNCHER);
    launcher2Follower =
        new PearadoxTalonFX(
            LauncherConstants.LAUNCHER_2_CAN_ID, launcherConfigs, Compeartment.LAUNCHER);

    launcher1Control = new VelocityTorqueCurrentFOC(0);
    velocityVoltageRequest = new VelocityVoltage(0);
    voltageRequest = new VoltageOut(0);
    launcher2Control = new Follower(launcher1Leader.getDeviceID(), MotorAlignmentValue.Opposed);

    hoodConfigs = LauncherConstants.HOOD_CONFIG();

    hood = new PearadoxTalonFX(LauncherConstants.HOOD_ID, hoodConfigs, Compeartment.HOOD);

    hoodControl = new PositionVoltage(0);
  }

  public void updateInputs(LauncherIOInputs inputs) {
    inputs.launcher1Data = launcher1Leader.getData();
    inputs.launcher2Data = launcher2Follower.getData();

    inputs.hoodData = hood.getData();
  }

  public void runLauncherVelocity(double velocityRPS, double ffamps) {
    launcher1Leader.setControl(launcher1Control.withVelocity(velocityRPS).withFeedForward(ffamps));
    launcher2Follower.setControl(launcher2Control);
    Logger.recordOutput("Launcher/VelocitySetpointRPS", velocityRPS);
    // Logger.recordOutput(
    //     "Launcher/ActualVelocityRPS", launcher1Leader.getVelocity().getValueAsDouble());
    // Logger.recordOutput(
    //     "Launcher/CurrentDrawStator", launcher1Leader.getStatorCurrent().getValueAsDouble());
  }
  
  public void runLauncherVelocity(double velocityRPS) {
    launcher1Leader.setControl(
      velocityVoltageRequest.withVelocity(velocityRPS).withEnableFOC(false));
    launcher2Follower.setControl(launcher2Control);
    Logger.recordOutput("Launcher/VelocitySetpointRPS", velocityRPS);
  }
  
  public void setLauncherVoltage(double voltage) {
    launcher1Leader.setControl(voltageRequest.withOutput(voltage));
    launcher2Follower.setControl(launcher2Control);
  }

  public void stopLauncher() {
    launcher1Leader.stopMotor();
    launcher2Follower.stopMotor();
  }

  public void setHoodAngleRads(double angleRads, double feedforward) {
    if (angleRads <= LauncherConstants.HOOD_MAX_ANGLE_RADS
        && angleRads >= LauncherConstants.HOOD_MIN_ANGLE_RADS) {
      double setpoint = Units.radiansToRotations(angleRads - LauncherConstants.HOOD_MIN_ANGLE_RADS)
              * LauncherConstants.HOOD_GEARING;
      hood.setControl(hoodControl.withPosition(setpoint).withFeedForward(feedforward));
      Logger.recordOutput("Hood/AngleSetpointRots", setpoint);
      Logger.recordOutput(
          "Hood/HoodError", Math.abs(setpoint - hood.getPosition().getValueAsDouble()));
      Logger.recordOutput("Hood/HoodAngle-inRange", true);
    } else {
      Logger.recordOutput("Hood/HoodAngle-inRange", false);
    }
  }

  public void zeroHood() {
    hood.setPosition(0);
  }

  @Override
  public void setLauncherPIDFF(double kP, double kD, double kS, double kV) {
    launcherConfigs.Slot0.kP = kP;
    launcherConfigs.Slot0.kD = kD;
    launcherConfigs.Slot0.kS = kS;
    launcherConfigs.Slot0.kV = kV;

    PhoenixUtil.tryUntilOk(5, () -> launcher1Leader.getConfigurator().apply(launcherConfigs));
    PhoenixUtil.tryUntilOk(5, () -> launcher2Follower.getConfigurator().apply(launcherConfigs));
  }

  @Override
  public void setCurrentLimits(double stator, double supply) {
    launcherConfigs.CurrentLimits.StatorCurrentLimit = stator;
    launcherConfigs.CurrentLimits.SupplyCurrentLimit = supply;

    PhoenixUtil.tryUntilOk(5, () -> launcher1Leader.getConfigurator().apply(launcherConfigs));
    PhoenixUtil.tryUntilOk(5, () -> launcher2Follower.getConfigurator().apply(launcherConfigs));
  }

  @Override
  public void setHoodPIDFF(double kP, double kI, double kD, double kS, double kG) {
    hoodConfigs.Slot0.kP = kP;
    hoodConfigs.Slot0.kI = kI;
    hoodConfigs.Slot0.kD = kD;
    hoodConfigs.Slot0.kS = kS;
    hoodConfigs.Slot0.kG = kG;

    PhoenixUtil.tryUntilOk(5, () -> hood.getConfigurator().apply(hoodConfigs));
  }
}
