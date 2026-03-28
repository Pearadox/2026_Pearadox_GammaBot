// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/** Launcher IO's base class for TalonFX motors */
public abstract class LauncherIOTalonFX implements LauncherIO {

  protected final PearadoxTalonFX launcher1Leader;
  protected final PearadoxTalonFX launcher2Follower;

  // protected final VelocityVoltage launcher1Control;
  protected final VelocityTorqueCurrentFOC launcher1Control;
  protected final Follower launcher2Control;

  protected final VelocityVoltage velocityVoltageRequest;

  // protected final ServoHub hoodServoHub;

  // protected final ServoChannel hoodServo1;
  // protected final ServoChannel hoodServo2;

  private final TalonFXConfiguration launcherConfigs;

  public LauncherIOTalonFX() {
    launcherConfigs = LauncherConstants.LAUNCHER_MOTOR_CONFIG();

    launcher1Leader =
        new PearadoxTalonFX(
            LauncherConstants.LAUNCHER_1_CAN_ID, launcherConfigs, Compeartment.LAUNCHER);
    launcher2Follower =
        new PearadoxTalonFX(
            LauncherConstants.LAUNCHER_2_CAN_ID, launcherConfigs, Compeartment.LAUNCHER);

    // launcher1Control = new VelocityVoltage(0);
    launcher1Control = new VelocityTorqueCurrentFOC(0);
    velocityVoltageRequest = new VelocityVoltage(0);
    launcher2Control = new Follower(launcher1Leader.getDeviceID(), MotorAlignmentValue.Opposed);

    // hoodServoHub = new ServoHub(LauncherConstants.HOOD_SERVO_HUB_CAN_ID);

    // hoodServo1 = hoodServoHub.getServoChannel(LauncherConstants.HOOD_1_ID);
    // hoodServo1.setEnabled(true);
    // hoodServo1.setPowered(true);

    // hoodServo2 = hoodServoHub.getServoChannel(LauncherConstants.HOOD_2_ID);
    // hoodServo2.setEnabled(true);
    // hoodServo2.setPowered(true);
  }

  public void updateInputs(LauncherIOInputs inputs) {
    inputs.launcher1Data = launcher1Leader.getData();
    inputs.launcher2Data = launcher2Follower.getData();

    // inputs.hoodServoHubVoltage = hoodServoHub.getDeviceVoltage();

    // inputs.hoodServo1Position =
    //     LauncherConstants.pulseWidthtoAngularPosition(hoodServo1.getPulseWidth());
    // inputs.hoodServo2Position =
    //     LauncherConstants.pulseWidthtoAngularPosition(hoodServo2.getPulseWidth());
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

  @Override
  public void runLauncherVelocityWithoutFOC(double velocityRPS) {
    launcher1Leader.setControl(
        velocityVoltageRequest.withVelocity(velocityRPS).withEnableFOC(false));
    launcher2Follower.setControl(launcher2Control);
  }

  public void stopLauncher() {
    launcher1Leader.stopMotor();
    launcher2Follower.stopMotor();
  }

  public void setHoodAngleRads(double angleRads) {
    // if (angleRads < LauncherConstants.HOOD_MAX_ANGLE_RADS
    //     && angleRads > LauncherConstants.HOOD_MIN_ANGLE_RADS) {
    //   double angleRadsFromMinimum = angleRads - LauncherConstants.HOOD_MIN_ANGLE_RADS;

    //   double servoRotations =
    //       (Units.radiansToRotations(angleRadsFromMinimum) * LauncherConstants.HOOD_GEARING)
    //           / LauncherConstants.SERVO_POSITION_TO_ROTATIONS_CONVERSION;

    //   hoodServo1.setPulseWidth(LauncherConstants.rotationstoPulseWidth(servoRotations));
    //   hoodServo2.setPulseWidth(
    //       LauncherConstants.SERVO_MAX_PULSE_WIDTH
    //           - LauncherConstants.rotationstoPulseWidth(servoRotations));
    // } else {
    //   Logger.recordOutput("HoodAngleOutOfRange", angleRads);
    // }
  }

  @Override
  public void setPIDFF(double kP, double kD, double kS, double kV) {
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
}
