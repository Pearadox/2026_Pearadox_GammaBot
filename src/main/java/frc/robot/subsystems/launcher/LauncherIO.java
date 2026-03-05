// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

/** Launcher's IO interface */
public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public MotorData launcher1Data = new MotorData();
    public MotorData launcher2Data = new MotorData();

    public double hoodServoHubVoltage = 0.0;

    public double hoodServo1Position = 0.0;
    public double hoodServo2Position = 0.0;

    public double hoodServo1PulseWidth = 0.0;
    public double hoodServo2PulseWidth = 0.0;

    public boolean limitSwitchPressed = false;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void stopLauncher() {}

  /**
   * @param velocityRPS the rotor velocity setpoint in Rotations per Second
   */
  public default void runLauncherVelocity(double velocityRPS) {}

  /**
   * This is assuming that the left servo is at 0.0 and the right servo is at 1.0 when the hood is
   * at it's lowest.
   *
   * @param angleRads the desired angle of the hood
   */
  public default void setHoodAngleRads(double angleRads) {}

  /**
   * @param isPassing if the robot is in PASSING mode or other modes
   */
  public default void setHoodAngle(boolean isPassing) {}
}
