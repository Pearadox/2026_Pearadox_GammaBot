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

    public MotorData hoodData = new MotorData();
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void stopLauncher() {}

  /**
   * @param velocityRPS the rotor velocity setpoint in Rotations per Second
   * @param ffamps the feedforward value applied to the launcher motors in Amps
   */
  public default void runLauncherVelocity(double velocityRPS, double ffamps) {}

  /**
   * @param velocityRPS the rotor velocity setpoint in Rotations per Second
   */
  public default void runLauncherVelocity(double velocityRPS) {}

  /**
   * @param angleRads the desired angle of the hood
   */
  public default void setHoodAngleRads(double angleRads) {}

  // /**
  //  * @param isPassing if the robot is in PASSING mode or other modes
  //  */
  // public default void setHoodAngle(boolean isPassing) {}

  public default void setPIDFF(double kP, double kD, double kS, double kV) {}

  public default void setCurrentLimits(double stator, double supply) {}
}
