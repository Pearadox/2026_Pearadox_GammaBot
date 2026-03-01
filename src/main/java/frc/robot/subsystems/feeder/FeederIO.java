// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import frc.lib.drivers.PearadoxTalonFX.MotorData;
import org.littletonrobotics.junction.AutoLog;

/** Interface for Feeder subsystem */
public interface FeederIO {

  @AutoLog
  public static class FeederIOInputs {
    public MotorData feederData = new MotorData();
    public double canRangeDistanceMeters = 0.0;
    public boolean canRangeIsDetected = false;
  }

  public default void updateInputs(FeederIOInputsAutoLogged inputs) {}

  /**
   * @param voltage voltage to run the feeder at
   */
  public default void runFeederVoltage(double voltage) {}
}
