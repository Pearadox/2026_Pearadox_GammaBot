// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederConstants.FeederState;
import frc.robot.subsystems.feeder.FeederConstants.StateConfig;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private FeederState feederState = FeederState.STOPPED;

  private Debouncer canRangeDebouncer = new Debouncer(0.1, DebounceType.kFalling);
  private boolean isDetectedDebounced = false;

  private int fuelCount = 0;
  private boolean lastDetected = false;
  private boolean hasSeenFuel = false;

  private Timer timer = new Timer();

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    this.io = io;
    timer.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    isDetectedDebounced = canRangeDebouncer.calculate(inputs.canRangeIsDetected);
    Logger.processInputs("FeederInputs", inputs);
    Logger.recordOutput("Feeder/CanRange/Distance from Fuel", canRangeGetDistanceMeters());
    Logger.recordOutput("Feeder/CanRange/FuelIsDetected", isDetectedDebounced());
    Logger.recordOutput("Feeder/CanRange/Number of Fuel", getFuelCount());
    Logger.recordOutput("Feeder/CanRange/Signal", canRangeGetSignalStrength());
    io.runFeederVoltage(StateConfig.FEEDER_STATE_MAP.get(feederState).voltage());
    updateFuelCount();
  }

  public void setStopped() {
    feederState = FeederState.STOPPED;
  }

  public void setRunning() {
    feederState = FeederState.RUNNING;
  }

  // CANRange methods
  public boolean isDetectedDebounced() {
    return isDetectedDebounced;
  }

  public void updateFuelCount() {

    if (isDetectedDebounced && !lastDetected) {
      fuelCount++;
      hasSeenFuel = true; // solves the edge case where we don't see fuel
    }

    // reset timer any time fuel is detected
    if (isDetectedDebounced && hasSeenFuel) {
      timer.reset();
    }

    lastDetected = isDetectedDebounced;
  }

  public double canRangeGetDistanceMeters() {
    return inputs.canRangeDistanceMeters;
  }

  public double canRangeGetSignalStrength() {
    return inputs.canRangeSignal;
  }

  public int getFuelCount() {
    return fuelCount;
  }

  public boolean isHopperEmpty() {
    return hasSeenFuel
        && !isDetectedDebounced
        && getTimestamp() > FeederConstants.IS_HOPPER_EMPTY_BUFFER_TIME;
  }

  // timer methods
  public double getTimestamp() {
    return timer.get();
  }

  public void startTimer() {
    timer.start();
  }

  // public void launch() {
  //   io.runFeederVoltage(FeederConstants.FEEDER_ACTIVE_VOLTAGE);
  // }

  // public void stopLaunch() {
  //   io.runFeederVoltage(0.0);
  // }
}
