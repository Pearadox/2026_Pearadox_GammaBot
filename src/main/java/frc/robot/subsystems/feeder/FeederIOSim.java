// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants;
import frc.robot.util.EnergyTracker.Compeartment;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {

  // FEEDER_GEARING is mechanism-per-motor (the pulley turns a quarter turn per rotor turn), so the
  // reduction the motor model wants, and the factor that converts mechanism units back to rotor
  // units, is its inverse. Worth a look on the real robot: the constant is 1/4 while its own
  // comment describes tooth counts that work out to 12/20.
  private static final double FEEDER_REDUCTION = 1.0 / FeederConstants.FEEDER_GEARING;

  // Crude stand-in for the CANrange. There was no model at all, so canRangeIsDetected was always
  // false, hasSeenFuel never latched, isHopperEmpty() could never return true, and every sim run
  // of an auto containing "Set Launching" hung on that command forever.
  private static final int SIM_STARTING_FUEL = 5;
  private static final double SIM_SECONDS_PER_FUEL = 0.7;
  private static final double SIM_DETECT_PULSE_SECONDS = 0.15;
  private static final double SIM_RELOAD_IDLE_SECONDS = 2.0;
  private static final double SIM_DETECTED_DISTANCE_METERS = 0.04;
  private static final double SIM_CLEAR_DISTANCE_METERS = 0.30;
  private static final double SIM_SIGNAL_STRENGTH = 5000.0;

  private PearadoxTalonFX feeder;
  private TalonFXSimState feederSim;

  private DCMotorSim physicsSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(FeederConstants.FEEDER_MOTOR, 0.001, FEEDER_REDUCTION),
          FeederConstants.FEEDER_MOTOR);

  private VoltageOut feederControl;

  private boolean feederRunning = false;
  private int simFuelRemaining = SIM_STARTING_FUEL;
  private double feedTimer = 0.0;
  private double idleTimer = 0.0;
  private boolean simDetected = false;

  public FeederIOSim() {
    feeder =
        new PearadoxTalonFX(
            FeederConstants.FEEDER_CAN_ID,
            FeederConstants.FEEDER_MOTOR_CONFIG(),
            Compeartment.FEEDER);
    feederSim = feeder.getSimState();

    feederControl = new VoltageOut(0.0);
  }

  public void updateInputs(FeederIOInputs inputs) {
    updateSim();
    inputs.feederData = feeder.getData();
    inputs.canRangeIsDetected = simDetected;
    inputs.canRangeDistanceMeters =
        simDetected ? SIM_DETECTED_DISTANCE_METERS : SIM_CLEAR_DISTANCE_METERS;
    inputs.canRangeSignal = SIM_SIGNAL_STRENGTH;
  }

  public void runFeederVoltage(double voltage) {
    feederRunning = Math.abs(voltage) > 1.0;
    feeder.setControl(feederControl.withOutput(voltage));
  }

  public void updateSim() {
    feederSim.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);
    physicsSim.setInputVoltage(feederSim.getMotorVoltage());

    // Without this the motor model never advances and the sim feeder never moves at all.
    physicsSim.update(Constants.LOOP_PERIOD);

    feederSim.setRawRotorPosition(physicsSim.getAngularPositionRotations() * FEEDER_REDUCTION);
    // getAngularVelocityRPM is rotations per MINUTE; setRotorVelocity wants rotations per second.
    feederSim.setRotorVelocity(physicsSim.getAngularVelocityRPM() / 60.0 * FEEDER_REDUCTION);

    updateFuelSim();
  }

  /** Walks a hopper's worth of fuel past the sensor while the feeder runs, then reloads at rest. */
  private void updateFuelSim() {
    if (feederRunning && simFuelRemaining > 0) {
      idleTimer = 0.0;
      feedTimer += Constants.LOOP_PERIOD;

      simDetected = feedTimer < SIM_DETECT_PULSE_SECONDS;

      if (feedTimer >= SIM_SECONDS_PER_FUEL) {
        feedTimer = 0.0;
        simFuelRemaining--;
      }
      return;
    }

    simDetected = false;
    feedTimer = 0.0;

    // Standing still for a while stands in for going and intaking more.
    if (!feederRunning) {
      idleTimer += Constants.LOOP_PERIOD;
      if (idleTimer > SIM_RELOAD_IDLE_SECONDS) {
        simFuelRemaining = SIM_STARTING_FUEL;
      }
    }
  }
}
