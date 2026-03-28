// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;

/** Add your docs here. */
public class FeederIOSim implements FeederIO {

  private PearadoxTalonFX feeder;
  private TalonFXSimState feederSim;

  private DCMotorSim physicsSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              FeederConstants.FEEDER_MOTOR, 0.001, FeederConstants.FEEDER_GEARING),
          FeederConstants.FEEDER_MOTOR);

  private VoltageOut feederControl;

  public FeederIOSim() {
    feeder =
        new PearadoxTalonFX(
            FeederConstants.FEEDER_CAN_ID,
            FeederConstants.FEEDER_MOTOR_CONFIG(),
            Compeartment.FEEDER);
    feederSim = feeder.getSimState();

    feederControl = new VoltageOut(0.0);
  }

  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    updateSim();
    inputs.feederData = feeder.getData();
  }

  public void runFeederVoltage(double voltage) {
    feeder.setControl(feederControl.withOutput(voltage));
  }

  public void updateSim() {
    feederSim.setSupplyVoltage(12);
    physicsSim.setInputVoltage(feederSim.getMotorVoltage());

    feederSim.setRawRotorPosition(
        physicsSim.getAngularPositionRotations() * FeederConstants.FEEDER_GEARING);
    feederSim.setRotorVelocity(physicsSim.getAngularVelocityRPM() * FeederConstants.FEEDER_GEARING);
  }
}
