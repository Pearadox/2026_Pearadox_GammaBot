// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.lib.drivers.PearadoxTalonFX;

/** Add your docs here. */
public abstract class FeederIOTalonFX implements FeederIO {

  private PearadoxTalonFX feeder;

  private VoltageOut feederControl;

  // private CANrange canRange;
  // private CANrangeConfiguration CAN_RANGE_CONFIG = FeederConstants.createCANrangeConfig();

  public FeederIOTalonFX() {
    feeder =
        new PearadoxTalonFX(FeederConstants.FEEDER_CAN_ID, FeederConstants.FEEDER_MOTOR_CONFIG());
    feederControl = new VoltageOut(0.0);

    // canRange = new CANrange(FeederConstants.CANRANGE_CAN_ID);

    // canRange.getConfigurator().apply(CAN_RANGE_CONFIG);
  }

  @Override
  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    inputs.feederData = feeder.getData();

    // StatusSignal<Distance> dist = canRange.getDistance();
    // if (dist.getStatus().isOK()) {
    //   inputs.canRangeDistanceMeters = canRange.getDistance().getValueAsDouble();
    // } else {
    //   Logger.recordOutput(
    //       "Feeder/CanRange/StatusCode CanRange Distance Error", dist.getStatus().getName());
    // }

    // StatusSignal<Boolean> isDetected = canRange.getIsDetected();
    // if (isDetected.getStatus().isOK()) {
    //   inputs.canRangeIsDetected = canRange.getIsDetected().getValue();
    // } else {
    //   Logger.recordOutput(
    //       "Feeder/CanRange/StatusCode CanRange IsDetected Error", isDetected.getStatus().getName());
    // }

    // StatusSignal<Double> signal = canRange.getSignalStrength();
    // if (signal.getStatus().isOK()) {
    //   inputs.canRangeSignal = canRange.getSignalStrength().getValueAsDouble();
    // } else {
    //   Logger.recordOutput(
    //       "Feeder/CanRange/StatusCode CanRange Signal Error", signal.getStatus().getName());
    // }
  }

  @Override
  public void runFeederVoltage(double voltage) {
    feeder.setControl(feederControl.withOutput(voltage));
  }
}
