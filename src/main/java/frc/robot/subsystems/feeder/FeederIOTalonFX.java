// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public abstract class FeederIOTalonFX implements FeederIO {

  private PearadoxTalonFX feeder;

  private VoltageOut feederControl;

  private CANrange canRange;
  private CANrangeConfiguration CAN_RANGE_CONFIG = FeederConstants.createCANrangeConfig();

  public FeederIOTalonFX() {
    feeder =
        new PearadoxTalonFX(
            FeederConstants.FEEDER_CAN_ID,
            FeederConstants.FEEDER_MOTOR_CONFIG(),
            Compeartment.FEEDER);
    feederControl = new VoltageOut(0.0);

    canRange = new CANrange(FeederConstants.CANRANGE_CAN_ID);

    // try to get the config 5 times with 0.25 sec in between each try
    PhoenixUtil.tryUntilOk(5, () -> canRange.getConfigurator().apply(CAN_RANGE_CONFIG, 0.25));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederData = feeder.getData();

    StatusSignal<Distance> dist = canRange.getDistance();
    if (dist.getStatus().isOK()) {
      inputs.canRangeDistanceMeters = canRange.getDistance().getValueAsDouble();
    } else {
      Logger.recordOutput(
          "Feeder/CanRange/StatusCode CanRange Distance Error", dist.getStatus().getName());
    }

    StatusSignal<Boolean> isDetected = canRange.getIsDetected();
    if (isDetected.getStatus().isOK()) {
      inputs.canRangeIsDetected = canRange.getIsDetected().getValue();
    } else {
      Logger.recordOutput(
          "Feeder/CanRange/StatusCode CanRange IsDetected Error", isDetected.getStatus().getName());
    }

    StatusSignal<Double> signal = canRange.getSignalStrength();
    if (signal.getStatus().isOK()) {
      inputs.canRangeSignal = canRange.getSignalStrength().getValueAsDouble();
    } else {
      Logger.recordOutput(
          "Feeder/CanRange/StatusCode CanRange Signal Error", signal.getStatus().getName());
    }
  }

  @Override
  public void runFeederVoltage(double voltage) {
    feeder.setControl(feederControl.withOutput(voltage));
  }
}
