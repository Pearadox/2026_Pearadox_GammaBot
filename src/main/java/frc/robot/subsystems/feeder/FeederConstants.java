// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Map;

/** Add your docs here. */
public class FeederConstants {

  public static enum FeederState {
    STOPPED,
    RUNNING
  }

  public static record StateConfig(double voltage) {
    public static final Map<FeederState, StateConfig> FEEDER_STATE_MAP =
        Map.of(
            FeederState.STOPPED, new StateConfig(0),
            FeederState.RUNNING, new StateConfig(-12));
  }

  // feeder constants
  public static final int FEEDER_CAN_ID = 41;

  public static final int FEEDER_SUPPLY_CURRENT_LIMIT = 50; // changed 3/17/26 for #119
  public static final int FEEDER_STATOR_CURRENT_LIMIT = 60;

  public static final double FEEDER_GEARING =
      12.0 / 48.0; // ratio of teeth on motor to teeth on pulley (originally 11/24 -> (12/20))

  public static final DCMotor FEEDER_MOTOR = DCMotor.getKrakenX60(1);
  public static final double FEEDER_ACTIVE_VOLTAGE = -5.414;

  public static final TalonFXConfiguration FEEDER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs FEEDER_SLOT0_CONFIGS = FEEDER_CONFIG.Slot0;

  public static final TalonFXConfiguration FEEDER_MOTOR_CONFIG() {
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.StatorCurrentLimit = FEEDER_STATOR_CURRENT_LIMIT;

    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FEEDER_CONFIG.CurrentLimits.SupplyCurrentLimit = FEEDER_SUPPLY_CURRENT_LIMIT;

    FEEDER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FEEDER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    FEEDER_SLOT0_CONFIGS.kP = 0.1;
    FEEDER_SLOT0_CONFIGS.kI = 0.0;
    FEEDER_SLOT0_CONFIGS.kD = 0.0;
    FEEDER_SLOT0_CONFIGS.kV = 0.0;

    return FEEDER_CONFIG;
  }

  // canRange constants
  public static final int CANRANGE_CAN_ID = 25;
  public static final double IS_HOPPER_EMPTY_BUFFER_TIME = 3.0;

  public static CANrangeConfiguration createCANrangeConfig() {
    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    canRangeConfig.ProximityParams.ProximityThreshold = 0.09;
    canRangeConfig.ProximityParams.ProximityHysteresis = 0.01;
    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    canRangeConfig.FovParams.FOVCenterX = 3;
    canRangeConfig.FovParams.FOVCenterY = -6.5;
    canRangeConfig.FovParams.FOVRangeX = 10;
    canRangeConfig.FovParams.FOVRangeY = 10;
    return canRangeConfig;
  }
}
