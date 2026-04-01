// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/** Constants for the launcher */
public class LauncherConstants {
  public static enum LauncherState {
    OFF(Units.degreesToRadians(60)),
    MANUAL(Units.degreesToRadians(40)),
    IDLE(Units.degreesToRadians(40)),
    SELF_DIRECTING(Units.degreesToRadians(40));
    // PASSING(Units.degreesToRadians(25)); // TODO: find proper hood angles

    @Getter private final double hoodAngleRads;

    private LauncherState(double angle) {
      hoodAngleRads = angle;
    }
  }

  public static final int LAUNCHER_1_CAN_ID = 22;
  public static final int LAUNCHER_2_CAN_ID = 21;

  public static final double SHOOTER_VELOCITY_DEADBAND = 3.0; // in rps
  public static final double SHOOTER_MAX_VELOCITY = 100.0; // in rps

  public static final int LAUNCHER_SUPPLY_CURRENT_LIMIT = 50; // changed 3/17/26 for #119
  public static final int LAUNCHER_STATOR_CURRENT_LIMIT = 60;

  public static final double LAUNCHER_GEARING = 1.0;
  public static final double DEFAULT_VELOCITY_SETPOINT_RPS = 51.3;

  public static final TalonFXConfiguration LAUNCHER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs LAUNCHER_CONFIG_SLOT0 = LAUNCHER_CONFIG.Slot0;

  public static final TalonFXConfiguration LAUNCHER_MOTOR_CONFIG() {
    LAUNCHER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    LAUNCHER_CONFIG.CurrentLimits.StatorCurrentLimit = LAUNCHER_STATOR_CURRENT_LIMIT;

    LAUNCHER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    LAUNCHER_CONFIG.CurrentLimits.SupplyCurrentLimit = LAUNCHER_SUPPLY_CURRENT_LIMIT;

    LAUNCHER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    LAUNCHER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    LAUNCHER_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = 40.0; // added for bang-bang control
    LAUNCHER_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = 0.0; // added for bang-bang control

    LAUNCHER_CONFIG.Voltage.PeakForwardVoltage = 12;
    LAUNCHER_CONFIG.Voltage.PeakReverseVoltage = -3;

    LAUNCHER_CONFIG_SLOT0.kP = 99999.0; // 0.8
    LAUNCHER_CONFIG_SLOT0.kI = 0.0;
    LAUNCHER_CONFIG_SLOT0.kD = 0.0;
    LAUNCHER_CONFIG_SLOT0.kS = 0.19;
    LAUNCHER_CONFIG_SLOT0.kV = 0.12;
    return LAUNCHER_CONFIG;
  }

  public static final int HOOD_ID = 0; // TODO: get

  public static final double HOOD_STATOR_CURRENT = 0.0; // TODO: tune
  public static final double HOOD_SUPPLY_CURRENT = 0.0; // TODO: tune

  public static final double HOOD_GEARING = 0.0; // TODO: tune

  public static final double HOOD_MIN_ANGLE_RADS = Units.degreesToRadians(10);
  public static final double HOOD_MAX_ANGLE_RADS = Units.degreesToRadians(40);

  public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs HOOD_CONFIG_SLOT0 = HOOD_CONFIG.Slot0;

  public static final TalonFXConfiguration HOOD_CONFIG() {
    HOOD_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    HOOD_CONFIG.CurrentLimits.StatorCurrentLimit = HOOD_STATOR_CURRENT;

    HOOD_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    HOOD_CONFIG.CurrentLimits.SupplyCurrentLimit = HOOD_SUPPLY_CURRENT;

    HOOD_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    HOOD_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: find

    HOOD_CONFIG.Voltage.PeakForwardVoltage = 6;
    HOOD_CONFIG.Voltage.PeakReverseVoltage = 6;

    HOOD_CONFIG_SLOT0.kP = 0.1; // TODO: tune
    HOOD_CONFIG_SLOT0.kI = 0.0; // TODO: tune
    HOOD_CONFIG_SLOT0.kD = 0.0; // TODO: tune
    HOOD_CONFIG_SLOT0.kV = 0.0; // TODO: tune

    return HOOD_CONFIG;
  }

  // SIM
  public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60(1);
  public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final double ROLLER_MASS_KG = Units.lbsToKilograms(0.7); // TODO: get weight
  public static final double ROLLER_CIRCUMFERENCE_METERS = Units.inchesToMeters(4.0 * Math.PI);
  public static final double LAUNCHER_HEIGHT_METERS =
      Units.inchesToMeters(22.5); // TODO: double check
  public static final double LAUNCHER_ROLLER_MOI = 0.003;
  // 0.5 * ROLLER_MASS_KG * Math.pow(ROLLER_RADIUS_METERS, 2);
  public static final int ROLLER_SEGMENT_COUNT = 60;
  public static final int SIM_LINE_WIDTH = 5;

  public static final DCMotor HOOD_MOTOR = DCMotor.getKrakenX44(1);
  public static final double HOOD_LENGTH_METERS =
      Units.inchesToMeters(10); // TODO: get better values
  public static final double HOOD_MASS_KG = Units.lbsToKilograms(3); // TODO: get better values
}
