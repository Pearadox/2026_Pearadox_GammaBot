// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/** Constants for the launcher */
public class LauncherConstants {
  public static enum LauncherState {
    OFF(Units.degreesToRadians(60)),
    MANUAL(Units.degreesToRadians(40)),
    SCORING(Units.degreesToRadians(40)),
    PASSING(Units.degreesToRadians(25)); // TODO: find proper hood angles

    @Getter private final double hoodAngleRads;

    private LauncherState(double angle) {
      hoodAngleRads = angle;
    }
  }

  public static final int LAUNCHER_1_CAN_ID = 21; // TODO: double check
  public static final int LAUNCHER_2_CAN_ID = 22; // TODO: double check

  public static final int LAUNCHER_SUPPLY_CURRENT_LIMIT = 40; // changed to match the breaker
  public static final int LAUNCHER_STATOR_CURRENT_LIMIT =
      35; // originally 40 on 3/3/2026 during testing

  public static final double LAUNCHER_GEARING = 1.0;
  public static final double DEFAULT_VELOCITY_SETPOINT_RPS = 60.0;

  public static final TalonFXConfiguration LAUNCHER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs LAUNCHER_CONFIG_SLOT0 = LAUNCHER_CONFIG.Slot0;

  public static final TalonFXConfiguration LAUNCHER_MOTOR_CONFIG() {
    LAUNCHER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    LAUNCHER_CONFIG.CurrentLimits.StatorCurrentLimit = LAUNCHER_STATOR_CURRENT_LIMIT;

    LAUNCHER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    LAUNCHER_CONFIG.CurrentLimits.SupplyCurrentLimit = LAUNCHER_SUPPLY_CURRENT_LIMIT;

    LAUNCHER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    LAUNCHER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    LAUNCHER_CONFIG_SLOT0.kP = 0.1;
    LAUNCHER_CONFIG_SLOT0.kI = 0.0;
    LAUNCHER_CONFIG_SLOT0.kD = 0.0;
    LAUNCHER_CONFIG_SLOT0.kS = 0.19;
    LAUNCHER_CONFIG_SLOT0.kV = 0.1;
    return LAUNCHER_CONFIG;
  }

  // SERVO
  public static final int HOOD_SERVO_HUB_CAN_ID = 23; // TODO: set
  public static final ChannelId HOOD_1_ID = ChannelId.kChannelId0; // TODO: set
  public static final ChannelId HOOD_2_ID = ChannelId.kChannelId5; // TODO: set

  public static final double HOOD_GEARING = 25 / 12; // TODO: double check

  public static final double HOOD_MAX_ANGLE_RADS = Units.degreesToRadians(60.0);
  public static final double HOOD_MIN_ANGLE_RADS = Units.degreesToRadians(20.0);

  public static final int SERVO_MAX_PULSE_WIDTH = 2500;
  public static final int SERVO_MIN_PULSE_WIDTH = 500;

  public static final double SERVO_POSITION_TO_ROTATIONS_CONVERSION = 4.25;
  public static final double SERVO_POSITION_TO_PW_CONVERSION = 2000;
  public static final double SERVO_ROTATIONS_TO_PW_CONVERSION =
      SERVO_POSITION_TO_PW_CONVERSION / SERVO_POSITION_TO_ROTATIONS_CONVERSION;

  public static final double pulseWidthtoAngularPosition(int pulseWidth) {
    return (pulseWidth - SERVO_MIN_PULSE_WIDTH) / SERVO_POSITION_TO_PW_CONVERSION;
  }

  public static final double pulseWidthtoRotations(int pulseWidth) {
    return (pulseWidth - SERVO_MIN_PULSE_WIDTH) / SERVO_ROTATIONS_TO_PW_CONVERSION;
  }

  public static final int angularPositiontoPulseWidth(double angularPosition) {
    return (int) (angularPosition * SERVO_POSITION_TO_PW_CONVERSION)
        + LauncherConstants.SERVO_MIN_PULSE_WIDTH;
  }

  public static final double angularPositiontoRotations(double angularPosition) {
    return angularPosition * SERVO_POSITION_TO_ROTATIONS_CONVERSION;
  }

  public static final int rotationstoPulseWidth(double rotations) {
    return (int) (rotations * SERVO_ROTATIONS_TO_PW_CONVERSION) + SERVO_MIN_PULSE_WIDTH;
  }

  public static final double rotationstoAngularPosition(double rotations) {
    return rotations / SERVO_POSITION_TO_ROTATIONS_CONVERSION;
  }

  // SIM
  public static final DCMotor ROLLER_MOTOR = DCMotor.getKrakenX60(1);
  public static final double ROLLER_RADIUS_METERS = Units.inchesToMeters(2.0);
  public static final double ROLLER_MASS_KG = Units.lbsToKilograms(0.7); // TODO: get weight
  public static final double ROLLER_CIRCUMFERENCE_METERS = Units.inchesToMeters(4.0 * Math.PI);
  public static final double LAUNCHER_HEIGHT_METERS =
      Units.inchesToMeters(22.5); // TODO: double check
  public static final double LAUNCHER_ROLLER_MOI =
      0.5 * ROLLER_MASS_KG * Math.pow(ROLLER_RADIUS_METERS, 2);
  public static final int ROLLER_SEGMENT_COUNT = 60;
  public static final int SIM_LINE_WIDTH = 5;
}
