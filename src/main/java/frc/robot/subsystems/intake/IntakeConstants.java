package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class IntakeConstants {

  public static enum IntakeState {
    STOWED,
    DEPLOYED,
    INTAKING,
    OUTTAKING
  }

  /**
   * record that maps the intake state to its respective angle and voltage
   *
   * @param angleRad the angle in radians
   * @param voltage the voltage in volts
   */
  public static record StateConfig(double angleDeg, double voltage) {
    public static final Map<IntakeState, StateConfig> INTAKE_STATE_MAP =
        Map.of(
            IntakeState.STOWED, new StateConfig(0, 0),
            IntakeState.DEPLOYED, new StateConfig(100, 0),
            IntakeState.INTAKING, new StateConfig(100, 3),
            IntakeState.OUTTAKING, new StateConfig(100, -3));
  }

  // public static record StateConfig(double angleDeg, double amps, double maxDuty) {
  //   public static final Map<IntakeState, StateConfig> INTAKE_STATE_MAP =
  //       Map.of(
  //           IntakeState.STOWED, new StateConfig(0, 0, 0),
  //           IntakeState.DEPLOYED, new StateConfig(95, 0, 0),
  //           IntakeState.INTAKING, new StateConfig(95, 45, 0.4),
  //           IntakeState.OUTTAKING, new StateConfig(90, -40, 0.4));
  // }

  // roller constants
  public static final int ROLLER_1_LEADER_ID = 31;
  public static final int ROLLER_2_FOLLOWER_ID = 32;

  public static final int ROLLER_SUPPLY_CURRENT_LIMIT = 50; // changed 3/17/26 for #119
  public static final int ROLLER_STATOR_CURRENT_LIMIT = 60;

  // pivot constants
  public static final int PIVOT_ID = 30;

  public static final int PIVOT_SUPPLY_CURRENT_LIMIT = 50; // changed 3/17/26 for #119
  public static final int PIVOT_STATOR_CURRENT_LIMIT = 60;

  public static final double GEARING = (44.0 / 12.0) * (60.0 / 16.0) * (44.0 / 14.0);
  public static final double LENGTH_METERS = Units.inchesToMeters(15.114);
  public static final double MASS_KG = 11.246;

  public static final double OP_ADJUST_INCREMENT_DEGREES = 2;

  // intake sim constants
  public static final double SIM_STARTING_ANGLE_RADS = Units.degreesToRadians(0);
  public static final double SIM_MIN_ANGLE_RADS = Double.NEGATIVE_INFINITY;
  public static final double SIM_MAX_ANGLE_RADS = Double.POSITIVE_INFINITY;

  // talonFX config for roller motor
  public static final TalonFXConfiguration ROLLER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs ROLLER_SLOT0_CONFIGS = ROLLER_CONFIG.Slot0;

  public static final TalonFXConfiguration getRollerConfigTalonFX() {

    ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    ROLLER_CONFIG.CurrentLimits.SupplyCurrentLimit = ROLLER_SUPPLY_CURRENT_LIMIT;
    ROLLER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    ROLLER_CONFIG.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_CURRENT_LIMIT;

    ROLLER_SLOT0_CONFIGS.kP = 0.1;
    ROLLER_SLOT0_CONFIGS.kI = 0.0;
    ROLLER_SLOT0_CONFIGS.kD = 0.0;
    // ROLLER_SLOT0_CONFIGS.kV = 0.12;

    ROLLER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ROLLER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    return ROLLER_CONFIG;
  }

  // talonFX config for pivot motor
  public static final TalonFXConfiguration PIVOT_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs PIVOT_SLOT0_CONFIGS = PIVOT_CONFIG.Slot0;

  public static final TalonFXConfiguration getPivotConfigTalonFX() {

    PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = PIVOT_SUPPLY_CURRENT_LIMIT;
    PIVOT_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    PIVOT_CONFIG.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT;

    PIVOT_SLOT0_CONFIGS.kP = 1.0;
    PIVOT_SLOT0_CONFIGS.kI = 0.0;
    PIVOT_SLOT0_CONFIGS.kD = 0.03;

    PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PIVOT_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // PIVOT_CONFIG.MotorOutput.PeakReverseDutyCycle = -0.5;
    // PIVOT_CONFIG.MotorOutput.PeakForwardDutyCycle = 0.5;

    return PIVOT_CONFIG;
  }
}
