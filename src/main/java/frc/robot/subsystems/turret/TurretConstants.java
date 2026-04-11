package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
  public static final int TURRET_ID = 20;
  public static final double TURRET_GEAR_RATIO = 45.0; // (90/10)(40/16)(32/16)
  public static final double TURRET_P_COEFFICIENT = 2 * Math.PI / TURRET_GEAR_RATIO;

  public static final int TURRET_SUPPLY_CURRENT_LIMIT = 50; // changed 3/17/26 for #119
  public static final int TURRET_STATOR_CURRENT_LIMIT = 60;

  // changed the field relative offset instead, but turret now zeroes facing 90 CW from intake side
  public static final double TURRET_STARTING_ANGLE = Units.degreesToRadians(0);

  public static final double TURRET_MIN_ANGLE = Units.degreesToRadians(-275);   // absolute min is prob near -540 
  public static final double TURRET_MAX_ANGLE = Units.degreesToRadians(155); // absolute max is near 180

  public static final double SAFETY_LIMIT = Units.degreesToRadians(5);
  public static final double TURRET_SAFE_MIN = TURRET_MIN_ANGLE + SAFETY_LIMIT;
  public static final double TURRET_SAFE_MAX = TURRET_MAX_ANGLE - SAFETY_LIMIT;

  // only used in sim
  public static final double TURRET_MASS = Units.lbsToKilograms(9);
  public static final double TURRET_CG_RADIUS = Units.inchesToMeters(1.67);

  // mass ≈ 9 lb, Lzz ≈ 218 in^2 lb for the turret in cad
  // center of mass ≈ 1.67 in from its axis of rotation
  // I = I_cm + md^2 = 218 + 9(1.67)^2 = 243 in^2 lb ≈ 0.071 kg m^2
  // irl the turret weighed 4.56 kg (>10 lbs), so rounded up to 0.08
  public static final double TURRET_MOI = 0.08 * 2.5;

  public static final DCMotor TURRET_MOTOR = DCMotor.getKrakenX60Foc(1);

  // feedforward term: adds a voltage to the turret as the chassis rotates
  //   public static final double K_OMEGA = 0.2; // volts per radian per second

  // only apply feedforward if the turret is within 45 degrees of its setpoint
  // this prevents FF from working against the turret while it "wraps"
  // its angle (changing the setpoint by +/-360 and thus error)
  public static final double FF_ERROR_THRESHOLD = Units.degreesToRadians(90);

  // only apply feedforward if the drivetrain is rotating at a reasonable speed
  // note: this may not be necessary
  public static final double FF_CHASSIS_ROT_VELOCITY_LIMIT = 1.5 * Math.PI; // rad/s

  public static final int TURRET_CANCODER_ID = 26;
  // a wild robonauts appeared!
  public static final double TURRET_CANCODER_OFFSET_ROTS = -0.1184;

  // now geared 1:1 with turret, (90/10)(15/45)(15/45)
  public static final double TURRET_TO_CANCODER_RATIO = 1.0; 

  public static final TalonFXConfiguration getTurretConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TURRET_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_CURRENT_LIMIT;

    config.MotionMagic.MotionMagicCruiseVelocity = 85;
    config.MotionMagic.MotionMagicAcceleration = 450;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 6.7;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;

    return config;
  }
}
