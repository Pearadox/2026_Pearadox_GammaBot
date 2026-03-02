package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import java.util.Map;

public class SpindexerConstants {
  public static enum SpindexerState {
    STOPPED,
    RUNNING
  }

  public static record StateConfig(double voltage) {
    public static final Map<SpindexerState, StateConfig> SPINDEXER_STATE_MAP =
        Map.of(
            SpindexerState.STOPPED, new StateConfig(0),
            SpindexerState.RUNNING, new StateConfig(8));
  }

  public static final TalonFXConfiguration SPINDEXER_CONFIG = new TalonFXConfiguration();
  public static final Slot0Configs SPINDEXER_SLOT0_CONFIGS = SPINDEXER_CONFIG.Slot0;

  public static final int SPINDEXER_MOTOR_ID = 42;
  public static final int LAUNCHER_CURRENT_LIMIT = 20;
  public static final int SPINDEXER_CURRENT_LIMIT = 40;

  public static final double SPINDEXER_GEARING = 1.0 / 15.0;
  public static final double SPINDEXER_RADIUS_METERS = Units.inchesToMeters(4);
  public static final double SPINDEXER_MASS_KG = Units.lbsToKilograms(5);
  public static final double SPINDEXER_MOI =
      0.5 * SPINDEXER_MASS_KG * Math.pow(SPINDEXER_RADIUS_METERS, 2);

  public static final TalonFXConfiguration spindexerConfig() {
    SPINDEXER_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    SPINDEXER_CONFIG.CurrentLimits.StatorCurrentLimit = SPINDEXER_CURRENT_LIMIT;

    SPINDEXER_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    SPINDEXER_CONFIG.CurrentLimits.SupplyCurrentLimit = SPINDEXER_CURRENT_LIMIT;

    SPINDEXER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    SPINDEXER_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    SPINDEXER_SLOT0_CONFIGS.kP = 0.1;
    SPINDEXER_SLOT0_CONFIGS.kI = 0.0;
    SPINDEXER_SLOT0_CONFIGS.kD = 0.0;
    SPINDEXER_SLOT0_CONFIGS.kS = 0.0;
    SPINDEXER_SLOT0_CONFIGS.kV = 0.0;

    return SPINDEXER_CONFIG;
  }
}
