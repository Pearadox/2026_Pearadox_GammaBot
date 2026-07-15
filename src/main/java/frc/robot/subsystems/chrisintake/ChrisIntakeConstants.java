package frc.robot.subsystems.chrisintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ChrisIntakeConstants {
    public static final int LEFT_PIVOT_INTAKE_MOTOR_ID = 31;
    public static final int RIGHT_PIVOT_INTAKE_MOTOR_ID = 32;
    public static final int LEFT_ROLLER_INTAKE_MOTOR_ID = 33;
    public static final int RIGHT_ROLLER_INTAKE_MOTOR_ID = 34;

    public static final TalonFXConfiguration INTAKE_CONFIG = new TalonFXConfiguration();

        public static final TalonFXConfiguration getIntakeConfigTalonFX() {
            INTAKE_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;

            return INTAKE_CONFIG;
        }
}
