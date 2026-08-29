package frc.robot.subsystems.turret;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class TurretIOSim extends TurretIOTalonFX {
  private final SingleJointedArmSim physicsSim =
      new SingleJointedArmSim(
          TurretConstants.TURRET_MOTOR,
          TurretConstants.TURRET_GEAR_RATIO,
          TurretConstants.TURRET_MOI,
          TurretConstants.TURRET_CG_RADIUS, // unused
          TurretConstants.TURRET_MIN_ANGLE,
          TurretConstants.TURRET_MAX_ANGLE,
          false,
          TurretConstants.TURRET_STARTING_ANGLE);

  private final TalonFXSimState turretSimState;

  public TurretIOSim() {
    turretSimState = turretMotor.getSimState();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    super.updateInputs(inputs);

    turretSimState.setSupplyVoltage(Constants.NOMINAL_VOLTAGE);

    physicsSim.setInputVoltage(turretSimState.getMotorVoltage());
    physicsSim.update(Constants.LOOP_PERIOD);

    turretSimState.setRawRotorPosition(
        physicsSim.getAngleRads() / TurretConstants.TURRET_P_COEFFICIENT);
    turretSimState.setRotorVelocity(
        physicsSim.getVelocityRadPerSec() / TurretConstants.TURRET_P_COEFFICIENT);

    // The real cancoder reads opposite the motor, which is why requestZero negates it. Writing the
    // sim cancoder with the same sign as the simulated motor meant zeroing in sim set the motor
    // position to the negative of the truth, so the sim turret aimed mirrored afterwards and
    // nothing validated in sim transferred to the robot. Use the same constant both sides.
    inputs.cancoderPosition =
        TurretConstants.CANCODER_SIGN
            * Units.radiansToRotations(physicsSim.getAngleRads())
            / TurretConstants.TURRET_TO_CANCODER_RATIO;
    inputs.cancoderConnected = true;
  }
}
