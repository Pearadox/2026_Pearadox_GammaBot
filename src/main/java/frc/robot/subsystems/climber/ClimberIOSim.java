package frc.robot.subsystems.climber;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.drive.Drive;

public class ClimberIOSim extends ClimberIOTalonFX {
  private ElevatorSim physicsSim =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          ClimberConstants.CLIMBER_GEARING,
          Drive.ROBOT_MASS_KG,
          ClimberConstants.CLIMBER_DRUM_RADIUS_METERS,
          ClimberConstants.CLIMBER_MIN_HEIGHT_METERS,
          ClimberConstants.CLIMBER_MAX_HEIGHT_METERS,
          true,
          ClimberConstants.CLIMBER_MIN_HEIGHT_METERS);
  private TalonFXSimState climberSimState;

  public ClimberIOSim() {
    climberSimState = climber.getSimState();
  }

  public void updateInputs(ClimberIOInputs inputs) {
    climberSimState.setSupplyVoltage(12);
    physicsSim.setInputVoltage(climberSimState.getMotorVoltage());
    climberSimState.setRawRotorPosition(
        physicsSim.getPositionMeters() / ClimberConstants.CLIMBER_DRUM_CIRCUMFERENCE);
    climberSimState.setRotorVelocity(
        physicsSim.getVelocityMetersPerSecond() / ClimberConstants.CLIMBER_DRUM_CIRCUMFERENCE);
    physicsSim.update(0.02);
  }
}
