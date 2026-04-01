package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.util.EnergyTracker.Compeartment;

public abstract class SpindexerIOTalonFX implements SpindexerIO {
  protected PearadoxTalonFX spindexer;
  private TalonFXConfiguration spindexerConfig;
  private TorqueCurrentFOC spindexerControl;

  public SpindexerIOTalonFX() {
    spindexerConfig = SpindexerConstants.spindexerConfig();

    spindexer =
        new PearadoxTalonFX(
            SpindexerConstants.SPINDEXER_MOTOR_ID, spindexerConfig, Compeartment.SPINDEXER);

    spindexerControl = new TorqueCurrentFOC(0);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.spindexerMotorData = spindexer.getData();
  }

  @Override
  public void runSpindexerTorqueCurrent(double amps, double maxDutyCycle) {
    spindexer.setControl(spindexerControl.withOutput(amps).withMaxAbsDutyCycle(maxDutyCycle));
  }
}
