package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.spindexer.SpindexerConstants.*;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  @AutoLogOutput private SpindexerState spindexerState = SpindexerState.STOPPED;
  public static double adjust = 0;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  private final LoggedTunableNumber spindexerCurrentAmps =
      new LoggedTunableNumber("Spindexer/Current-Amps", 70.0); // try lowering back to 50
  private final LoggedTunableNumber spindexerMaxDutyCycle =
      new LoggedTunableNumber("Spindexer/Max-Duty-Cycle", 0.9);

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Spindexer", inputs);
    Logger.recordOutput("Spindexer/Voltage", inputs.spindexerMotorData.appliedVolts());
    Logger.recordOutput("Spindexer/Velocity", inputs.spindexerMotorData.velocity());
    Logger.recordOutput("Spindexer/StatorCurrent", inputs.spindexerMotorData.statorCurrent());
    Logger.recordOutput("Spindexer/SupplyCurrent", inputs.spindexerMotorData.supplyCurrent());

    if (spindexerState.equals(SpindexerState.RUNNING)) {
      io.runSpindexerTorqueCurrent(spindexerCurrentAmps.get(), spindexerMaxDutyCycle.get());

    } else if (spindexerState.equals(SpindexerState.REVERSE)) {
      io.runSpindexerTorqueCurrent(-spindexerCurrentAmps.get(), spindexerMaxDutyCycle.get());
    } else {
      io.runSpindexerTorqueCurrent(0, spindexerMaxDutyCycle.get());
    }
  }

  public void setStopped() {
    spindexerState = SpindexerState.STOPPED;
  }

  public void setRunning() {
    spindexerState = SpindexerState.RUNNING;
  }

  public void setReverse() {
    spindexerState = SpindexerState.REVERSE;
  }

  public void adjustVoltage(double adjustBy) {
    adjust += adjustBy;
  }

  public double getAdjust() {
    return adjust;
  }

  public void resetAdjust() {
    adjust = 0;
  }
}
