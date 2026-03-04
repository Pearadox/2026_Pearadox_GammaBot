package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.spindexer.SpindexerConstants.*;
import frc.robot.util.SmarterDashboard;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private SpindexerState spindexerState = SpindexerState.RUNNING;
  public static double adjust = 0;

  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Spindexer", inputs);
    SmarterDashboard.putNumber("Spindexer/Voltage", inputs.spindexerMotorData.appliedVolts());
    SmarterDashboard.putNumber("Spindexer/Velocity", inputs.spindexerMotorData.velocity());
    SmarterDashboard.putNumber(
        "Spindexer/StatorCurrent", inputs.spindexerMotorData.statorCurrent());
    SmarterDashboard.putNumber(
        "Spindexer/SupplyCurrent", inputs.spindexerMotorData.supplyCurrent());

    // io.runSpindexerVoltage(StateConfig.SPINDEXER_STATE_MAP.get(spindexerState).voltage());
  }

  public void setStopped() {
    spindexerState = SpindexerState.STOPPED;
  }

  public void setRunning() {
    spindexerState = SpindexerState.RUNNING;
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
