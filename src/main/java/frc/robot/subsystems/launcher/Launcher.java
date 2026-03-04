// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launcher.LauncherConstants.LauncherState;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private final LauncherIO io;

  public final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  private static LauncherState launcherState = LauncherState.SCORING;
  @Getter @Setter private double adjust = 0.0;

  public Launcher(LauncherIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    // io.runLauncherVelocity((launcherState == LauncherState.SCORING ? 60 : 0));
    io.setHoodAngleRads(launcherState.getHoodAngleRads());
    Logger.recordOutput("Launcher/State", getState());

    LauncherVisualizer.getInstance()
        .updateFlywheelPositionDeg(Units.rotationsToDegrees(inputs.launcher1Data.position()));
    LauncherVisualizer.getInstance()
        .updateHoodPositionDeg(
            Units.rotationsToDegrees(
                LauncherConstants.angularPositiontoRotations(inputs.hoodServo1Position)));

    Logger.recordOutput("Hood/Desired-Angle", launcherState.getHoodAngleRads());
    Logger.recordOutput("Hood/Servo-Position", inputs.hoodServo1Position);
    Logger.recordOutput(
        "Hood/Current-Angle",
        LauncherConstants.angularPositiontoRotations(inputs.hoodServo1Position)
            / LauncherConstants.HOOD_GEARING); // 5 because 1.0 position -> 5 rotations

    // setVelocity(67);
  }

  /** velocity will be calculated from aim assist command factory */
  public void setVelocity(double velocityRPS) {
    io.runLauncherVelocity(velocityRPS + (launcherState == LauncherState.OFF ? 0.0 : adjust));
  }

  public void setOff() {
    launcherState = LauncherState.OFF;
  }

  public void setManual() {
    launcherState = LauncherState.MANUAL;
  }

  public void setScoring() {
    launcherState = LauncherState.SCORING;
  }

  public void setPassing() {
    launcherState = LauncherState.PASSING;
  }

  public static LauncherState getState() {
    return launcherState;
  }
}
