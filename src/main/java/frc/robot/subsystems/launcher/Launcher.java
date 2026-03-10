// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoringMode;
import frc.robot.subsystems.launcher.LauncherConstants.LauncherState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  /** Creates a new Launcher. */
  private final LauncherIO io;

  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  @AutoLogOutput @Getter @Setter private static LauncherState launcherState = LauncherState.MANUAL;

  private final LoggedTunableNumber tunableffAmps = new LoggedTunableNumber("Launcher/ffamps", 0);

  private double rpsAdjust = 0.0;

  public void adjustRPSBy(double adj) {
    rpsAdjust += adj;
  }

  private final LoggedTunableNumber fullyManualInitialVelocity =
      new LoggedTunableNumber(
          "Launcher/Default velocity", LauncherConstants.DEFAULT_VELOCITY_SETPOINT_RPS);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Launcher/kP", 99999);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Launcher/kD", 0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Launcher/kS", 0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Launcher/kV", 0);
  private final LoggedTunableNumber statorCurrentLimit =
      new LoggedTunableNumber(
          "Launcher/Stator Current Limit", LauncherConstants.LAUNCHER_STATOR_CURRENT_LIMIT);
  private final LoggedTunableNumber supplyCurrentLimit =
      new LoggedTunableNumber(
          "Launcher/Supply Current Limit", LauncherConstants.LAUNCHER_SUPPLY_CURRENT_LIMIT);

  public Launcher(LauncherIO io) {
    this.io = io;

    io.setPIDFF(kP.get(), kD.get(), kS.get(), kV.get());
    io.setCurrentLimits(statorCurrentLimit.get(), supplyCurrentLimit.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    // io.runLauncherVelocity((launcherState == LauncherState.SCORING ? 60 : 0));
    // io.setHoodAngleRads(launcherState.getHoodAngleRads());
    Logger.processInputs("Launcher", inputs);

    Logger.recordOutput("Launcher/State", getState());

    double desiredVelocity = 0;
    ScoringMode currentScoringMode = RobotContainer.getScoringMode();
    if (currentScoringMode == ScoringMode.FULLY_AUTO) {
      desiredVelocity = RobotContainer.getShotSolution().getShooterSpeedRPS();

    } else if ((currentScoringMode == ScoringMode.PARTIAL_AUTO
            || currentScoringMode == ScoringMode.PASSING)
        && RobotContainer.drivercontroller.rightBumper().getAsBoolean()) {

      desiredVelocity =
          RobotContainer.getShouldSOTM()
              ? RobotContainer.getShotSolution().getShooterSpeedRPS()
              : fullyManualInitialVelocity.get();

    } else if (currentScoringMode == ScoringMode.FULLY_MANUAL) {
      desiredVelocity = fullyManualInitialVelocity.get();
    }

    double launcherRPS = Math.abs(desiredVelocity + rpsAdjust);

    if (DriverStation.isAutonomousEnabled() && launcherState != LauncherState.SCORING)
      launcherRPS = 0;

    if (launcherState != LauncherState.OFF
        && Math.abs(launcherRPS) > LauncherConstants.SHOOTER_VELOCITY_DEADBAND) {
      setVelocity(launcherRPS, tunableffAmps.get());
    } else {
      turnLauncherOff();
    }

    LauncherVisualizer.getInstance()
        .updateFlywheelPositionDeg(Units.rotationsToDegrees(inputs.launcher1Data.position()));
    LauncherVisualizer.getInstance()
        .updateHoodPositionDeg(
            Units.rotationsToDegrees(
                LauncherConstants.angularPositiontoRotations(inputs.hoodServo1Position)));

    Logger.recordOutput("Launcher/adjust", rpsAdjust);
    Logger.recordOutput("Debug/ScoringMode", currentScoringMode);
    Logger.recordOutput("Debug/desiredLauncherRPS", launcherRPS);
    Logger.recordOutput("Debug/shouldSOTM", RobotContainer.getShouldSOTM());
    Logger.recordOutput("Debug/ScoringMode", currentScoringMode);
    Logger.recordOutput("Debug/getLauncherVelocity", getLauncherVelocity());

    // Logger.recordOutput("Hood/Desired-Angle", launcherState.getHoodAngleRads());
    // Logger.recordOutput("Hood/Servo-Position", inputs.hoodServo1Position);
    // Logger.recordOutput(
    //     "Hood/Current-Angle",
    //     LauncherConstants.angularPositiontoRotations(inputs.hoodServo1Position)
    //         / LauncherConstants.HOOD_GEARING); // 5 because 1.0 position -> 5 rotations

    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())) {
      io.setPIDFF(kP.get(), kD.get(), kS.get(), kV.get());
    }
    if (statorCurrentLimit.hasChanged(hashCode()) || supplyCurrentLimit.hasChanged(hashCode())) {
      io.setCurrentLimits(statorCurrentLimit.get(), supplyCurrentLimit.get());
    }
  }

  /** velocity will be calculated from aim assist command factory */
  private void setVelocity(double velocityRPS, double ffamps) {
    io.runLauncherVelocity(velocityRPS, ffamps);
  }

  public double getLauncherVelocity() {
    return inputs.launcher1Data.velocity();
  }

  // this is the CORRECT method to turn launcher off
  private void turnLauncherOff() {
    io.stopLauncher();
  }

  // this is for the hood
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
