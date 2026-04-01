// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.MovingShotSolver;
import frc.robot.subsystems.launcher.LauncherConstants.LauncherState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  @AutoLogOutput @Getter @Setter private LauncherState launcherState = LauncherState.OFF;

  private double rpsAdjust = 0.0;

  public void adjustRPSBy(double adj) {
    rpsAdjust += adj;
  }

  private final LoggedTunableNumber tunableffAmps = new LoggedTunableNumber("Launcher/ffamps", 0);
  private final LoggedTunableNumber manualDefaultVelocity =
      new LoggedTunableNumber(
          "Launcher/Manual Mode Default Velocity", LauncherConstants.DEFAULT_VELOCITY_SETPOINT_RPS);
  private final LoggedTunableNumber idleDefaultVelocity =
      new LoggedTunableNumber("Launcher/Idle Mode Default Velocity", 20);

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
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    double desiredVelocity;
    if (launcherState == LauncherState.SELF_DIRECTING) {
      desiredVelocity = MovingShotSolver.getShotSolution().speed();
    } else if (launcherState == LauncherState.MANUAL) {
      desiredVelocity = manualDefaultVelocity.get();
    } else if (launcherState == LauncherState.IDLE) {
      desiredVelocity = idleDefaultVelocity.get();
    } else {
      desiredVelocity = 0;
    }

    desiredVelocity = Math.abs(desiredVelocity + rpsAdjust);

    if (desiredVelocity < LauncherConstants.SHOOTER_VELOCITY_DEADBAND) {
      desiredVelocity = 0;
      turnLauncherOff();
    } else {
      setVelocity(desiredVelocity, tunableffAmps.get());
    }

    LauncherVisualizer.getInstance()
        .updateFlywheelPositionDeg(Units.rotationsToDegrees(inputs.launcher1Data.position()));
    LauncherVisualizer.getInstance()
        .updateHoodPositionDeg(Units.rotationsToDegrees(inputs.hoodData.position()));

    Logger.recordOutput("Launcher/adjust", rpsAdjust);
    Logger.recordOutput("Debug/getLauncherVelocity", getLauncherVelocity());

    // io.setHoodAngleRads(launcherState.getHoodAngleRads());
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

    io.setHoodAngleRads(launcherState.getHoodAngleRads());
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

  public void setOff() {
    launcherState = LauncherState.OFF;
  }

  public void setManual() {
    launcherState = LauncherState.MANUAL;
  }

  public void setScoring() {
    launcherState = LauncherState.SELF_DIRECTING;
  }

  public void setIdle() {
    launcherState = LauncherState.IDLE;
  }

  public Command score() {
    return Commands.startEnd(
        () -> {
          if (launcherState != LauncherState.MANUAL) {
            launcherState = LauncherState.SELF_DIRECTING;
          }
        },
        () -> {
          if (launcherState == LauncherState.SELF_DIRECTING) {
            launcherState = LauncherState.IDLE;
          }
        });
  }

  // public void setPassing() {
  //   launcherState = LauncherState.PASSING;
  // }
}
