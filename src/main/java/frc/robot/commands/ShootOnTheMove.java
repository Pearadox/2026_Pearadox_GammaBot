// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivers.MovingShotSolver;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove extends Command {
  private final Launcher launcher;
  private final Feeder feeder;
  private final Spindexer spindexer;

  // Read-only: the turret keeps running its own aiming command, so it is deliberately not a
  // requirement of this command.
  private final Turret turret;

  private Debouncer debouncer = new Debouncer(0.2, DebounceType.kFalling);
  private Debouncer turretRotationDebouncer = new Debouncer(0.2, DebounceType.kBoth);

  private static final LoggedTunableNumber rotationToleranceDeg =
      new LoggedTunableNumber("SOTM/Rotation Tolerance Deg", 4.0);
  private static final LoggedTunableNumber velocityToleranceRPS =
      new LoggedTunableNumber("SOTM/Velocity Tolerance RPS", 7.0);

  // Set to 1 to feed regardless of readiness. This exists so a field emergency never needs a code
  // edit; it replaces the hardcoded "readyToShoot = true" that used to live here.
  private static final LoggedTunableNumber bypassReadinessGate =
      new LoggedTunableNumber("SOTM/Bypass Readiness Gate", 0);

  private boolean atDesiredVelocity = false;
  private boolean atDesiredRotation = false;
  private boolean readyToShoot = false;

  public ShootOnTheMove(Launcher launcher, Feeder feeder, Spindexer spindexer, Turret turret) {
    this.turret = turret;
    this.launcher = launcher;
    this.feeder = feeder;
    this.spindexer = spindexer;
    addRequirements(launcher, feeder, spindexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double desiredVelocity = MovingShotSolver.getShotSolution().speed();
    Rotation2d desiredRotation = MovingShotSolver.getShotSolution().turretAngle();

    double shooterVelocityError = launcher.getLauncherVelocity() - desiredVelocity;

    atDesiredVelocity =
        debouncer.calculate(Math.abs(shooterVelocityError) < velocityToleranceRPS.get());
    atDesiredRotation =
        turretRotationDebouncer.calculate(
            turret.isAtFieldAngle(desiredRotation, rotationToleranceDeg.get()));

    readyToShoot = bypassReadinessGate.get() > 0.5 || (atDesiredVelocity && atDesiredRotation);

    if (readyToShoot) {
      feeder.setRunning();
      spindexer.setRunning();
    } else {
      feeder.setStopped();
      spindexer.setStopped();
    }

    Logger.recordOutput("SOTM/readyToShoot", readyToShoot);
    Logger.recordOutput("SOTM/atDesiredVelocity", atDesiredVelocity);
    Logger.recordOutput("SOTM/Velocity-Error_RPS", shooterVelocityError);
    Logger.recordOutput("SOTM/Desired-Velocity_RPS", desiredVelocity);

    Logger.recordOutput("SOTM/atDesiredRotation", atDesiredRotation);
    Logger.recordOutput("SOTM/desiredAngle", desiredRotation.getDegrees());
    Logger.recordOutput(
        "SOTM/currentAngle", turret.getFieldRelativeTurretAngleRotation2d().getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setStopped();
    spindexer.setStopped();
    atDesiredVelocity = false;
    atDesiredRotation = false;
    readyToShoot = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
