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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove extends Command {
  private final Launcher launcher;
  private final Feeder feeder;
  private final Spindexer spindexer;

  private Debouncer debouncer = new Debouncer(0.25, DebounceType.kFalling);
  // private Debouncer turretRotationDebouncer = new Debouncer(0.1, DebounceType.kBoth);
  private Supplier<Rotation2d> turretRotationSupplier;

  private boolean atDesiredVelocity = false;
  // private boolean atDesiredRotation = false;
  private boolean readyToShoot = false;

  public ShootOnTheMove(
      Launcher launcher,
      Feeder feeder,
      Spindexer spindexer,
      Supplier<Rotation2d> turretRotationSupplier) {
    this.turretRotationSupplier = turretRotationSupplier;
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
    // Rotation2d desiredRotation = MovingShotSolver.getShotSolution().turretAngle();

    double shooterVelocityError = launcher.getLauncherVelocity() - desiredVelocity;

    // double currentAngle = turretRotationSupplier.get().getDegrees();
    // double desiredAngle = desiredRotation.getDegrees();

    // double turretRotationError = currentAngle - desiredAngle;

    atDesiredVelocity = debouncer.calculate(Math.abs(shooterVelocityError) < 7.0);
    // atDesiredRotation = turretRotationDebouncer.calculate(Math.abs(turretRotationError) < 8.0);

    readyToShoot = true; // atDesiredRotation && atDesiredRotation;

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
    // Logger.recordOutput("SOTM/rotationError", turretRotationError);
    // Logger.recordOutput("SOTM/atDesiredRotation", atDesiredRotation);
    // Logger.recordOutput("SOTM/currentAngle", currentAngle);
    // Logger.recordOutput("SOTM/desiredAngle", desiredAngle);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setStopped();
    spindexer.setStopped();
    atDesiredVelocity = false;
    // atDesiredRotation = false;
    readyToShoot = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
