// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AlignConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.DriveHelpers;
import frc.robot.util.LoggedTunableNumber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DemoAimAtTrashCan extends Command {

    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Demo/kP", 2);
    private static final LoggedTunableNumber drivekI = new LoggedTunableNumber("Demo/kI", 0);
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Demo/kD", 0);
    private static final LoggedTunableNumber driveMaxVel =
            new LoggedTunableNumber("Demo/Max Vel", 2);
    private static final LoggedTunableNumber driveMaxAcc =
            new LoggedTunableNumber("Demon/Max Acc", 3);

  private final Drive drive;
  private final Vision vision;
  private final Turret turret;

  private final Supplier<Pose2d> targetPoseSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;

 private final ProfiledPIDController translationController =
  new ProfiledPIDController(drivekP.get(), drivekI.get(), drivekD.get(),
    new TrapezoidProfile.Constraints(
      driveMaxVel.get(),
      driveMaxAcc.get()
    )
);
  
  public DemoAimAtTrashCan(Drive drive, Vision vision, Turret turret) {
    
    this.drive = drive;
    this.vision = vision;
    this.turret = turret;


    this.targetPoseSupplier = () -> DriveHelpers.calculateOffsetInFrontOfTag(vision.getLastTrustedPose());
    this.robotPoseSupplier = drive::getPose;

    addRequirements(drive, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d target = targetPoseSupplier.get();
    Pose2d robot = robotPoseSupplier.get();
    
    Translation2d error = target.minus(robot).getTranslation();
    translationController.reset(error.getNorm());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d robotPose = robotPoseSupplier.get();

    Translation2d error = targetPose.minus(robotPose).getTranslation();
    double distance = error.getNorm();
    double speed = translationController.calculate(distance, 0.0);
    Translation2d direction =
      distance > 0.01 ? error.div(distance) : new Translation2d(); //1e-6

    Translation2d velocity = direction.times(speed); // new velocity

    drive.runVelocity(new ChassisSpeeds(
      velocity.getX(),
      velocity.getY(),
      0.0
    ));

    turret.followFieldCentricTarget(
      () -> vision.getLastTrustedPose().getRotation()
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
