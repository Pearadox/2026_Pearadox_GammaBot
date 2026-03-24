// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivers.MovingShotSolver;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOReal;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.DriveHelpers;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Feeder feeder;
  private final Intake intake;
  public final Launcher launcher;
  public final Spindexer spindexer;
  private final Turret turret;
  public final Vision vision;

  // Visualizer
  public final RobotVisualizer visualizer;

  // Controller
  private final CommandXboxController drivercontroller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register named commands for PathPlanner
    registerNamedCommands();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        feeder = new Feeder(new FeederIOReal());
        intake = new Intake(new IntakeIOReal());
        launcher = new Launcher(new LauncherIOReal());
        spindexer = new Spindexer(new SpindexerIOReal());
        turret = new Turret(new TurretIOReal(), drive::getChassisSpeeds, drive::getRotation);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation)
                // new VisionIOPhotonVision(
                //     VisionConstants.camera1Name, VisionConstants.robotToCamera1)
                );

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim());
        launcher = new Launcher(new LauncherIOSim());
        spindexer = new Spindexer(new SpindexerIOSim());
        turret = new Turret(new TurretIOSim(), drive::getChassisSpeeds, drive::getRotation);
        vision = new Vision(drive::addVisionMeasurement);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        launcher = new Launcher(new LauncherIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        turret = new Turret(new TurretIO() {}, drive::getChassisSpeeds, drive::getRotation);
        vision = new Vision(drive::addVisionMeasurement);

        break;
    }

    // Set up auto routines
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Super auto chooser", autoChooser);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    visualizer =
        new RobotVisualizer(
            turret::getTurretAngleRads,
            () -> 0, // TODO: replace with hood angle supplier
            () -> 0, // TODO: replace with spindexer angle supplier
            () -> Units.degreesToRadians(90 - intake.getAngleDegs()),
            () -> 0 // TODO: replace with climber displacement supplier
            );

    // Configure the button bindings
    configureButtonBindings();

    vision.setDefaultCommand(
        new RunCommand(
            () -> MovingShotSolver.getInstance().solve(drive::getPose, drive::getChassisSpeeds),
            vision));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Bindings
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -drivercontroller.getLeftY(),
            () -> -drivercontroller.getLeftX(),
            () -> -drivercontroller.getRightX()));

    // Switch to X pattern when X button is pressed
    drivercontroller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when start button is pressed
    drivercontroller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Drive at a 45° for going over the bump
    drivercontroller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -drivercontroller.getLeftY(),
                () -> -drivercontroller.getLeftX(),
                () -> DriveHelpers.findClosestCorner(drive::getPose)));

    drivercontroller
        .rightBumper()
        .whileTrue(
            new ShootOnTheMove(
                    launcher, feeder, spindexer, turret::getFieldRelativeTurretAngleRotation2d)
                .alongWith(launcher.score()))
        .onFalse(
            new InstantCommand(
                () -> {
                  feeder.setStopped();
                  spindexer.setStopped();
                }));

    drivercontroller
        .leftBumper()
        .whileTrue(new InstantCommand(() -> intake.setIntaking()))
        .onFalse(new InstantCommand(() -> intake.setDeployed()));
    drivercontroller.povUp().onTrue(new InstantCommand(() -> intake.setStowed()));
    drivercontroller.povDown().onTrue(new InstantCommand(() -> intake.setDeployed()));
    drivercontroller
        .povLeft()
        .onTrue(new InstantCommand(() -> intake.setOuttaking()))
        .onFalse(new InstantCommand(() -> intake.setDeployed()));

    drivercontroller
        .b()
        .whileTrue(new RunCommand(() -> spindexer.setReverse(), spindexer))
        .onFalse(new InstantCommand(() -> spindexer.setStopped(), spindexer));

    // Op Bindings
    opController.a().onTrue(new InstantCommand(() -> launcher.setIdle()));
    opController
        .b()
        .onTrue(
            new InstantCommand(
                () -> {
                  launcher.setOff();
                  spindexer.setStopped();
                  feeder.setStopped();
                }));
    opController.y().onTrue(new InstantCommand(() -> launcher.setManual()));

    opController.povLeft().whileTrue(new RunCommand(() -> turret.adjustRotationBy(+0.01)));
    opController.povRight().whileTrue(new RunCommand(() -> turret.adjustRotationBy(-0.01)));

    opController.povUp().whileTrue(new RunCommand(() -> intake.adjustVoltsBy(+0.1)));
    opController.povDown().whileTrue(new RunCommand(() -> intake.adjustVoltsBy(-0.1)));

    Trigger intakeAdjust = new Trigger(() -> Math.abs(opController.getLeftY()) > 0.9);
    intakeAdjust.whileTrue(
        new RunCommand(
            () ->
                intake.adjustPivotAngleBy(
                    Math.signum(-opController.getLeftY())
                        * IntakeConstants.OP_ADJUST_INCREMENT_DEGREES)));

    Trigger launcherAdjust = new Trigger(() -> Math.abs(opController.getRightY()) > 0.9);
    launcherAdjust.whileTrue(
        new RunCommand(
            () ->
                launcher.adjustRPSBy(
                    Math.signum(opController.getRightY())
                        * IntakeConstants.OP_ADJUST_INCREMENT_DEGREES)));

    turret.setDefaultCommand(
        new RunCommand(
            () ->
                turret.followFieldCentricTarget(
                    () -> MovingShotSolver.getShotSolution().turretAngle()),
            turret));

    opController
        .back()
        .onTrue(
            new RunCommand( // same as default cmd btw
                () ->
                    turret.followFieldCentricTarget(
                        () -> MovingShotSolver.getShotSolution().turretAngle()),
                turret));

    opController.leftBumper().onTrue(new RunCommand(() -> turret.goToZero(), turret));
    opController.rightBumper().onTrue(new RunCommand(() -> turret.goToTestSetpoint(), turret));

    opController
        .start()
        .onTrue(
            new InstantCommand(
                    () -> {
                      if (!turret.isHasZeroed()) {
                        // Zeroes the turret and sets it to brake mode
                        turret.requestZero();
                      } else {
                        // If the start button is pressed a second time,
                        // set it back to coast mode so it can be rezeroed
                        turret.undoZero();
                      }
                    })
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    // Feeder Commands
    NamedCommands.registerCommand(
        "Set Launching",
        new InstantCommand(() -> launcher.setScoring())
            .andThen(new WaitCommand(0.2))
            .andThen(new InstantCommand(() -> feeder.setRunning()))
            .andThen(new WaitCommand(0.2))
            .andThen(
                (new InstantCommand(
                    () -> {
                      spindexer.setRunning();
                    }))));
    NamedCommands.registerCommand(
        "Stop Launching",
        new InstantCommand(() -> feeder.setStopped())
            .andThen(
                (new InstantCommand(
                    () -> {
                      spindexer.setStopped();
                      launcher.setOff();
                    }))));

    // Intake Commands
    NamedCommands.registerCommand("Set Intaking", new InstantCommand(() -> intake.setIntaking()));
    NamedCommands.registerCommand("Stop Intaking", new InstantCommand(() -> intake.setDeployed()));
    NamedCommands.registerCommand("Stow Intake", new InstantCommand(() -> intake.setStowed()));
    NamedCommands.registerCommand("Flow Intake", new InstantCommand(() -> intake.setFlow()));
  }
}
