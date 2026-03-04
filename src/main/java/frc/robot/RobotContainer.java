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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivers.MovingShotSolver;
import frc.lib.drivers.MovingShotSolver.ShotSolution;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Feeder feeder;
  private final Intake intake;
  private final Launcher launcher;
  private final Spindexer spindexer;
  private final Turret turret;
  private final Vision vision;

  // Visualizer
  public final RobotVisualizer visualizer;

  // Controller
  private final CommandXboxController drivercontroller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static ShotSolution shotSolution = new ShotSolution(0., 0, new Rotation2d(), false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

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
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
            () -> 0, // TODO: replace with intake angle supplier
            () -> 0 // TODO: replace with climber displacement supplier
            );

    // Configure the button bindings
    configureButtonBindings();
    // Register named commands for PathPlanner
    registerNamedCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -drivercontroller.getLeftY(),
            () -> -drivercontroller.getLeftX(),
            () -> -drivercontroller.getRightX()));

    // Lock to 0° when A button is held
    drivercontroller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -drivercontroller.getLeftY(),
                () -> -drivercontroller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    drivercontroller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    drivercontroller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // drivercontroller.y().whileTrue(new RotateToBump(drive, drive:: getPose));

    drivercontroller
        .y()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -drivercontroller.getLeftY(),
                () -> -drivercontroller.getLeftX(),
                () -> DriveHelpers.findClosestCorner(drive::getPose)));

    drivercontroller
        .rightBumper()
        .whileTrue(
            new ShootOnTheMove(launcher, feeder, turret::getFieldRelativeTurretAngleRotation2d));

    // drivercontroller.leftBumper().whileTrue(new RunCommand(() -> launcher.setVelocity(40)));

    // Uncomment when ready to run turret SysID routines
    // opController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // opController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    // opController.y().whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // opController.a().whileTrue(turret.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // opController.b().whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // opController.x().whileTrue(turret.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    opController.a().onTrue(new InstantCommand(() -> turret.goToZero(), turret));
    opController.b().onTrue(new InstantCommand(() -> turret.goToPlus180(), turret));
    // opController.x().onTrue(new InstantCommand(() -> turret.goToMinus180(), turret));
    opController
        .y()
        .onTrue(
            new RunCommand(
                () -> {
                  setShotSolution(MovingShotSolver.solve(drive::getPose, drive::getChassisSpeeds));
                  turret.followFieldCentricTarget(shotSolution::getTurretAngleRot2d);
                },
                turret));

    // opController.povLeft().whileTrue(new InstantCommand(() -> intake.setIntaking()));
    drivercontroller.povUp().onTrue(new InstantCommand(() -> intake.setDeployed()));
    drivercontroller.povDown().onTrue(new InstantCommand(() -> intake.setStowed()));

    // drivercontroller.y().onTrue(new InstantCommand(() -> launcher.setPassing()));
    // drivercontroller.a().onTrue(new InstantCommand(() -> launcher.setScoring()));

    // drivercontroller.rightBumper().whileTrue(new InstantCommand(() ->
    // feeder.launch()));
    // drivercontroller.rightBumper().onFalse(new InstantCommand(() ->
    // feeder.stopLaunch()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void registerNamedCommands() {
    // Feeder Commands
    NamedCommands.registerCommand("Set Launching", new InstantCommand(() -> feeder.setRunning()));
    NamedCommands.registerCommand("Stop Launching", new InstantCommand(() -> feeder.setStopped()));

    // Intake Commands
    // NamedCommands.registerCommand("Set Intaking", new InstantCommand(() ->
    // intake.setIntaking()));
    // NamedCommands.registerCommand("Stop Intaking", new InstantCommand(() ->
    // intake.setDeployed()));
  }

  public void setShotSolution(ShotSolution sol) {
    shotSolution = sol;
  }

  public static ShotSolution getShotSolution() {
    return shotSolution;
  }
}
