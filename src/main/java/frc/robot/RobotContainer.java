// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.drivers.MovingShotSolver;
import frc.lib.drivers.MovingShotSolver.Goal;
import frc.robot.Constants.VisualizerConstants;
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
import frc.robot.subsystems.launcher.LauncherConstants.LauncherState;
import frc.robot.subsystems.launcher.LauncherIO;
import frc.robot.subsystems.launcher.LauncherIOReal;
import frc.robot.subsystems.launcher.LauncherIOSim;
import frc.robot.subsystems.leds.LEDStrip;
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
import frc.robot.util.LoggedTracer;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Feeder feeder;
  private final Intake intake;
  public final Launcher launcher;
  public final Spindexer spindexer;
  private final Turret turret;
  public final Vision vision;
  public static final LEDStrip ledStrip = LEDStrip.getInstance();

  // Visualizer
  public final RobotVisualizer visualizer;
  @Getter @Setter private double robotSpeedMultiplier = 1.0;

  // Controller
  private final CommandXboxController drivercontroller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);

  private final CommandXboxController blakeController = new CommandXboxController(3);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register named commands for PathPlanner

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
        turret =
            new Turret(
                new TurretIOReal(),
                drive::getChassisSpeeds,
                drive::getRotation,
                intake::turretHasClearance);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getChassisSpeeds,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation)
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
        turret =
            new Turret(
                new TurretIOSim(),
                drive::getChassisSpeeds,
                drive::getRotation,
                intake::turretHasClearance);
        vision = new Vision(drive::addVisionMeasurement, drive::getChassisSpeeds);

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

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
        turret =
            new Turret(
                new TurretIO() {},
                drive::getChassisSpeeds,
                drive::getRotation,
                intake::turretHasClearance);
        vision = new Vision(drive::addVisionMeasurement, drive::getChassisSpeeds);

        break;
    }

    registerNamedCommands();

    // Set up auto routines
    setUpAutonomousCommand();

    visualizer =
        new RobotVisualizer(
            turret::getTurretAngleRads,
            launcher::getHoodAngleRads,
            () ->
                VisualizerConstants.INTAKE_STARTING_ANGLE
                    - Units.degreesToRadians(intake.getAngleDegs()));

    // Configure the button bindings
    configureButtonBindings();

    vision.setDefaultCommand(
        new RunCommand(
            () -> {
              LoggedTracer.reset();
              MovingShotSolver.getInstance().solve(drive::getPose, drive::getChassisSpeeds);
              LoggedTracer.record("MovingShotSolve");

              //   Logger.recordOutput(
              //       "Odometry/test",
              //       new Pose3d(drive.getPose()).transformBy(visualizer.getLlTransform()));
            },
            vision));
    ledStrip.setDefaultCommand(new RunCommand(() -> ledStrip.isHubActive(), ledStrip));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Define preferred controller active checking logic:
    // If the drivercontroller is trying to drive (sticks pushed beyond standard 0.1
    // deadband),
    // we use drivercontroller joysticks and ignore blakeController completely.
    DoubleSupplier activeY =
        () -> {
          boolean driverActive =
              Math.abs(drivercontroller.getLeftY()) > 0.1
                  || Math.abs(drivercontroller.getLeftX()) > 0.1
                  || Math.abs(drivercontroller.getRightX()) > 0.1;
          return driverActive ? drivercontroller.getLeftY() : blakeController.getLeftY();
        };

    DoubleSupplier activeX =
        () -> {
          boolean driverActive =
              Math.abs(drivercontroller.getLeftY()) > 0.1
                  || Math.abs(drivercontroller.getLeftX()) > 0.1
                  || Math.abs(drivercontroller.getRightX()) > 0.1;
          return driverActive ? drivercontroller.getLeftX() : blakeController.getLeftX();
        };

    DoubleSupplier activeOmega =
        () -> {
          boolean driverActive =
              Math.abs(drivercontroller.getLeftY()) > 0.1
                  || Math.abs(drivercontroller.getLeftX()) > 0.1
                  || Math.abs(drivercontroller.getRightX()) > 0.1;
          return driverActive ? drivercontroller.getRightX() : blakeController.getRightX();
        };

    // Driver Bindings
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -activeY.getAsDouble() * getRobotSpeedMultiplier(),
            () -> -activeX.getAsDouble() * getRobotSpeedMultiplier(),
            () -> -activeOmega.getAsDouble()));

    // Switch to X pattern when X button is pressed
    drivercontroller.x().or(blakeController.x()).onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when start button is pressed
    drivercontroller
        .start()
        .or(blakeController.start())
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
        .or(blakeController.a())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -activeY.getAsDouble(),
                () -> -activeX.getAsDouble(),
                () -> DriveHelpers.findClosestCorner(drive::getPose)));

    drivercontroller
        .y()
        .or(blakeController.y())
        .toggleOnTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -activeY.getAsDouble(),
                () -> -activeX.getAsDouble(),
                () ->
                    DriveHelpers.getCourseRotation2d(drive::getChassisSpeeds, drive::getRotation)));

    // drivercontroller
    //     .rightBumper()
    //     .and(() -> launcher.getLauncherState() != LauncherState.MANUAL)
    //     .whileTrue(
    //         new ShootOnTheMove(
    //                 launcher, feeder, spindexer, turret)
    //             .alongWith(launcher.score()))
    //     .onFalse(
    //         new InstantCommand(() -> spindexer.setStopped())
    //             .andThen(new WaitCommand(0.2))
    //             .andThen(new InstantCommand(() -> feeder.setStopped())));

    // drivercontroller
    //     .rightBumper()
    //     .and(() -> launcher.getLauncherState() == LauncherState.MANUAL)
    //     .whileTrue(
    //         new InstantCommand(
    //             () -> {
    //               feeder.setRunning();
    //               spindexer.setRunning();
    //             }))
    //     .onFalse(
    //         new InstantCommand(() -> spindexer.setStopped())
    //             .andThen(new WaitCommand(0.2))
    //             .andThen(new InstantCommand(() -> feeder.setStopped())));

    drivercontroller
        .rightBumper()
        .whileTrue(
            Commands.either(
                Commands.startEnd(
                    () -> {
                      feeder.setRunning();
                      spindexer.setRunning();
                    },
                    () -> {
                      spindexer.setStopped();
                      feeder.setStopped();
                    }),
                new ShootOnTheMove(launcher, feeder, spindexer, turret)
                    .alongWith(launcher.score())
                    .finallyDo(
                        (b) -> {
                          spindexer.setStopped();
                          feeder.setStopped();
                        }),
                () -> launcher.getLauncherState() == LauncherState.MANUAL));

    drivercontroller
        .rightBumper()
        .and(() -> MovingShotSolver.getInstance().getGoal() == Goal.HUB)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  setRobotSpeedMultiplier(Math.sqrt(0.25));
                },
                () -> {
                  setRobotSpeedMultiplier(1.0);
                }));

    drivercontroller
        .leftBumper()
        .or(blakeController.leftTrigger())
        .whileTrue(new InstantCommand(() -> intake.setIntaking()))
        .onFalse(new InstantCommand(() -> intake.setDeployed()));

    drivercontroller.povUp().onTrue(new InstantCommand(() -> intake.setFlow()));

    drivercontroller
        .povDown()
        .or(blakeController.povDown())
        .onTrue(new InstantCommand(() -> intake.setDeployed()));

    drivercontroller
        .povLeft()
        .or(blakeController.povLeft())
        .onTrue(new InstantCommand(() -> intake.setOuttaking()))
        .onFalse(new InstantCommand(() -> intake.setDeployed()));

    drivercontroller
        .b()
        .or(blakeController.b())
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

    opController
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (launcher.getLauncherState() != LauncherState.MANUAL) {
                    launcher.setManual();
                  } else launcher.setIdle();
                }));

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
        .or(blakeController.back())
        .onTrue(
            new RunCommand( // same as default cmd btw
                () ->
                    turret.followFieldCentricTarget(
                        () -> MovingShotSolver.getShotSolution().turretAngle()),
                turret));

    // opController.leftBumper().onTrue(new RunCommand(() -> turret.goToZero(), turret));
    // opController.rightBumper().onTrue(new RunCommand(() -> turret.goToTestSetpoint(), turret));

    // Don't jostle while the driver is intaking. This has to live on the trigger: as a plain if
    // around the binding it was evaluated once at boot (when nobody is holding the bumper), so
    // the binding always registered and the condition never applied at runtime.
    opController
        .rightBumper()
        .and(() -> !drivercontroller.leftBumper().getAsBoolean())
        .whileTrue(
            new SequentialCommandGroup(
                    new InstantCommand(() -> intake.setFlow()),
                    new WaitCommand(IntakeConstants.INTAKE_JOSTLE_TIME_SEC),
                    new InstantCommand(() -> intake.setDeployed()),
                    new WaitCommand(IntakeConstants.INTAKE_JOSTLE_TIME_SEC))
                .repeatedly())
        .onFalse(new InstantCommand(() -> intake.setDeployed()));

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

    // runs the hood down, then when released, zeroes the hood
    // if disabled, the hood won't run down
    opController.x().whileTrue(launcher.zeroHoodCommand().ignoringDisable(true));

    // ==========================================
    // Blake Controller (Port 3) Bindings
    // ==========================================

    // STOW Intake when HELD (Left Bumper)
    blakeController
        .leftBumper()
        .whileTrue(new InstantCommand(() -> intake.setStowed()))
        .onFalse(new InstantCommand(() -> intake.setDeployed()));

    // STOW Intake (Dpad Up)
    blakeController.povUp().onTrue(new InstantCommand(() -> intake.setStowed()));

    // IN-Take Intake (Dpad Right)
    blakeController
        .povRight()
        .onTrue(new InstantCommand(() -> intake.setIntaking()))
        .onFalse(new InstantCommand(() -> intake.setDeployed()));

    // Toggle AUTO Shoot (Right Bumper)
    blakeController
        .rightBumper()
        .toggleOnTrue(
            new ShootOnTheMove(launcher, feeder, spindexer, turret)
                .alongWith(launcher.score())
                .alongWith(
                    new SequentialCommandGroup(
                            Commands.run(
                                    () -> {
                                      blakeController
                                          .getHID()
                                          .setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                                      blakeController
                                          .getHID()
                                          .setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
                                    })
                                .withTimeout(0.5),
                            new SequentialCommandGroup(
                                    Commands.run(
                                            () -> {
                                              blakeController
                                                  .getHID()
                                                  .setRumble(
                                                      GenericHID.RumbleType.kLeftRumble, 0.0);
                                              blakeController
                                                  .getHID()
                                                  .setRumble(
                                                      GenericHID.RumbleType.kRightRumble, 1.0);
                                            })
                                        .withTimeout(0.2),
                                    Commands.run(
                                            () -> {
                                              blakeController
                                                  .getHID()
                                                  .setRumble(
                                                      GenericHID.RumbleType.kLeftRumble, 0.0);
                                              blakeController
                                                  .getHID()
                                                  .setRumble(
                                                      GenericHID.RumbleType.kRightRumble, 0.0);
                                            })
                                        .withTimeout(0.05))
                                .repeatedly())
                        .finallyDo(
                            (b) -> {
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                            }))
                .finallyDo(
                    (b) -> {
                      spindexer.setStopped();
                      feeder.setStopped();
                    }));

    // Shoot OTM (Right Trigger)
    blakeController
        .rightTrigger()
        .whileTrue(
            new ShootOnTheMove(launcher, feeder, spindexer, turret)
                .alongWith(launcher.score())
                .alongWith(
                    Commands.startEnd(
                        () -> {
                          blakeController
                              .getHID()
                              .setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                          blakeController
                              .getHID()
                              .setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
                        },
                        () -> {
                          blakeController
                              .getHID()
                              .setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                          blakeController
                              .getHID()
                              .setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                        }))
                .finallyDo(
                    (b) -> {
                      spindexer.setStopped();
                      feeder.setStopped();
                    }));

    // Rumble for Left Trigger (Intake)
    blakeController
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  blakeController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                  blakeController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.22);
                },
                () -> {
                  blakeController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                  blakeController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                }));

    // Rumble for B Button (Reverse Spindexer)
    blakeController
        .b()
        .onTrue(
            Commands.run(
                    () -> {
                      blakeController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                      blakeController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
                    })
                .withTimeout(0.25)
                .finallyDo(
                    (b) -> {
                      blakeController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                      blakeController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                    }));

    // Rumble for Start Button (Zero Turret Heartbeat)
    blakeController
        .start()
        .onTrue(
            new SequentialCommandGroup(
                    Commands.run(
                            () -> {
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                            })
                        .withTimeout(0.15),
                    Commands.run(
                            () -> {
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                            })
                        .withTimeout(0.15),
                    Commands.run(
                            () -> {
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                              blakeController
                                  .getHID()
                                  .setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                            })
                        .withTimeout(0.15))
                .ignoringDisable(true)
                .finallyDo(
                    (b) -> {
                      blakeController.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                      blakeController.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void setUpAutonomousCommand() {
    // autoChooser.addOption(
    //     "OTrench-NZone-2.5-Sweeps", new PathPlannerAuto("OTrench-NZone-2.5-Sweeps"));
    // autoChooser.addOption(
    //     "DTrench-NZone-2.5-Sweeps", new PathPlannerAuto("OTrench-NZone-2.5-Sweeps", true));

    autoChooser.addOption(
        "Adamant Trench (Outpost, 3 Sweeps, Rush)",
        new PathPlannerAuto("Adamant Trench (Outpost, 3 Sweeps, Rush)"));
    autoChooser.addOption(
        "Adamant Trench (Depot, 3 Sweeps, Rush)",
        new PathPlannerAuto("Adamant Trench (Outpost, 3 Sweeps, Rush)", true));

    // autoChooser.addOption(
    //     "CircleBack Outpost",
    //     new PathPlannerAuto("CircleBack Adamant Trench (Outpost, 3 Sweeps, Rush)"));

    // autoChooser.addOption(
    //     "CircleBack Depot",
    //     new PathPlannerAuto("CircleBack Adamant Trench (Outpost, 3 Sweeps, Rush)", true));

    autoChooser.addOption("Center Depot", new PathPlannerAuto("Center (Depot Intaking)"));

    SmartDashboard.putData("clean auto chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void registerNamedCommands() {
    // Timer Commands
    NamedCommands.registerCommand("Start Timer", new InstantCommand(() -> feeder.startTimer()));

    // Launching Sequence Commands
    NamedCommands.registerCommand(
        "Set Launching",
        new InstantCommand(() -> launcher.setScoring())
            .andThen(new InstantCommand(() -> feeder.startTimer()))
            .andThen(new WaitCommand(0.4))
            .andThen(new InstantCommand(() -> feeder.setRunning()))
            .andThen(new WaitCommand(0.2))
            .andThen(
                (new RunCommand(() -> spindexer.setRunning(), spindexer))
                    .until(() -> feeder.isHopperEmpty())
                    // isHopperEmpty() needs the CANrange to have seen fuel since resetForAuto().
                    // A dirty lens, weak signal, or an empty pickup means it never goes true and
                    // the auto stops here forever. Autos may waste seconds, never stall.
                    .withTimeout(4.0))
            .finallyDo(
                (bool) -> {
                  feeder.setStopped();
                  spindexer.setStopped();
                  launcher.setIdle();
                }));

    NamedCommands.registerCommand(
        "Set Launching (No Wait)",
        new InstantCommand(() -> launcher.setScoring())
            .andThen(new WaitCommand(0.2))
            .andThen(new InstantCommand(() -> feeder.setRunning()))
            .andThen(new WaitCommand(0.2))
            .andThen(new InstantCommand(() -> spindexer.setRunning())));

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
    NamedCommands.registerCommand(
        "Set Intaking Fast", new InstantCommand(() -> intake.setIntakingFast()));
    NamedCommands.registerCommand("Stop Intaking", new InstantCommand(() -> intake.setDeployed()));
    NamedCommands.registerCommand("Stow Intake", new InstantCommand(() -> intake.setStowed()));
    NamedCommands.registerCommand("Flow Intake", new InstantCommand(() -> intake.setFlow()));
    NamedCommands.registerCommand(
        "Jostle Intake",
        new RunCommand(() -> intake.setIntaking(), intake)
            .withTimeout(1) // purposefully left a little longer so no jamming
            .andThen(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setFlow()),
                        new WaitCommand(IntakeConstants.INTAKE_JOSTLE_TIME_SEC),
                        new InstantCommand(() -> intake.setDeployed()),
                        new WaitCommand(IntakeConstants.INTAKE_JOSTLE_TIME_SEC))
                    .repeatedly())
            .finallyDo((bool) -> intake.setIntaking()));

    new EventTrigger("Set Intaking").onTrue(new InstantCommand(() -> intake.setIntaking()));
  }
}
