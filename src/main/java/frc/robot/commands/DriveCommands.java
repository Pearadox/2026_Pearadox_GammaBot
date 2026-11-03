// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LinearAccelLimiter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.StickCutoffLatch;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 7.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = Units.degreesToRadians(540);
  private static final double ANGLE_MAX_ACCELERATION = Units.degreesToRadians(720);
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Dashboard numbers for driver practice. They show up under /Tuning/Drive on the dashboard
  // (AdvantageScope, Elastic, or Shuffleboard), the same place the launcher's tunables live.
  // All four are percents. They are clamped in code to PERCENT_MIN..PERCENT_MAX so a typo on the
  // dashboard cannot freeze the robot or push it past its real limits. They only affect joystick
  // driving, never autonomous paths.
  private static final double PERCENT_MIN = 20.0;
  private static final double PERCENT_MAX = 100.0;
  private static final LoggedTunableNumber maxSpeedPercent =
      new LoggedTunableNumber("Drive/Max Speed Percent", 100.0);
  private static final LoggedTunableNumber maxAccelPercent =
      new LoggedTunableNumber("Drive/Max Acceleration Percent", 100.0);
  private static final LoggedTunableNumber maxRotationPercent =
      new LoggedTunableNumber("Drive/Max Rotation Speed Percent", 100.0);
  // Practice mode. Stick pushes past this percent of full deflection stop the robot until the
  // stick comes back inside the deadband. 100 means off.
  private static final LoggedTunableNumber stickCutoffPercent =
      new LoggedTunableNumber("Drive/Stick Cutoff Percent", 100.0);

  private DriveCommands() {}

  /**
   * Reads a percent tunable, clamps it to the allowed range, and returns it as a 0 to 1 fraction.
   */
  private static double percentToFraction(LoggedTunableNumber percent) {
    return MathUtil.clamp(percent.get(), PERCENT_MIN, PERCENT_MAX) / 100.0;
  }

  /**
   * Returns the stick magnitude clamped to 1.0. Each stick axis reports its own value in [-1, 1],
   * so a diagonal push can read up to sqrt(2) through hypot. Without the clamp a corner push
   * squared to 2.0 and asked the drivebase for twice its top speed, so diagonals drove faster than
   * equal-feeling axial pushes.
   */
  static double getStickMagnitude(double x, double y) {
    return Math.min(Math.hypot(x, y), 1.0);
  }

  /** Converts a stick direction and an already-shaped magnitude into a unit-scale velocity. */
  private static Translation2d getLinearVelocityFromJoysticks(
      double x, double y, double magnitude) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(magnitude, DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Rotation that converts between the driver's frame (stick forward points away from the driver
   * station) and the robot frame. Flipped for the red alliance.
   */
  private static Rotation2d driverFrameRotation(Drive drive) {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
  }

  /** The robot's measured velocity expressed in the driver's frame, in meters per second. */
  private static Translation2d currentDriverFrameVelocity(Drive drive) {
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), driverFrameRotation(drive));
    return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
  }

  /**
   * Per-command state for turning the translation stick into a velocity: the practice cutoff latch
   * and the acceleration limiter. Each drive command gets its own instance.
   */
  private static class TranslationShaper {
    private final StickCutoffLatch latch = new StickCutoffLatch();
    private final LinearAccelLimiter limiter = new LinearAccelLimiter();

    /** Call when the command starts so the first loop ramps from the robot's current motion. */
    void reset(Drive drive) {
      latch.reset();
      limiter.reset(currentDriverFrameVelocity(drive));
    }

    /**
     * Stick x and y (each in [-1, 1]) to a driver-frame velocity in meters per second, in this
     * order: clamp the magnitude, practice cutoff latch, deadband, squaring, max speed percent,
     * then the acceleration limit.
     */
    Translation2d calculate(Drive drive, double x, double y) {
      double stickMagnitude = getStickMagnitude(x, y);
      boolean tripped =
          latch.update(stickMagnitude, percentToFraction(stickCutoffPercent), DEADBAND);
      double shapedMagnitude = tripped ? 0.0 : stickMagnitude;
      Translation2d unitVelocity = getLinearVelocityFromJoysticks(x, y, shapedMagnitude);

      double maxSpeed = drive.getMaxLinearSpeedMetersPerSec() * percentToFraction(maxSpeedPercent);
      double maxAccel =
          drive.getMaxLinearAccelMetersPerSecSq() * percentToFraction(maxAccelPercent);
      Translation2d limited =
          limiter.calculate(unitVelocity.times(maxSpeed), maxAccel, Constants.LOOP_PERIOD);

      Logger.recordOutput("Drive/Practice/StickMagnitude", stickMagnitude);
      Logger.recordOutput("Drive/Practice/CutoffTripped", tripped);
      Logger.recordOutput("Drive/Practice/MaxSpeedMetersPerSec", maxSpeed);
      Logger.recordOutput("Drive/Practice/MaxAccelMetersPerSecSq", maxAccel);
      return limited;
    }
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    TranslationShaper shaper = new TranslationShaper();
    return Commands.run(
            () -> {
              // Get linear velocity (driver frame, meters per second)
              Translation2d linearVelocity =
                  shaper.calculate(drive, xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              double maxAngularSpeed =
                  drive.getMaxAngularSpeedRadPerSec() * percentToFraction(maxRotationPercent);
              Logger.recordOutput("Drive/Practice/MaxAngularSpeedRadPerSec", maxAngularSpeed);

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX(), linearVelocity.getY(), omega * maxAngularSpeed);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driverFrameRotation(drive)));
            },
            drive)
        // Reset the practice latch and the acceleration limiter when the command starts
        .beforeStarting(() -> shaper.reset(drive));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Practice cutoff latch and acceleration limiter for the translation stick
    TranslationShaper shaper = new TranslationShaper();

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity (driver frame, already in meters per second)
              Translation2d linearVelocity =
                  shaper.calculate(drive, xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driverFrameRotation(drive)));
            },
            drive)

        // Reset PID controller, practice latch, and acceleration limiter when command starts
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians());
              shaper.reset(drive);
            });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
