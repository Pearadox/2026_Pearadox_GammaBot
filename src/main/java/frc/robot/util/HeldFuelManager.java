package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HeldFuelManager {
  // 5 speech bubbles in spindexer + 1 between transfer & turret
  private static final int TOTAL_CAPACITY = 50;
  private static final int SPINDEXER_CAPACITY = TOTAL_CAPACITY - 1;
  private static final Translation3d BUBBLE_TRANSLATION = new Translation3d(0.11, -0.336, 0.05);
  private static final Rotation3d BUBBLE_ROT = new Rotation3d(0, Units.degreesToRadians(-120), 0);
  private static final int PRELOAD_QTY = 8;

  private static final double INTAKE_ROLLER_RADIUS = Units.inchesToMeters(1);
  private static final double TRANSFER_RADIUS = Units.inchesToMeters(3);
  private static final double LAUNCHER_WHEEL_RADIUS = Units.inchesToMeters(2);
  private static final double LAUNCH_EFFICIENCY = 0.5;
  private static final double BUBBLE_TO_SPINDEXER = Units.inchesToMeters(6.379881);

  private List<HeldSpeechBubble> bubbles = new ArrayList<>();
  private HeldSpeechBubble[] spindexerSlots = new HeldSpeechBubble[SPINDEXER_CAPACITY];
  private boolean transferFull = false;

  private final DoubleSupplier intakeVelocitySupplier;
  private final DoubleSupplier spindexerPositionSupplier;
  private final DoubleSupplier transferVelocitySupplier;
  private final DoubleSupplier shooterVelocitySupplier;
  private final DoubleSupplier hoodAngleSupplier;
  private final DoubleSupplier turretAngleSupplier;
  private final Supplier<Transform3d> hoodTransformSupplier;

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  public HeldFuelManager(
      DoubleSupplier intakeVelocitySupplier,
      DoubleSupplier spindexerPositionSupplier,
      DoubleSupplier transferVelocitySupplier,
      DoubleSupplier shooterVelocitySupplier,
      DoubleSupplier hoodAngleSupplier,
      DoubleSupplier turretAngleSupplier,
      Supplier<Transform3d> hoodTransformSupplier,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldRelativeChassisSpeedsSupplier) {

    this.intakeVelocitySupplier = intakeVelocitySupplier;
    this.spindexerPositionSupplier = spindexerPositionSupplier;
    this.transferVelocitySupplier = transferVelocitySupplier;
    this.shooterVelocitySupplier = shooterVelocitySupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
    this.turretAngleSupplier = turretAngleSupplier;
    this.hoodTransformSupplier = hoodTransformSupplier;

    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = fieldRelativeChassisSpeedsSupplier;

    for (int i = 0; i < PRELOAD_QTY; i++) addFuelToIntake();
  }

  public void periodic() {
    double intakeVel = intakeVelocitySupplier.getAsDouble() * INTAKE_ROLLER_RADIUS;
    double spindexerPos = spindexerPositionSupplier.getAsDouble();
    double transferVel = transferVelocitySupplier.getAsDouble() * TRANSFER_RADIUS;
    double shooterVel =
        shooterVelocitySupplier.getAsDouble() * LAUNCHER_WHEEL_RADIUS * LAUNCH_EFFICIENCY;
    Pose2d robotPose = poseSupplier.get();

    var iterator = bubbles.iterator();
    List<Pose3d> blueHeldbubblePoses = new ArrayList<>();
    List<Pose3d> redHeldbubblePoses = new ArrayList<>();
    while (iterator.hasNext()) {
      HeldSpeechBubble b = iterator.next();
      boolean shouldEject =
          b.update(Constants.LOOP_PERIOD, intakeVel, spindexerPos, transferVel, shooterVel);
      if (shouldEject) {
        iterator.remove();
        launch(shooterVel, b.isRed);
      } else if (b.isRed) {
        redHeldbubblePoses.add(new Pose3d(robotPose).plus(b.transform));
      } else {
        blueHeldbubblePoses.add(new Pose3d(robotPose).plus(b.transform));
      }
    }

    Logger.recordOutput(
        "GamePieceManager/Held Red Bubbles",
        redHeldbubblePoses.toArray(new Pose3d[redHeldbubblePoses.size()]));
    Logger.recordOutput(
        "GamePieceManager/Held Blue Bubbles",
        blueHeldbubblePoses.toArray(new Pose3d[blueHeldbubblePoses.size()]));
    Logger.recordOutput(
        "GamePieceManager/Test Bubble",
        new Pose3d(robotPose).transformBy(getBubbleInShooterTransform(4)));
  }

  public void addFuelToIntake() {
    bubbles.add(new HeldSpeechBubble(false));
  }

  private void launch(double shooterVel, boolean isRed) {
    Transform3d shooterTransform = getBubbleInShooterTransform(4);
    Rotation2d turretRotation =
        Rotation2d.fromRadians(-turretAngleSupplier.getAsDouble() - Math.PI);

    RobotContainer.fuelSim.launchFuel(
        MetersPerSecond.of(shooterVel),
        Radians.of(hoodAngleSupplier.getAsDouble()),
        turretRotation.getMeasure(),
        shooterTransform.getMeasureZ());
  }

  private int getOpenSpindexerSlot() {
    for (int i = 0; i < spindexerSlots.length; i++) {
      if (spindexerSlots[i] == null) {
        return i;
      }
    }
    return -1;
  }

  private int getSpindex(HeldSpeechBubble bubble) {
    for (int i = 0; i < spindexerSlots.length; i++) {
      if (spindexerSlots[i] == bubble) {
        return i;
      }
    }
    return -1;
  }

  private boolean isSpindexSlotNearTransfer(int slot, double spindexerPos) {
    if (slot < 0 || slot >= spindexerSlots.length) return false;

    double angle = MathUtil.angleModulus(spindexerPos);
    double slotAngle = MathUtil.angleModulus(Math.PI / 2.0 + Units.degreesToRadians(slot * 72));

    double error = MathUtil.angleModulus(angle - slotAngle);
    return error < Units.degreesToRadians(54);
  }

  private Transform3d getSpindexerTransform(double spindexerYaw) {
    return new Transform3d(Translation3d.kZero, new Rotation3d(0, 0, -spindexerYaw));
  }

  private Transform3d getBubbleInSpindexerTransform(double spindexerYaw, int slot) {
    double slotAngle = Math.PI / 2.0 + Units.degreesToRadians(slot * 72);
    return getSpindexerTransform(spindexerYaw)
        .plus(
            new Transform3d(
                new Translation3d(BUBBLE_TO_SPINDEXER, 0, 0.2)
                    .rotateBy(new Rotation3d(0, 0, -spindexerYaw + slotAngle)),
                new Rotation3d(0, 0, -spindexerYaw + slotAngle)));
  }

  private Transform3d getBubbleInShooterTransform(double x) {
    double pitch = Units.degreesToRadians(-(6 * x) * (x - 4));
    return hoodTransformSupplier
        .get()
        .plus(
            new Transform3d(BUBBLE_TRANSLATION.rotateBy(new Rotation3d(0, pitch, 0)), BUBBLE_ROT));
  }

  class HeldSpeechBubble {
    public Location location;
    public double x;
    public Transform3d transform;
    public boolean isRed;

    public HeldSpeechBubble(boolean isRed) {
      this.isRed = isRed;

      location = Location.INTAKE;
      x = 0;
      transform = Transform3d.kZero;
    }

    public boolean update(
        double dt, double intakeVel, double spindexerPos, double transferVel, double shooterVel) {
      switch (location) {
        case INTAKE -> {
          x += intakeVel * dt;
          if (x > 1) {
            int slot = getOpenSpindexerSlot();
            if (slot < 0) {
              x = 1.01;
              break;
            }
            spindexerSlots[slot] = this;
            location = Location.SPINDEXER;
            break;
          }
          if (x < 0) {
            x = 0; // todo implement eject out intake
          }
          transform = new Transform3d(0, 0.5 - x * 0.3, 0.2 * x, Rotation3d.kZero);
          break;
        }
        case SPINDEXER -> {
          int index = getSpindex(this);
          if (index < 0) {
            location = Location.INTAKE;
            x = 0.5;
            break;
          }
          if (isSpindexSlotNearTransfer(index, spindexerPos)
              && transferVel > 0.5
              && !transferFull) {
            spindexerSlots[index] = null;
            location = Location.TRANSFER;
            x = 2;
            transferFull = true;
            break;
          }
          transform = getBubbleInSpindexerTransform(spindexerPos, index);
          break;
        }
        case TRANSFER -> {
          x += transferVel * dt;
          if (x > 3) {
            location = Location.SHOOTER;
            transferFull = false;
            break;
          }
          if (x < 2) {
            int slot = getOpenSpindexerSlot();
            if (slot < 0) {
              x = 1.01;
              location = Location.INTAKE;
            } else {
              spindexerSlots[slot] = this;
              location = Location.SPINDEXER;
            }
            transferFull = false;
            break;
          }
          transform = new Transform3d(0, -0.2, 0.2 + (x - 2) * 0.4, Rotation3d.kZero);
          break;
        }
        case SHOOTER -> {
          x += shooterVel * dt;
          if (x > 4) {
            return true;
          }
          transform = getBubbleInShooterTransform(x);
        }
      }
      return false;
    }
  }

  public enum Location {
    INTAKE, // 0-1
    SPINDEXER, // 1-2
    TRANSFER, // 2-3
    SHOOTER // 3-4
  }
}
