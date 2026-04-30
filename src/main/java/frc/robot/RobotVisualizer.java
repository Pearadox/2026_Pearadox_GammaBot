package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisualizerConstants;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private final DoubleSupplier turretYawSupplier;
  private final DoubleSupplier hoodAngleSupplier;
  private final DoubleSupplier intakeAngleSupplier;

  @Getter private Transform3d turretTransform = Transform3d.kZero;
  @Getter private Transform3d hoodTransform = Transform3d.kZero;
  @Getter private Transform3d llTransform = Transform3d.kZero;

  public RobotVisualizer(
      DoubleSupplier turretYawSupplier,
      DoubleSupplier hoodAngleSupplier,
      DoubleSupplier intakeAngleSupplier) {
    this.turretYawSupplier = turretYawSupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
    this.intakeAngleSupplier = intakeAngleSupplier;
  }

  public void periodic() {
    double turretYaw = turretYawSupplier.getAsDouble();
    double hoodAngle = hoodAngleSupplier.getAsDouble();
    double intakeAngle = intakeAngleSupplier.getAsDouble();
    double hopperExtensionMeters = getHopperExtension(intakeAngle);

    turretTransform =
        new Transform3d(
            VisualizerConstants.MODEL0_ZERO,
            new Rotation3d(0, 0, -turretYaw + VisualizerConstants.TURRET_STARTING_ANGLE));

    hoodTransform =
        turretTransform.plus(
            new Transform3d(
                VisualizerConstants.MODEL1_OFFSET,
                new Rotation3d(-hoodAngle + VisualizerConstants.HOOD_STARTING_ANGLE, 0, 0)));

    Transform3d intake =
        new Transform3d(
            VisualizerConstants.MODEL2_ZERO,
            new Rotation3d(0, -intakeAngle + VisualizerConstants.INTAKE_STARTING_ANGLE, 0));

    Transform3d hopper =
        new Transform3d(new Translation3d(hopperExtensionMeters, 0, 0), Rotation3d.kZero);

    Logger.recordOutput(
        "RobotVisualizer/Components", new Transform3d[] {turretTransform, hoodTransform, intake, hopper});

    // llTransform =
    //     turret.plus(
    //         new Transform3d(
    //             0,
    //             VisualizerConstants.LIMELIGHT_TO_CENTER_OF_TURRET,
    //             VisualizerConstants.LIMELIGHT_HEIGHT,
    //             new Rotation3d(0, Math.toRadians(-20), Math.toRadians(90))));

    // Logger.recordOutput("RobotVisualizer/LL", llTransform);
  }

  private static double getHopperExtension(double intakeAngle) {
    // return (VisualizerConstants.MAX_HOPPER_EXTENSION / Units.degreesToRadians(125)) * intakeAngle
    // + VisualizerConstants.MAX_HOPPER_EXTENSION;
    return VisualizerConstants.MAX_HOPPER_EXTENSION
            * Math.cos((Math.PI / 2.0 * intakeAngle) / VisualizerConstants.INTAKE_STARTING_ANGLE)
        - VisualizerConstants.MAX_HOPPER_EXTENSION;
  }

  // wip
  public boolean turretUnderOppTrench() {
    // Translation3d tt = turretTransform.getTranslation();
    // double x = Robot.getAlliance() == Alliance.Blue ? FieldConstants.LinesVertical.redHubCenterX : FieldConstants.LinesVertical.blueHubCenterX;
    // if (MathUtil.isNear(x, tt.getX(), VisualizerConstants.IS_UNDER_TRENCH_X_TOLERANCE)
    //   && (MathUtil.(x, x, x))) {
      
    // }
    return false;
  }
}
