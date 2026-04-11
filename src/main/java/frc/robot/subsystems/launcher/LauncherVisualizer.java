// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.CircleSim;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class LauncherVisualizer {
  private final LoggedMechanism2d mech2d = new LoggedMechanism2d(3, 3);

  private final LoggedMechanismRoot2d root = mech2d.getRoot("Launcher", 1, 1.5);
  private final LoggedMechanismRoot2d root2 = mech2d.getRoot("fda", 1, 1.5);

  private final CircleSim flywheelSim =
      new CircleSim(
          root,
          LauncherConstants.ROLLER_SEGMENT_COUNT,
          LauncherConstants.ROLLER_RADIUS_METERS,
          LauncherConstants.SIM_LINE_WIDTH,
          new Color8Bit(Color.kDarkSlateGray),
          new Color8Bit(Color.kSeaGreen),
          new Color8Bit(Color.kGoldenrod));

  private LoggedMechanismLigament2d hood =
      root2.append(
          new LoggedMechanismLigament2d(
              "Hood",
              1,
              LauncherConstants.HOOD_MIN_ANGLE_RADS,
              LauncherConstants.SIM_LINE_WIDTH,
              new Color8Bit(Color.kDimGray)));

  private double previousRollerAngle = 0.0;
  private double rollerAngleDeg = 0.0;
  private double hoodAngleDeg = 0.0;

  private static final LauncherVisualizer instance = new LauncherVisualizer();

  public static LauncherVisualizer getInstance() {
    return instance;
  }

  private LauncherVisualizer() {}

  public void periodic() {
    SmartDashboard.putData("Launcher Sim", mech2d);
  }

  public void updateFlywheelPositionDeg(double angleDeg) {
    rollerAngleDeg =
        angleDeg / 10; // reducing the speed of the flywheel for more clarity in visualizatoin
    flywheelSim.updateAngleDeg(rollerAngleDeg, previousRollerAngle);
    previousRollerAngle = rollerAngleDeg;
  }

  public void updateHoodPositionDeg(double angleDeg) {
    // this.hoodAngleDeg = angleDeg + Units.radiansToDegrees(LauncherConstants.HOOD_MIN_ANGLE_RADS);
    this.hoodAngleDeg = angleDeg;
    hood.setAngle(hoodAngleDeg + Units.radiansToDegrees(LauncherConstants.HOOD_MIN_ANGLE_RADS));
  }
}
