// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Launcher IO's real implementation */
public class LauncherIOSim extends LauncherIOTalonFX {
  private SingleJointedArmSim launcherPhysicsSim =
      new SingleJointedArmSim(
          LauncherConstants.ROLLER_MOTOR,
          LauncherConstants.LAUNCHER_GEARING,
          LauncherConstants.LAUNCHER_ROLLER_MOI,
          LauncherConstants.ROLLER_RADIUS_METERS,
          Double.NEGATIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          false,
          0);
  private TalonFXSimState launcherSimState;

  private SingleJointedArmSim hoodPhysicsSim =
      new SingleJointedArmSim(
          LauncherConstants.HOOD_MOTOR,
          LauncherConstants.HOOD_GEARING,
          SingleJointedArmSim.estimateMOI(
              LauncherConstants.HOOD_LENGTH_METERS, LauncherConstants.HOOD_MASS_KG),
          LauncherConstants.HOOD_LENGTH_METERS,
          Double.NEGATIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          true,
          LauncherConstants.HOOD_MIN_ANGLE_RADS);
  private TalonFXSimState hoodSimState;

  public LauncherIOSim() {
    super();
    launcherSimState = launcher1Leader.getSimState();
    hoodSimState = hood.getSimState();
  }

  public void updateInputs(LauncherIOInputs inputs) {
    super.updateInputs(inputs);

    launcherSimState.setSupplyVoltage(12);
    launcherPhysicsSim.setInputVoltage(launcherSimState.getMotorVoltage());
    
    hoodSimState.setSupplyVoltage(12);
    hoodPhysicsSim.setInputVoltage(hoodSimState.getMotorVoltage());
    
    launcherPhysicsSim.update(Constants.UPDATE_FREQ_SEC);
    hoodPhysicsSim.update(Constants.UPDATE_FREQ_SEC);

    launcherSimState.setRawRotorPosition(
        Units.radiansToRotations(launcherPhysicsSim.getAngleRads()));
    launcherSimState.setRotorVelocity(
        Units.radiansToRotations(launcherPhysicsSim.getVelocityRadPerSec()));

    hoodSimState.setRawRotorPosition(
        (hoodPhysicsSim.getAngleRads() - LauncherConstants.HOOD_MIN_ANGLE_RADS));
    hoodSimState.setRotorVelocity(
        Units.radiansToRotations(hoodPhysicsSim.getVelocityRadPerSec()));
  }
}
