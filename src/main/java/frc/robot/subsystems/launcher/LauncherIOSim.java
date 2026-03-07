// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.servohub.ServoHubSim;
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
  private ServoHubSim hoodServoHubSim;

  public LauncherIOSim() {
    super();
    launcherSimState = launcher1Leader.getSimState();
    hoodServoHubSim = new ServoHubSim(hoodServoHub);
    hoodServoHubSim.enable();
  }

  public void updateInputs(LauncherIOInputs inputs) {
    super.updateInputs(inputs);

    updateSim();
  }

  public void updateSim() {
    launcherSimState.setSupplyVoltage(12);
    launcherPhysicsSim.setInputVoltage(launcherSimState.getMotorVoltage());

    launcherSimState.setRawRotorPosition(
        Units.radiansToRotations(launcherPhysicsSim.getAngleRads()));
    launcherSimState.setRotorVelocity(
        Units.radiansPerSecondToRotationsPerMinute(launcherPhysicsSim.getVelocityRadPerSec()) / 60);

    launcherPhysicsSim.update(Constants.UPDATE_FREQ_SEC);
    hoodServoHubSim.setServoVoltage(12);
  }
}
