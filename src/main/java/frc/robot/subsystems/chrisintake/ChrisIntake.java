package frc.robot.subsystems.chrisintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

class ChrisIntake extends SubsystemBase {
  // We are going to follow the AdvantageKit IO pattern for this subsystem
  // We have 2 pivot motors and 2 roller motors
  // Kraken X44 motors
  private ChrisIntakeIO io;

  public ChrisIntake(ChrisIntakeIO io) {
    this.io = io;
  }
}
