package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LoggedTunableNumber;

public class DemoLaunching {

    private static final LoggedTunableNumber velocityVariation = new LoggedTunableNumber("Demo/velocity max variance rps", 10.0);
    private static final LoggedTunableNumber yawRange = new LoggedTunableNumber("Demo/yaw max variance deg", 5.0);
    private static final LoggedTunableNumber waitTime = new LoggedTunableNumber("Demo/wait time s", 1.0);

    public static Command launchRandomly(
        Launcher launcher,
        Turret turret,
        Feeder feeder,
        Spindexer spindexer) {
        
        return Commands.sequence(
            Commands.runOnce(() -> { // initialization code
                launcher.setManual();
                turret.goToTestSetpoint();
            }),
            Commands.repeatingSequence( // execute code
                Commands.runOnce(() -> {
                    launcher.setAdjust(Math.random() * velocityVariation.get());
                    turret.setRotationAdjust(Math.random() * Units.degreesToRadians(yawRange.get()));
                }, launcher, turret),

                new WaitCommand(waitTime.get() * 4/5),

                Commands.runOnce(() -> {
                    feeder.setRunning();
                    spindexer.setRunning();
                }, feeder, spindexer),

                new WaitCommand(waitTime.get() * 1/5),

                Commands.runOnce(() -> {
                    spindexer.setStopped();
                    feeder.setStopped();
                }, feeder, spindexer)
            )
        ).finallyDo(() -> {
            spindexer.setStopped();
            feeder.setStopped();
        });
    }
}
