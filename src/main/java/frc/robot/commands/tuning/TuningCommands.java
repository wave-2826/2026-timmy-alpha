package frc.robot.commands.tuning;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTuningCommands;

public class TuningCommands {
    public static LoggedDashboardChooser<Command> constructTuningChooser(RobotContainer robotContainer) {
        LoggedDashboardChooser<Command> testChooser = new LoggedDashboardChooser<>("Test Command");
        testChooser.addOption("Turret: Auto tune", robotContainer.turret.runTuning());
        testChooser.addOption("Turret: Oscillation test", robotContainer.turret.runOscillationTest());

        testChooser.addOption("Orchestra", Commands.startEnd(() -> {
            Robot.orchestra.loadMusic("spin.chrp");
            Robot.orchestra.play();
        }, () -> {
            Robot.orchestra.stop();
        }));

        DriveTuningCommands.addTuningCommandsToChooser(robotContainer.drive, testChooser);
        VisionTuningCommands.addTuningCommandsToChooser(robotContainer.drive, robotContainer.vision, testChooser);

        return testChooser;
    }
}
