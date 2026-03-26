package frc.robot.commands.tuning;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveTuningCommands;

public class TuningCommands {
    public static LoggedDashboardChooser<Command> constructTuningChooser(RobotContainer robotContainer) {
        LoggedDashboardChooser<Command> testChooser = new LoggedDashboardChooser<>("Test Command");
        testChooser.addOption("Turret: Auto tune", robotContainer.turret.runTuning());
        testChooser.addOption("Turret: Oscillation test", robotContainer.turret.runOscillationTest());

        DriveTuningCommands.addTuningCommandsToChooser(robotContainer.drive, testChooser);
        VisionTuningCommands.addTuningCommandsToChooser(robotContainer.vision, testChooser);

        return testChooser;
    }
}
