package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.autos.ChoreoTraj;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.LoggedAutoChooser;

public class AutoRoutines {
    private final AutoFactory autoFactory;
    private final Drive drive;
    private final Intake intake;

    public AutoRoutines(Drive drive, Intake intake, Spindexer spindexer, Climber climber, Turret turret, LoggedAutoChooser autoChooser) {
        this.drive = drive;
        this.intake = intake;

        autoFactory = drive.createAutoFactory((traj, isStart) -> {
            Logger.recordOutput("Odometry/Trajectory", traj.getPoses());
            Logger.recordOutput("Odometry/IsStart", isStart);
        });

        // .bind("Intake Stop", intake.disable())
        // .bind("Outtake", intake.enableOutward());
        
        autoChooser.addRoutine("Left Double Swipe", () -> this.getDoubleSwipe(false));
        // autoChooser.addRoutine("Right Double Swipe (fallback)", () -> this.getDoubleSwipe(true));
    }

    private AutoRoutine getDoubleSwipe(boolean right) {
        var routine = autoFactory.newRoutine("LeftDoubleSwipe");
        
        AutoTrajectory traj = ChoreoTraj.LeftDoubleSwipe.asAutoTraj(routine);

        traj.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        System.out.println("Got routine");

        return routine;
    }

}