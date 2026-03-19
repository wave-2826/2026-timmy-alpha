package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
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

        autoFactory.bind("deployIntake", intake.deployIntake())
            .bind("startIntake", intake.runRollerPercent(0.20))
            .bind("stopIntake", intake.runRollerPercent(0.0))
            .bind("climbUp", Commands.none())
            .bind("climbDown", Commands.none());

        autoChooser.addRoutine("Right Swipe Outpost", this::getRightSwipeOutpost);
        autoChooser.addRoutine("Right Swipe Climb Right", this::getRightSwipeClimbRight);
        autoChooser.addRoutine("Right Swipe Climb Left", this::getRightSwipeClimbLeft);
        // autoChooser.addRoutine("Right Sweep Climb Right", null);
        // autoChooser.addRoutine("Right Sweep Climb Left", null);
        // autoChooser.addRoutine("Left Double Swipe", null);
        // autoChooser.addRoutine("Left Swipe Climb Right", null);
        // autoChooser.addRoutine("Left Swipe Climb Left", null);
        // autoChooser.addRoutine("Left Sweep Climb Right", null);
        // autoChooser.addRoutine("Left Sweep Climb Left", null);
        // autoChooser.addRoutine("Left Sweep Outpost", null);

        autoChooser.addRoutine("4-piece L1", this::get4Piece);

        // double correctionRemoveMeWhenItActuallyWorks = 4;
        // double distanceInches = 90 - 3 - 1 + correctionRemoveMeWhenItActuallyWorks;
        // double timeSeconds = 1.5;
        // autoChooser.addCmd("1-piece center", () -> Commands.sequence(
        //     DriveCommands.driveStraightCommand(drive, Units.inchesToMeters(distanceInches / timeSeconds),
        //         RobotState.getInstance()::getRotation).withTimeout(timeSeconds),
        //     roller.runPercent(0.5).withTimeout(1.5)));
    }

    private AutoRoutine getRightSwipeOutpost() {
        var routine = autoFactory.newRoutine("RightSwipeOutpost Auto");

        AutoTrajectory traj0 = routine.trajectory("RightSwipeOutpost");

        traj0.atTime("deployIntake").onTrue(intake.deployIntake());

        routine.active().onTrue(Commands.print("Started the routine!"));
        routine.active().onTrue(Commands.sequence(
            traj0.resetOdometry(),
            traj0.cmd()
        ));

        return routine;
    }

    private AutoRoutine getRightSwipeClimbRight() {
        var routine = autoFactory.newRoutine("Right Swipe Climb Right");

        AutoTrajectory traj0 = routine.trajectory("RightSwipeClimb", 0);
        AutoTrajectory traj1 = routine.trajectory("RightSwipeClimb", 1);
        AutoTrajectory traj2 = routine.trajectory("RightSwipeClimb", 2);
        AutoTrajectory traj3 = routine.trajectory("RightSwipeClimb", 3);
        AutoTrajectory traj4 = routine.trajectory("RightSwipeClimb", 4);
        AutoTrajectory traj5 = routine.trajectory("RightSwipeClimb", 5);
        AutoTrajectory climb0 = routine.trajectory("ClimbRight", 0);
        AutoTrajectory climb1 = routine.trajectory("ClimbRight", 1);

        routine.active().onTrue(Commands.sequence(
            traj0.resetOdometry(),
            traj0.cmd(),
            traj1.cmd(),
            traj2.cmd(),
            traj3.cmd(),
            traj4.cmd(),
            traj5.cmd(),
            climb0.cmd(),
            climb1.cmd()
        ));

        return routine;
    }

    private AutoRoutine getRightSwipeClimbLeft() {
        var routine = autoFactory.newRoutine("Right Swipe Climb Left");

        AutoTrajectory traj0 = routine.trajectory("RightSwipeClimb", 0);
        AutoTrajectory traj1 = routine.trajectory("RightSwipeClimb", 1);
        AutoTrajectory traj2 = routine.trajectory("RightSwipeClimb", 2);
        AutoTrajectory traj3 = routine.trajectory("RightSwipeClimb", 3);
        AutoTrajectory traj4 = routine.trajectory("RightSwipeClimb", 4);
        AutoTrajectory traj5 = routine.trajectory("RightSwipeClimb", 5);
        AutoTrajectory climb0 = routine.trajectory("ClimbLeft", 0);
        AutoTrajectory climb1 = routine.trajectory("ClimbLeft", 1);

        routine.active().onTrue(Commands.sequence(
            traj0.resetOdometry(),
            traj0.cmd(),
            traj1.cmd(),
            traj2.cmd(),
            traj3.cmd(),
            traj4.cmd(),
            traj5.cmd(),
            climb0.cmd(),
            climb1.cmd()
        ));

        return routine;
    }

    private AutoRoutine get4Piece() {
        var routine = autoFactory.newRoutine("4-piece L1");

        AutoTrajectory firstPiece = routine.trajectory("4-piece L1", 0);
        AutoTrajectory secondPiece = routine.trajectory("4-piece L1", 1);
        AutoTrajectory thirdPiece = routine.trajectory("4-piece L1", 2);
        // AutoTrajectory fourthPiece = routine.trajectory("4-piece L1", 3);

        // @formatter:off
        routine.active().onTrue(Commands.sequence(
            firstPiece.resetOdometry(),
            firstPiece.cmd(),
            Commands.waitSeconds(2.0),
            secondPiece.cmd(),
            Commands.waitSeconds(2.0),
            thirdPiece.cmd()
        ));
        // @formatter:on

        return routine;
    }


}