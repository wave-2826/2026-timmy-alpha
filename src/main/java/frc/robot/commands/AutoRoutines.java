package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

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
    private final Climber climber;
    private final Spindexer spindexer;
    private final Turret turret;

    public AutoRoutines(RobotContainer rc, LoggedAutoChooser autoChooser) {
        drive = rc.drive;
        intake = rc.intake;
        climber = rc.climber;
        spindexer = rc.spindexer;
        turret = rc.turret;

        autoFactory = drive.createAutoFactory((traj, isStart) -> {
            Logger.recordOutput("Odometry/Trajectory", traj.getPoses());
            Logger.recordOutput("Odometry/IsStart", isStart);
        });

        autoFactory.bind("Deploy Intake", intake.deployIntake());
        autoFactory.bind("Start Intaking", intake.runRollerTeleop(() -> 0.20, () -> 0.0));
        autoFactory.bind("Stop Intaking", intake.runRollerTeleop(() -> 0.0, () -> 0.0));
        autoFactory.bind("Deploy Climb", Commands.none());
        autoFactory.bind("Climb Down", Commands.none());

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
        var routine = autoFactory.newRoutine("Right Swipe Outpost");

        AutoTrajectory traj0 = routine.trajectory("RightSwipeOutpost", 0);
        AutoTrajectory traj1 = routine.trajectory("RightSwipeOutpost", 1);
        AutoTrajectory traj2 = routine.trajectory("RightSwipeOutpost", 2);
        AutoTrajectory traj3 = routine.trajectory("RightSwipeOutpost", 3);
        AutoTrajectory traj4 = routine.trajectory("RightSwipeOutpost", 4);
        AutoTrajectory traj5 = routine.trajectory("RightSwipeOutpost", 5);

        routine.active().onTrue(Commands.sequence(
            traj0.resetOdometry(),
            traj0.cmd(),
            traj1.cmd(),
            traj2.cmd(),
            traj3.cmd(),
            traj4.cmd(),
            traj5.cmd()
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