package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.autos.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopperVision.HopperVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedAutoChooser;
import frc.robot.util.simUtils.Simulation;

public class AutoRoutines {
    private final AutoFactory autoFactory;
    private final Drive drive;
    private final Intake intake;
    private final Turret turret;
    private final Spindexer spindexer;
    private final HopperVision hopperVision;

    public AutoRoutines(RobotContainer rc, LoggedAutoChooser autoChooser) {
        this.drive = rc.drive;
        this.intake = rc.intake;
        this.turret = rc.turret;
        this.spindexer = rc.spindexer;
        this.hopperVision = rc.hopperVision;

        autoFactory = drive.createAutoFactory((traj, isStart) -> {
            Logger.recordOutput("Odometry/Trajectory", traj.getPoses());
            Logger.recordOutput("Odometry/IsStart", isStart);
        });

        if(Constants.isSim) {
            var traj = ChoreoTraj.ALL_TRAJECTORIES.getOrDefault(autoChooser.selectedCommand().getName(), null);
            if(traj != null) Simulation.getInstance().driveSimulation.setSimulationWorldPose(AllianceFlipUtil.apply(traj.initialPoseBlue()));
        }

        // .bind("Intake Stop", intake.disable())
        // .bind("Outtake", intake.enableOutward());
        
        autoChooser.addRoutine("Left Double Swipe", () -> this.getDoubleSwipe(false));
        autoChooser.addRoutine("Right Double Swipe", () -> this.getDoubleSwipe(true));
        
        autoChooser.addRoutine("Left Single Swipe", () -> this.getSingleSwipe(false));
        autoChooser.addRoutine("Right Single Swipe", () -> this.getSingleSwipe(true));

        autoChooser.addRoutine("Sweep Outpost (start left)", () -> this.getSweepOutpost());
        
        autoChooser.addRoutine("Center Preload", () -> this.getCenterPreload(), true);
        autoChooser.addRoutine("Center Depot", () -> this.getCenterDepot());
    }

    private AutoRoutine getDoubleSwipe(boolean right) {
        var choreoTraj = right ? ChoreoTraj.RightDoubleSwipeGenerated : ChoreoTraj.LeftDoubleSwipe;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        traj.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        return routine;
    }

    private AutoRoutine getSingleSwipe(boolean right) {
        var choreoTraj = right ? ChoreoTraj.RightDoubleSwipeGenerated$0 : ChoreoTraj.LeftDoubleSwipe$0;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        traj.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        return routine;
    }

    private AutoRoutine getSweepOutpost() {
        var choreoTraj = ChoreoTraj.SweepOutpost;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        traj.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        traj.atTime("Intake Stop").onTrue(intake.disable());
        traj.atTime("Outtake").onTrue(intake.enableOutward());
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        return routine;
    }

    private AutoRoutine getCenterPreload() {
        var choreoTraj = ChoreoTraj.CenterPreload;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        traj.atTime("Shoot").onTrue(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision));
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        return routine;
    }

    private AutoRoutine getCenterDepot() {
        var choreoTraj = ChoreoTraj.CenterDepot;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        traj.atTime("Shoot").onTrue(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision));
        traj.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            traj.cmd()
        ));

        return routine;
    }
}