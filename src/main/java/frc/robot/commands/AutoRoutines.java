package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.generated.autos.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.hopperVision.HopperVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.Turret.TurretTarget;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedAutoChooser;
import frc.robot.util.GenericPIDConstants.PIDSlot;
import frc.robot.util.simUtils.Simulation;

public class AutoRoutines {
    private final AutoFactory autoFactory;
    private final Drive drive;
    private final Intake intake;
    private final Turret turret;
    private final Spindexer spindexer;
    private final HopperVision hopperVision;

    /** The latest trajectory target. If null, no trajectory has been followed yet. */
    private Pose2d latestTrajectoryTarget = null;

    private PIDController xController = new PIDController(0, 0, 0);
    private PIDController yController = new PIDController(0, 0, 0);
    private PIDController thetaController = new PIDController(0, 0, 0);

    public AutoRoutines(RobotContainer rc, LoggedAutoChooser autoChooser) {
        DriveConstants.autoLinearPID.configureController(xController, PIDSlot.Slot0);
        DriveConstants.autoLinearPID.configureController(yController, PIDSlot.Slot0);
        DriveConstants.autoAngularPID.configureController(thetaController, PIDSlot.Slot0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.drive = rc.drive;
        this.intake = rc.intake;
        this.turret = rc.turret;
        this.spindexer = rc.spindexer;
        this.hopperVision = rc.hopperVision;

        autoFactory = createAutoFactory();

        if(Constants.isSim) {
            var traj = ChoreoTraj.ALL_TRAJECTORIES.getOrDefault(autoChooser.selectedCommand().getName(), null);
            if(traj != null) Simulation.getInstance().driveSimulation.setSimulationWorldPose(AllianceFlipUtil.apply(traj.initialPoseBlue()));
        }
        
        autoChooser.addRoutine("Left Double Swipe", () -> this.getDoubleSwipe(false));
        autoChooser.addRoutine("Right Double Swipe", () -> this.getDoubleSwipe(true));
        
        autoChooser.addRoutine("Left Single Swipe", () -> this.getSingleSwipe(false));
        autoChooser.addRoutine("Right Single Swipe", () -> this.getSingleSwipe(true));

        autoChooser.addRoutine("Left Sweep Swipe", () -> this.getSweepSwipe(false));
        autoChooser.addRoutine("Right Sweep Swipe", () -> this.getSweepSwipe(true));
        
        autoChooser.addRoutine("Left Danger Sweep Swipe", () -> this.getDangerSweepSwipe(false));
        autoChooser.addRoutine("Right Danger Sweep Swipe", () -> this.getDangerSweepSwipe(true));

        autoChooser.addRoutine("Center Preload Simplified", () -> this.getCenterPreload(true));
        autoChooser.addRoutine("Center Preload", () -> this.getCenterPreload(false));
        autoChooser.addRoutine("Center Depot", () -> this.getCenterDepot(), true);
        autoChooser.addCmd("Shoot Only (intake facing DS)", () -> this.getShootOnly());
    }

    
    /**
     * Creates a new auto factory for this drivetrain with the given trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        var robotState = RobotState.getInstance();
        return new AutoFactory(
            robotState::getEstimatedPose,
            drive::setPose,
            this::followPath,
            true,
            drive,
            (traj, isStart) -> {
                Logger.recordOutput("Odometry/Trajectory", traj.getPoses());
                Logger.recordOutput("Odometry/IsStart", isStart);
            }
        );
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        latestTrajectoryTarget = sample.getPose();

        var baseSpeeds = sample.getChassisSpeeds();

        double[] forcesN = new double[4];
        for(int i = 0; i < 4; i++) {
            double forceX = sample.moduleForcesX()[i];
            double forceY = sample.moduleForcesY()[i];
            forcesN[i] = Math.sqrt(forceX * forceX + forceY * forceY);
        }

        followPathToTarget(latestTrajectoryTarget, forcesN, baseSpeeds);
    }

    public void followPathToTarget(Pose2d targetPose, double[] ffForcesN, ChassisSpeeds baseSpeeds) {
        var pose = RobotState.getInstance().getEstimatedPose();

        Logger.recordOutput("Odometry/Auto/CurrentPose", pose);
        Logger.recordOutput("Odometry/Auto/TargetPose", targetPose);
        Logger.recordOutput("Odometry/Auto/TranslationError", targetPose.getTranslation().minus(pose.getTranslation()).getNorm());
        Logger.recordOutput("Odometry/Auto/RotationErrorDeg", targetPose.getRotation().minus(pose.getRotation()).getDegrees());

        baseSpeeds.vxMetersPerSecond += xController.calculate(pose.getX(), targetPose.getX());
        baseSpeeds.vyMetersPerSecond += yController.calculate(pose.getY(), targetPose.getY());
        baseSpeeds.omegaRadiansPerSecond += thetaController.calculate(pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(baseSpeeds, RobotState.getInstance().getRotation()),
            ffForcesN, false
        );
    }

    private Command stopDrive() {
        return Commands.runOnce(() -> drive.stop());
    }

    private AutoRoutine getDoubleSwipe(boolean right) {
        var fullTraj = right ? ChoreoTraj.RightDoubleSwipeGenerated : ChoreoTraj.LeftDoubleSwipe;
        var routine = autoFactory.newRoutine(fullTraj.name());
        
        var chorTraj0 = right ? ChoreoTraj.RightDoubleSwipeGenerated$0 : ChoreoTraj.LeftDoubleSwipe$0;
        var chorTraj1 = right ? ChoreoTraj.RightDoubleSwipeGenerated$1 : ChoreoTraj.LeftDoubleSwipe$1;
        AutoTrajectory traj0 = chorTraj0.asAutoTraj(routine);
        AutoTrajectory traj1 = chorTraj1.asAutoTraj(routine);

        traj0.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));

        routine.active().onTrue(Commands.sequence(
            traj0.resetOdometry(),
            ScoringCommands.prep(turret),
            traj0.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision)),
            traj1.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision))
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
            ScoringCommands.prep(turret),
            traj.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision))
        ));

        return routine;
    }

    private AutoRoutine getSweepSwipe(boolean right) {
        var choreoTraj = right ? ChoreoTraj.RightSweepSwipeGenerated : ChoreoTraj.LeftSweepSwipe;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        var choreoTraj1 = right ? ChoreoTraj.RightSweepSwipeGenerated$0 : ChoreoTraj.LeftSweepSwipe$0;
        var choreoTraj2 = right ? ChoreoTraj.RightSweepSwipeGenerated$1 : ChoreoTraj.LeftSweepSwipe$1;
        AutoTrajectory traj1 = choreoTraj1.asAutoTraj(routine);
        AutoTrajectory traj2 = choreoTraj2.asAutoTraj(routine);

        traj1.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        traj1.atTime("Intake Stop").onTrue(intake.disable());
        
        routine.active().onTrue(Commands.sequence(
            traj1.resetOdometry(),
            ScoringCommands.prep(turret),
            traj1.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision)),
            traj2.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision))
        ));

        return routine;
    }

    private AutoRoutine getDangerSweepSwipe(boolean right) {
        var choreoTraj = right ? ChoreoTraj.RightDangerSweepGenerated : ChoreoTraj.LeftDangerSweep;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        var choreoTraj1 = right ? ChoreoTraj.RightDangerSweepGenerated$0 : ChoreoTraj.LeftDangerSweep$0;
        var choreoTraj2 = right ? ChoreoTraj.RightDangerSweepGenerated$1 : ChoreoTraj.LeftDangerSweep$1;
        AutoTrajectory traj1 = choreoTraj1.asAutoTraj(routine);
        AutoTrajectory traj2 = choreoTraj2.asAutoTraj(routine);

        traj1.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        traj1.atTime("Intake Stop").onTrue(intake.disable());
        
        routine.active().onTrue(Commands.sequence(
            traj1.resetOdometry(),
            ScoringCommands.prep(turret),
            traj1.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision)),
            traj2.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision))
        ));

        return routine;
    }

    private AutoRoutine getCenterPreload(boolean noTurretControl) {
        var choreoTraj = ChoreoTraj.CenterPreload;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            ScoringCommands.prep(turret),
            traj.cmd(),
            stopDrive(),
            noTurretControl ? Commands.sequence(
                Commands.runOnce(() -> {
                    turret.target = new TurretTarget(
                        Units.rotationsPerMinuteToRadiansPerSecond(4225),
                        0.0,
                        TurretConstants.hoodMinAngle
                    );
                }),

                Commands.waitSeconds(1.0),

                Commands.run(() -> {
                    spindexer.setPower(0.0, 1.0);
                }).withTimeout(0.5),
                
                Commands.run(() -> {
                    spindexer.setPower(
                        // Oscillation to unstuck pieces
                        (Math.sin(Timer.getFPGATimestamp() * 4) * 0.75 + 0.25) * 1.0,
                        1.0
                    );
                }).withTimeout(10)
            ) : ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision)
        ));

        return routine;
    }

    private AutoRoutine getCenterDepot() {
        var choreoTraj = ChoreoTraj.CenterDepot;
        var routine = autoFactory.newRoutine(choreoTraj.name());
        
        AutoTrajectory traj = choreoTraj.asAutoTraj(routine);

        traj.atTime("Intake").onTrue(Commands.sequence(intake.deployIntake(), intake.enable()));
        
        routine.active().onTrue(Commands.sequence(
            traj.resetOdometry(),
            ScoringCommands.prep(turret),
            traj.cmd(),
            stopDrive().alongWith(ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision))
        ));

        return routine;
    }

    private Command getShootOnly() {
        return Commands.sequence(
            Commands.runOnce(() -> drive.setPose(new Pose2d(new Translation2d(2., FieldConstants.fieldWidthY / 2.), Rotation2d.kZero))),
            ScoringCommands.autoScoreHopper(turret, spindexer, hopperVision)
        );
    }
}