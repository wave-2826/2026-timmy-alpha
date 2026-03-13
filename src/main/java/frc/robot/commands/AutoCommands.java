package frc.robot.commands;

import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.simUtils.Simulation;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;

public class AutoCommands {
    public static enum AutoPaths {
        // SWIPE = Go out to center grab and come back
        // SWEEP = Go out and go across and come back
        RIGHT_SWIPE_OUTPOST,
        RIGHT_SWIPE_CLIMB_RIGHT,
        RIGHT_SWIPE_CLIMB_LEFT,
        RIGHT_SWEEP_CLIMB_RIGHT,
        RIGHT_SWEEP_CLIMB_LEFT,
        LEFT_DOUBLE_SWIPE,
        LEFT_SWIPE_CLIMB_RIGHT,
        LEFT_SWIPE_CLIMB_LEFT,
        LEFT_SWEEP_CLIMB_RIGHT,
        LEFT_SWEEP_CLIMB_LEFT,
        LEFT_SWEEP_OUTPOST,
    };
    
    public static Command runAuto(AutoPaths path, RobotContainer rc) {
        switch (path) {
            case RIGHT_SWIPE_OUTPOST:
            case RIGHT_SWIPE_CLIMB_RIGHT:
            case RIGHT_SWIPE_CLIMB_LEFT:
            case RIGHT_SWEEP_CLIMB_RIGHT:
            case RIGHT_SWEEP_CLIMB_LEFT:
            case LEFT_DOUBLE_SWIPE:
            case LEFT_SWIPE_CLIMB_RIGHT:
            case LEFT_SWIPE_CLIMB_LEFT:
            case LEFT_SWEEP_CLIMB_RIGHT:
            case LEFT_SWEEP_CLIMB_LEFT:
            case LEFT_SWEEP_OUTPOST:
        }
        return Commands.none();
    }
    public static Command resetOdom(boolean isRightSide, Drive drive) {
        final Pose2d defaultpose;
        if (isRightSide == true) {
            defaultpose = new Pose2d(new Translation2d(4.364,0.382), Rotation2d.fromDegrees(90));
        } else {
            defaultpose = new Pose2d(new Translation2d(4.300,7.655), Rotation2d.kZero);
        }
        return Commands.runEnd(() -> {
                Simulation.getInstance().driveSimulation.setSimulationWorldPose(defaultpose);
                drive.setPose(Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose()); // Reset odometry to actual robot pose during simulation
            }, () -> {});
    }
}
