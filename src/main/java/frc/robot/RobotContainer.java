package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveTuningCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Vision vision;
    // public final Intake intake;
    public final Turret turret;

    private SwerveDriveSimulation driveSimulation = null;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    public boolean noAutoSelected() {
        var selected = autoChooser.getSendableChooser().getSelected();
        return selected == null || selected == "None";
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        RobotState robotState = RobotState.getInstance();

        switch(Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFXReal(DriveConstants.frontLeftConfig),
                    new ModuleIOTalonFXReal(DriveConstants.frontRightConfig),
                    new ModuleIOTalonFXReal(DriveConstants.backLeftConfig),
                    new ModuleIOTalonFXReal(DriveConstants.backRightConfig));
                vision = new Vision(
                    new VisionIOLimelight(VisionConstants.camera0Name, robotState::getRotation),
                    new VisionIOLimelight(VisionConstants.camera1Name, robotState::getRotation));
                // intake = new Intake(new IntakeIOSpark());
                turret = new Turret(new TurretIOReal());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                RobotState.getInstance().resetSimulationPoseCallback = driveSimulation::setSimulationWorldPose;
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOTalonFXSim(DriveConstants.frontLeftConfig, driveSimulation.getModules()[0]),
                    new ModuleIOTalonFXSim(DriveConstants.frontRightConfig, driveSimulation.getModules()[1]),
                    new ModuleIOTalonFXSim(DriveConstants.backLeftConfig, driveSimulation.getModules()[2]),
                    new ModuleIOTalonFXSim(DriveConstants.backRightConfig, driveSimulation.getModules()[3]));
                vision = new Vision(
                    new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                // intake = new Intake(new IntakeIOSim());
                turret = new Turret(new TurretIOSim());
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
                vision = new Vision(new VisionIO() {}, new VisionIO() {});
                // intake = new Intake(new IntakeIO() {});
                turret = new Turret(new TurretIO() {});
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        DriveTuningCommands.addTuningCommandsToAutoChooser(drive, autoChooser);
        Controls.getInstance().configureControls(this, driveSimulation);

        resetSimulationField();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
