package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveTuningCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.simUtils.Simulation;
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
    public final Intake intake;
    public final Climber climber;
    public final Spindexer spindexer;
    public final Turret turret;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    public boolean noAutoSelected() {
        var selected = autoChooser.getSendableChooser().getSelected();
        return selected == null || selected == "None";
    }

    private final LoggedDashboardChooser<Command> testChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
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
                    new VisionIOPhotonVision(VisionConstants.camera0Name, robotToCamera0),
                    new VisionIOPhotonVision(VisionConstants.camera1Name, robotToCamera1));
                intake = new Intake(new IntakeIOReal());
                turret = new Turret(new TurretIOReal());
                // climber = new Climber(new ClimberIOReal());
                climber = new Climber(new ClimberIOReal());
                spindexer = new Spindexer(new SpindexerIOReal());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                var driveSimulation = Simulation.getInstance().configureSimulation();
                
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
                intake = new Intake(new IntakeIO() {});
                climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIO() {});
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
                intake = new Intake(new IntakeIO() {});
                climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIO() {});
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        Controls.getInstance().configureControls(this);

        testChooser = new LoggedDashboardChooser<>("Test Command");
        testChooser.addDefaultOption("Zero module rotations", drive.rezeroModules());
        testChooser.addOption("Auto tune turret", turret.runTuning());
        DriveTuningCommands.addTuningCommandsToAutoChooser(drive, testChooser);

        if(Constants.isSim) Simulation.getInstance().resetSimulationField();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
    public Command getTestCommand() {
        return testChooser.get();
    }
}
