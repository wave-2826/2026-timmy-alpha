package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoCommands.AutoPaths;
import frc.robot.commands.tuning.TuningCommands;
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
                    new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                    new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1));
                intake = new Intake(new IntakeIO() {});
                turret = new Turret(new TurretIOReal());
                climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIOReal());
                break;
            case SIM:
                var driveSimulation = Simulation.getInstance().configureSimulation();

                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(0),
                    new ModuleIOSim(1),
                    new ModuleIOSim(2),
                    new ModuleIOSim(3));
                driveSimulation.setModuleStateSupplier(drive::getModuleStates);

                vision = new Vision(
                    new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                turret = new Turret(new TurretIOSim());
                intake = new Intake(new IntakeIO() {});
                climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIO() {});
                
                drive.setPose(new Pose2d(3, 3, new Rotation2d()));
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
                intake = new Intake(new IntakeIO() {});
                turret = new Turret(new TurretIO() {});
                climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIO() {});
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        for(AutoPaths path : AutoCommands.AutoPaths.values()) {
            var name = path.name();
            name = name.replaceAll("_", " ");
            autoChooser.addOption(name, AutoCommands.runCodeCommand(path, drive));
        }

        Controls.getInstance().configureControls(this);
        testChooser = TuningCommands.constructTuningChooser(this);

        if(Constants.isSim) Simulation.getInstance().resetSimulationField();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
    public Command getTestCommand() {
        return testChooser.get();
    }
}
