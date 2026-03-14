package frc.robot;

import choreo.auto.AutoChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
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
        var selected = autoChooser.get().getName();
        return selected == null || selected.equals("None");
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
                intake = new Intake(new IntakeIOReal() {});
                turret = new Turret(new TurretIO() {});
                climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIOReal());
                break;
            case SIM:
                intake = new Intake(new IntakeIO() {});

                var driveSimulation = Simulation.getInstance().configureSimulation(intake);

                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOSim(DriveConstants.frontLeftConfig),
                    new ModuleIOSim(DriveConstants.frontRightConfig),
                    new ModuleIOSim(DriveConstants.backLeftConfig),
                    new ModuleIOSim(DriveConstants.backRightConfig));
                driveSimulation.setModuleStateSupplier(drive::getModuleStates);
                Simulation.getInstance().setDrive(drive);

                vision = new Vision(
                    new VisionIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                turret = new Turret(new TurretIOSim());
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
        autoChooser = new LoggedDashboardChooser<>("Auto");
        autoChooser.addDefaultOption("None", Commands.none().withName("None"));

        for(AutoPaths path : AutoCommands.AutoPaths.values()) {
            var name = path.name();
            name = name.replaceAll("_", " ");
            autoChooser.addOption(name, AutoCommands.runAuto(path, this));
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
