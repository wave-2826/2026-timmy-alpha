package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.tuning.TuningCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.hopperVision.HopperVision;
import frc.robot.subsystems.hopperVision.HopperVisionIO;
import frc.robot.subsystems.hopperVision.HopperVisionIOPhoton;
import frc.robot.subsystems.hopperVision.HopperVisionIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.LEDIO;
import frc.robot.subsystems.leds.LEDIORio;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIO;
import frc.robot.subsystems.spindexer.SpindexerIOReal;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonHighFrequency;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LoggedAutoChooser;
import frc.robot.util.simUtils.Simulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Vision vision;
    public final HopperVision hopperVision;
    public final Intake intake;
    // public final Climber climber;
    public final Spindexer spindexer;
    public final Turret turret;

    private final LEDs leds;

    // Dashboard inputs
    private final AutoRoutines routines;
    public final LoggedAutoChooser autoChooser;
    private final LoggedDashboardChooser<Command> testChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch(Constants.currentMode) {
            case REAL:
                leds = new LEDs(new LEDIORio());
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFXReal(DriveConstants.frontLeftConfig),
                    new ModuleIOTalonFXReal(DriveConstants.frontRightConfig),
                    new ModuleIOTalonFXReal(DriveConstants.backLeftConfig),
                    new ModuleIOTalonFXReal(DriveConstants.backRightConfig));
                vision = new Vision(
                    new VisionIOPhotonVision(VisionConstants.cameraLeftmost),
                    new VisionIOPhotonVision(VisionConstants.cameraFrontLeft),
                    new VisionIOPhotonVision(VisionConstants.cameraFrontRight),
                    new VisionIOPhotonVision(VisionConstants.cameraRightmost)
                );
                hopperVision = new HopperVision(new HopperVisionIOPhoton());
                intake = new Intake(new IntakeIOReal() {});
                turret = new Turret(new TurretIOTalonHighFrequency() {});
                // climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIOReal());
                break;
            case SIM:
                leds = new LEDs(new LEDIORio());
                turret = new Turret(new TurretIOSim());
                spindexer = new Spindexer(new SpindexerIOSim() {});
                intake = new Intake(new IntakeIOSim() {});

                var driveSimulation = Simulation.getInstance().configureSimulation(intake, spindexer, turret);

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
                    new VisionIOPhotonVisionSim(VisionConstants.cameraLeftmost, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.cameraFrontLeft, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.cameraFrontRight, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(VisionConstants.cameraRightmost, driveSimulation::getSimulatedDriveTrainPose)
                );
                hopperVision = new HopperVision(new HopperVisionIOSim());
                // climber = new Climber(new ClimberIO() {});
                
                drive.setPose(new Pose2d(3, 3, new Rotation2d()));
                break;
            default:
                leds = new LEDs(new LEDIO() {});
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {});
                vision = new Vision(new VisionIO() {}, new VisionIO() {});
                hopperVision = new HopperVision(new HopperVisionIO() {});
                intake = new Intake(new IntakeIO() {});
                turret = new Turret(new TurretIO() {});
                // climber = new Climber(new ClimberIO() {});
                spindexer = new Spindexer(new SpindexerIO() {});
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedAutoChooser("Auto Choices");
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        
        routines = new AutoRoutines(this, autoChooser);

        Controls.getInstance().configureControls(this);
        testChooser = TuningCommands.constructTuningChooser(this);

        if(Constants.isSim) Simulation.getInstance().resetSimulationField();
    }

    public Command getTestCommand() {
        return testChooser.get();
    }

    public boolean noAutoSelected() {
        if(autoChooser.getSelectedName() != "Nothing") {
            return false;
        } else {
            return true;
        }
    }
}
