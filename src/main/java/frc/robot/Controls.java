package frc.robot;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Elastic;
import frc.robot.util.simUtils.Simulation;
import frc.robot.util.ShiftHelpers;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Controls {
    private final Alert driverDisconnectedAlert = new Alert("Driver controller disconnected (port 0)", AlertType.kWarning);
    private final Alert coDriverDisconnectedAlert = new Alert("Co-driver controller disconnected (port 1)", AlertType.kWarning);

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController coDriver = new CommandXboxController(1);

    private final LoggedTunableNumber endgameAlert1Time = new LoggedTunableNumber("Controls/EndgameAlert1Time", 30.0);
    private final LoggedTunableNumber endgameAlert2Time = new LoggedTunableNumber("Controls/EndgameAlert2Time", 20.0);

    static LoggedNetworkBoolean shiftYours = new LoggedNetworkBoolean("YourShift"); 

    private static final Controls instance = new Controls();

    
    public static enum CodriverMode { Normal, TurretControl };
    private final Trigger normalCodriver;
    private final Trigger turretControlCodriver;
    private CodriverMode codriverMode = CodriverMode.Normal;

    public static Controls getInstance() {
        return instance;
    }

    // singleton class
    private Controls() {
        normalCodriver = new Trigger(() -> codriverMode == CodriverMode.Normal);
        turretControlCodriver = new Trigger(() -> codriverMode == CodriverMode.TurretControl);
    }

    /** Configures the controls. */
    public void configureControls(RobotContainer rc) {
        Drive drive = rc.drive;
        Turret turret = rc.turret;
        // Climber climber = rc.climber;
        Spindexer spindexer = rc.spindexer;
        Intake intake = rc.intake;
        
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> driver.getRightX()));
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        driver.leftBumper().onTrue(intake.deployIntake().alongWith(intake.enable()));
        driver.rightBumper().onTrue(intake.disable());

        intake.setDefaultCommand(intake.setIntakePositionNormalized(driver::getLeftTriggerAxis));

        // turret.setDefaultCommand(ScoringCommands.autoShoot(
        //     turret, spindexer,
        //     driver::getRightTriggerAxis,
        //     coDriver::getRightY,
        //     coDriver.rightBumper()::getAsBoolean
        // ));
        turretControlCodriver.and(coDriver.povRight()).onTrue(turret.adjustManualVelocity(250));
        turretControlCodriver.and(coDriver.povLeft()).onTrue(turret.adjustManualVelocity(-250));
        turretControlCodriver.and(coDriver.povUp()).onTrue(turret.adjustManualAngle(5));
        turretControlCodriver.and(coDriver.povDown()).onTrue(turret.adjustManualAngle(-5));
        turretControlCodriver.whileTrue(turret.runManual(
            coDriver::getRightTriggerAxis,
            coDriver::getLeftX
        ));
        turretControlCodriver.whileTrue(spindexer.runManual(coDriver::getLeftTriggerAxis));
        turretControlCodriver.and(coDriver.start().or(coDriver.back())).onTrue(turret.reset());

        normalCodriver.whileTrue(intake.runRollerScaled(coDriver::getLeftY));
        normalCodriver.and(coDriver.povDown()).onTrue(intake.enableOutward());
        normalCodriver.and(coDriver.povUp()).onTrue(intake.enable());
        normalCodriver.and(coDriver.povLeft().or(coDriver.povRight())).onTrue(intake.disable());

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.isSim
            // Reset odometry to actual robot pose in sim
            ? () -> drive.setPose(Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(
                RobotState.getInstance().getEstimatedPose().getTranslation(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg
            ));
        
        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        
        turretControlCodriver.whileTrue(controllerRumbleWhileRunning(coDriver, RumbleType.kRightRumble).withName("TurretCodriverControls"));
        coDriver.rightBumper().onTrue(Commands.runOnce(() -> {
            if(codriverMode == CodriverMode.Normal) {
                codriverMode = CodriverMode.TurretControl;
            } else {
                codriverMode = CodriverMode.Normal;
            }
        }).ignoringDisable(true));

        // Endgame Alerts
        Trigger endgameAlert1Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert1Time.get());
        Trigger endgameAlert2Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert2Time.get());

        endgameAlert1Trigger.onTrue(controllerRumbleWhileRunning(driver, RumbleType.kBothRumble).withTimeout(0.5));
        endgameAlert2Trigger.onTrue(controllerRumbleWhileRunning(driver, RumbleType.kBothRumble).withTimeout(0.4).andThen(Commands.waitSeconds(0.3)).repeatedly().withTimeout(2.0));

        RobotModeTriggers.autonomous().and(DriverStation::isFMSAttached).onTrue(Commands.runOnce(() -> {
            Elastic.selectTab("Autonomous");
        }));
        RobotModeTriggers.teleop().and(DriverStation::isFMSAttached).onTrue(Commands.runOnce(() -> {
            Elastic.selectTab("Teleoperated");
        }));

        
        // driver.b().whileTrue(climber.extendBoth()).onTrue(intake.bringIntakeIn(1));
        // driver.a().whileTrue(climber.retractBoth());
        // coDriver.rightBumper().whileTrue(climber.manualControls(coDriver::getLeftY, coDriver::getRightY));
    }

    private HashMap<Integer, Double> driverRumbleCommands = new HashMap<>();

    public void setRumble(CommandXboxController controller, RumbleType type, double value, int hash) {
        if(value == 0.0) {
            driverRumbleCommands.remove(hash);
        } else {
            driverRumbleCommands.put(hash, value);
        }
        controller.setRumble(type, driverRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public Command controllerRumbleWhileRunning(CommandXboxController controller, RumbleType type) {
        return Commands.startEnd(() -> {
            setRumble(controller, type, 1.0, hashCode());
        }, () -> {
            setRumble(controller, type, 0.0, hashCode());
        }).withName("ControllerRumbleWhileRunning").ignoringDisable(true);
    }

    /** Updates the controls, including shown alerts. */
    public void update() {
        // Controller disconnected alerts
        int driverPort = driver.getHID().getPort();
        int coDriverPort = coDriver.getHID().getPort();
        driverDisconnectedAlert.set(!DriverStation.isJoystickConnected(driverPort) || !DriverStation.getJoystickIsXbox(driverPort));
        coDriverDisconnectedAlert.set(!DriverStation.isJoystickConnected(coDriverPort) || !DriverStation.getJoystickIsXbox(coDriverPort));
        shiftYours.set(ShiftHelpers.currentShiftIsYours());
    }

}
