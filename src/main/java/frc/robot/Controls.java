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
import frc.robot.commands.AutoShoot;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Elastic;
import frc.robot.util.simUtils.Simulation;
import frc.robot.util.ShiftHelpers;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Controls {
    private final Alert driverDisconnectedAlert = new Alert("Driver controller disconnected (port 0)", AlertType.kWarning);
    private final Alert coDriverDisconnectedAlert = new Alert("Co-driver controller disconnected (port 1)", AlertType.kInfo);

    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController coDriver = new CommandXboxController(1);

    private final LoggedTunableNumber endgameAlert1Time = new LoggedTunableNumber("Controls/EndgameAlert1Time", 30.0);
    private final LoggedTunableNumber endgameAlert2Time = new LoggedTunableNumber("Controls/EndgameAlert2Time", 20.0);

    static LoggedNetworkBoolean shiftYours = new LoggedNetworkBoolean("YourShift"); 

    private static final Controls instance = new Controls();

    public static Controls getInstance() {
        return instance;
    }

    private Controls() {
        // This is a singleton
    }

    /** Configures the controls. */
    public void configureControls(RobotContainer rc) {
        Drive drive = rc.drive;
        Turret turret = rc.turret;
        Climber climber = rc.climber;
        Spindexer spindexer = rc.spindexer;
        Intake intake = rc.intake;
        
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
        driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // driver.b().whileTrue(climber.extendBoth()).onTrue(intake.bringIntakeIn(1));
        driver.a().whileTrue(climber.retractBoth());

        // driver.leftBumper().onTrue(intake.deployIntake());//.onTrue(intake.runRollerPercent(20));
        // driver.rightBumper().onTrue(intake.runRollerPercent(0));
        // intake.setDefaultCommand(intake.setIntakePositionNormalized(driver::getLeftTriggerAxis));

        RobotModeTriggers.teleop().onTrue(climber.extendServos());

        // turret.setDefaultCommand(turret.runManual(coDriver::getRightTriggerAxis, coDriver::getLeftX, coDriver::getRightY));
        turret.setDefaultCommand(turret.runOscillationTest());

        turret.setDefaultCommand(AutoShoot.autoShoot(turret, spindexer, driver::getRightTriggerAxis, coDriver::getRightTriggerAxis));

        // Reset gyro or odometry if in simulation
        final Runnable resetGyro = Constants.isSim ? () -> drive.setPose(Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(RobotState.getInstance().getEstimatedPose().getTranslation(),
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero
                    : Rotation2d.k180deg)); // Zero gyro
        final Runnable resetOdometry = Constants.isSim
            ? () -> drive.setPose(Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose()) // Reset odometry to actual robot pose during simulation
            : () -> drive.setPose(
                new Pose2d(0, 0, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Rotation2d.kZero : Rotation2d.k180deg)); // Zero gyro

        driver.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
        driver.start().and(driver.leftStick()).debounce(0.5).onTrue(Commands.runOnce(resetOdometry, drive).ignoringDisable(true));
        
        // Endgame Alerts
        Trigger endgameAlert1Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert1Time.get());
        Trigger endgameAlert2Trigger = new Trigger(() -> DriverStation.isTeleopEnabled()
            && DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= endgameAlert2Time.get());

        endgameAlert1Trigger.onTrue(controllerRumbleWhileRunning(RumbleType.kBothRumble).withTimeout(0.5));
        endgameAlert2Trigger.onTrue(controllerRumbleWhileRunning(RumbleType.kBothRumble).withTimeout(0.4).andThen(Commands.waitSeconds(0.3)).repeatedly().withTimeout(2.0));

        RobotModeTriggers.autonomous().and(DriverStation::isFMSAttached).onTrue(Commands.runOnce(() -> {
            Elastic.selectTab("Autonomous");
        }));
        RobotModeTriggers.teleop().and(DriverStation::isFMSAttached).onTrue(Commands.runOnce(() -> {
            Elastic.selectTab("Teleoperated");
        }));

        shiftYours.set(ShiftHelpers.currentShiftIsYours());
    }

    private HashMap<Integer, Double> driverRumbleCommands = new HashMap<>();

    public void setDriverRumble(RumbleType type, double value, int hash) {
        if(value == 0.0) {
            driverRumbleCommands.remove(hash);
        } else {
            driverRumbleCommands.put(hash, value);
        }
        driver.setRumble(type, driverRumbleCommands.values().stream().reduce(0.0, Double::max));
    }

    public Command controllerRumbleWhileRunning(RumbleType type) {
        return Commands.startEnd(() -> {
            setDriverRumble(type, 1.0, hashCode());
        }, () -> {
            setDriverRumble(type, 0.0, hashCode());
        }).withName("ControllerRumbleWhileRunning");
    }

    /** Updates the controls, including shown alerts. */
    public void update() {
        // Controller disconnected alerts
        int driverPort = driver.getHID().getPort();
        int coDriverPort = coDriver.getHID().getPort();
        driverDisconnectedAlert.set(!DriverStation.isJoystickConnected(driverPort) || !DriverStation.getJoystickIsXbox(driverPort));
        coDriverDisconnectedAlert.set(!DriverStation.isJoystickConnected(coDriverPort) || !DriverStation.getJoystickIsXbox(coDriverPort));
    }

}
