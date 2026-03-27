package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.simUtils.SwerveDriveSimulation.COTS;
import frc.robot.util.simUtils.SwerveDriveSimulation.DriveTrainSimulationConfig;

public final class Simulation {
    public static final int subTicks = 5;
    public static final Time simulationDt = Seconds.of(TimedRobot.kDefaultPeriod / subTicks);
    public static final double simulationDtSeconds = simulationDt.in(Seconds);

    private static Simulation instance = null;
    public static Simulation getInstance() {
        if(instance == null) instance = new Simulation();
        return instance;
    }

    Drive drive = null;
    Spindexer spindexer = null;
    Turret turret = null;

    HopperSim hopper = new HopperSim();

    public SwerveDriveSimulation driveSimulation = null;
    
    public final DriveTrainSimulationConfig drivetrainConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(DriveConstants.robotMass)
        .withCustomModuleTranslations(DriveConstants.moduleTranslations)
        .withBumperSize(Inches.of(31), Inches.of(37))
        .withTrackLengthTrackWidth(DriveConstants.wheelBaseX, DriveConstants.trackWidthY)
        .withGyro(COTS.pigeon2());
    
    private FuelSim fuel;

    private Simulation() {
        fuel = new FuelSim();
        fuel.enableAirResistance();
        fuel.setLoggingFrequency(50);
    }

    public SwerveDriveSimulation configureSimulation(Intake intake, Spindexer spindexer, Turret turret) {
        driveSimulation = new SwerveDriveSimulation(drivetrainConfig, new Pose2d(3, 3, new Rotation2d()));
        RobotState.getInstance().resetSimulationPoseCallback = driveSimulation::setSimulationWorldPose;

        fuel.registerRobot(Units.inchesToMeters(7), driveSimulation);
        double xMax = -drivetrainConfig.bumperLengthX.in(Meters) * 0.5;
        // hacky but whatever
        double halfWidth = drivetrainConfig.bumperWidthY.in(Meters) * 0.5 - Units.inchesToMeters(4);
        fuel.registerIntake(
            xMax - IntakeConstants.fullyExtendedIntakeDepth, xMax, -halfWidth, halfWidth,
            () -> intake.isDeployed() && hopper.canIntake(),
            this::intakeFuel
        );

        this.spindexer = spindexer;
        this.turret = turret;
        
        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        
        return driveSimulation;
    }

    public int getHopperFuel() {
        return hopper.getFuelCount();
    }
    
    private void intakeFuel() {
        hopper.addFuel();
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        fuel.clearFuel();
        fuel.spawnStartingFuel();
        hopper.resetToPreload();
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    private Timer shotTimer = new Timer();

    public void updateSimulation() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        // SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        
        for(int i = 0; i < subTicks; i++) {
            fuel.stepSimSubtick();
            driveSimulation.update(simulationDtSeconds);
            SimulatedBattery.simulationSubTick();

            // TODO: also run other sim IO with subticks
        }

        // Shot updates
        hopper.update();
        var bps = spindexer.getBallsPerSecond();
        if(bps > 0.05) {
            shotTimer.start();
            if(shotTimer.advanceIfElapsed(1.0 / spindexer.getBallsPerSecond())) {
                boolean removed = hopper.removeFuel();
                if(removed) {
                    fuel.launchFuel(
                        turret.getShotVelocity(),
                        turret.getShotAngle(),
                        turret.getRobotRelativeYaw(),
                        TurretConstants.robotToTurret.getMeasureZ()
                    );
                }
            }
        } else {
            shotTimer.stop();
        }
    }

    public void simulationInit() {
        CommandScheduler.getInstance().schedule(
            Commands.waitSeconds(0.25).andThen(Commands.runOnce(() -> {
                DriverStationSim.setDsAttached(true);
                DriverStationSim.setAllianceStationId(AllianceStationID.Blue3);
            })).ignoringDisable(true)
        );
    }
}
