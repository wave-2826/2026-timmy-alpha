package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
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

    public SwerveDriveSimulation configureSimulation(Intake intake) {
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
        
        return driveSimulation;
    }

    
    private void intakeFuel() {
        hopper.addFuel();
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        
        fuel.clearFuel();
        fuel.spawnStartingFuel();
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public void updateSimulation() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        // SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        
        for(int i = 0; i < subTicks; i++) {
            fuel.stepSimSubtick();
            driveSimulation.update(simulationDtSeconds);
            SimulatedBattery.simulationSubTick();

            if(drive != null) driveSimulation.updateOdom(drive);

            // TODO: also run other sim IO with subticks
            
               
        }
    }

    public void simulationInit() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
    }
}
