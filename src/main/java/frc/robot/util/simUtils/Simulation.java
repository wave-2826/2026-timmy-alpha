package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
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

    public SwerveDriveSimulation driveSimulation = null;
    
    public final DriveTrainSimulationConfig drivetrainConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(DriveConstants.robotMass)
        .withCustomModuleTranslations(DriveConstants.moduleTranslations)
        .withBumperSize(Inches.of(31), Inches.of(37))
        .withTrackLengthTrackWidth(DriveConstants.wheelBaseX, DriveConstants.trackWidthY)
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulation.SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getFalcon500(1),
            DriveConstants.driveGearRatio,
            DriveConstants.steerGearRatio,
            DriveConstants.driveFrictionVoltage,
            DriveConstants.steerFrictionVoltage,
            DriveConstants.wheelRadius,
            DriveConstants.steerInertia,
            DriveConstants.wheelCOF));
    
    private FuelSim fuel;

    private Simulation() {
        fuel = new FuelSim();
        fuel.enableAirResistance();
    }

    public SwerveDriveSimulation configureSimulation() {
        driveSimulation = new SwerveDriveSimulation(drivetrainConfig, new Pose2d(3, 3, new Rotation2d()));
        RobotState.getInstance().resetSimulationPoseCallback = driveSimulation::setSimulationWorldPose;

        fuel.registerRobot(Units.inchesToMeters(7), driveSimulation);
        
        return driveSimulation;
    }

    public void resetSimulationField() {
        if(Constants.currentMode != Constants.Mode.SIM) return;

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        
        fuel.clearFuel();
        fuel.spawnStartingFuel();
    }

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
    }
}
