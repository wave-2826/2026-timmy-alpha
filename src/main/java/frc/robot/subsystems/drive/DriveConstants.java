package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * A cleaned up TunerConstants file; most constants correspond to those in TunerConstants.java.
 * To tune the drivetrain, the following need to be changed:
 * - 
 */
public class DriveConstants {
    public static class SwerveModuleConfig {
        public final int steerMotorId;
        public final int driveMotorId;
        public final int encoderId;
        public final Angle encoderOffset;
        public final Distance xPosition;
        public final Distance yPosition;
        public final boolean invertSide;
        public final boolean invertMotor;
        public final boolean invertEncoder;

        /**
         * @param driveId
         * @param steerId
         * @param encoderId
         * @param encoderOffset
         * @param yPosition The forward-backward coordinate. Positive X is the front of the robot.
         * @param xPosition The left-right coordinate. Poitive Y is toward the left of the robot.
         * @param invertSide
         */
        public SwerveModuleConfig(
            int driveId, int steerId, int encoderId,
            Angle encoderOffset, Distance yPosition, Distance xPosition,
            boolean invertSide) {
            this.steerMotorId = steerId;
            this.driveMotorId = driveId;
            this.encoderId = encoderId;
            this.encoderOffset = encoderOffset;
            this.xPosition = xPosition;
            this.yPosition = yPosition;
            this.invertSide = invertSide;
            this.invertMotor = false;
            this.invertEncoder = false;
        }
 
        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(driveGearRatio)
                .withSteerMotorGearRatio(steerGearRatio)
                .withCouplingGearRatio(coupleRatio)
                .withWheelRadius(wheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSlipCurrent(slipCurrent)
                .withSpeedAt12Volts(linearFreeSpeed)
                .withDriveMotorType(driveMotorType)
                .withSteerMotorType(steerMotorType)
                .withFeedbackSource(steerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(steerInertia)
                .withDriveInertia(driveInertia)
                .withSteerFrictionVoltage(steerFrictionVoltage)
                .withDriveFrictionVoltage(driveFrictionVoltage);

        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants() {
            return ConstantCreator.createModuleConstants(
                steerMotorId, driveMotorId, encoderId,
                encoderOffset,
                xPosition, yPosition,
                invertSide, invertMotor, invertEncoder
            );
        }

        Translation2d getTranslation() {
            return new Translation2d(xPosition, yPosition);
        }
    }

    // Both sets of gains need to be tuned to your individual robot.

    public static final Mass robotMass = Kilogram.of(74.088);
    public static final MomentOfInertia robotMomentOfInertia = KilogramSquareMeters.of(6.883);
    public static final double wheelCOF = 1.2;

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(325).withKI(0).withKD(0.8).withKS(0.1).withKV(1.59).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3.0).withKI(0).withKD(0).withKS(0).withKV(0.124);

    /** The closed-loop output type to use for the steer motors; this affects their PID/FF gains */
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    /** The closed-loop output type to use for the steer motors; this affects their PID/FF gains */
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    /** The type of motor used for the drive motor */
    private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    /** The type of motor used for the steer motor */
    private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final Current slipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null. Some configs will be overwritten.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(60))
            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus CANBus = new CANBus("Default Name", "./logs/example.hoot");

    // Effective free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity linearFreeSpeed = MetersPerSecond.of(4.73); // "Magic" number from max speed measurement

    public static final Distance trackWidth = Inches.of(17.75);
    public static final Distance wheelBase = Inches.of(16);
    public static final double driveBaseRadius = Math.hypot(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0);

    public static final double maxSpeedMetersPerSec = linearFreeSpeed.in(MetersPerSecond);
    public static final double maxAngularSpeedRadPerSec = maxSpeedMetersPerSec / driveBaseRadius;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double coupleRatio = 3.5714285714285716;

    public static final double driveGearRatio = 6.746031746031747;
    public static final double steerGearRatio = 12.8;
    public static final Distance wheelRadius = Inches.of(2);

    private static final int pigeonId = 9;

    // These are only used for simulation
    public static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.004);
    public static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.025);
    // Simulated voltage necessary to overcome friction
    public static final Voltage steerFrictionVoltage = Volts.of(0.2);
    public static final Voltage driveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(CANBus.getName())
            .withPigeon2Id(pigeonId)
            .withPigeon2Configs(pigeonConfigs);
        
    public static final SwerveModuleConfig frontLeftConfig =
        new SwerveModuleConfig(10, 11, 12, Rotations.of(-0.042236328125), trackWidth.div(2.0), wheelBase.div(2.0), true);
    public static final SwerveModuleConfig frontRightConfig =
        new SwerveModuleConfig(20, 21, 22, Rotations.of(0.157958984375), trackWidth.div(-2.0), wheelBase.div(2.0), false);
    public static final SwerveModuleConfig backLeftConfig =
        new SwerveModuleConfig(40, 41, 42, Rotations.of(-0.431396484375), trackWidth.div(2.0), wheelBase.div(-2.0), true);
    public static final SwerveModuleConfig backRightConfig =
        new SwerveModuleConfig(30, 31, 32, Rotations.of(-0.030517578125), trackWidth.div(-2.0), wheelBase.div(-2.0), false);

    public static final ArrayList<SwerveModuleConfig> moduleConfigs = new ArrayList<>(Arrays.asList(
        frontLeftConfig,
        frontRightConfig,
        backLeftConfig,
        backRightConfig
    ));
    public static final Translation2d[] moduleTranslations = moduleConfigs
        .stream()
        .map(SwerveModuleConfig::getTranslation)
        .toArray(Translation2d[]::new);
    
    
    public static final RobotConfig pathplannerConfig = new RobotConfig(robotMass, robotMomentOfInertia,
        new ModuleConfig(wheelRadius, linearFreeSpeed, wheelCOF,
            DCMotor.getKrakenX60Foc(1).withReduction(driveGearRatio), DriveConstants.slipCurrent, 1),
        moduleTranslations);

    public static final PPHolonomicDriveController simHolonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(13.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
    public static final PPHolonomicDriveController realHolonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(6.5, 0.0, 0.25), new PIDConstants(8.0, 1.0, 0.75));

    static final double odometryFrequency = new CANBus(DriveConstants.drivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(DriveConstants.robotMass)
        .withCustomModuleTranslations(DriveConstants.moduleTranslations)
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getFalcon500(1),
            DriveConstants.driveGearRatio,
            DriveConstants.steerGearRatio,
            DriveConstants.driveFrictionVoltage,
            DriveConstants.steerFrictionVoltage,
            DriveConstants.wheelRadius,
            DriveConstants.steerInertia,
            DriveConstants.wheelCOF));

    /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If
         * they need the devices, they can access them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If
         * they need the devices, they can access them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is
         *     250 Hz on CAN FD, and 100 Hz on CAN 2.0.
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, odometryUpdateFrequency, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * <p>This constructs the underlying hardware devices, so users should not construct the devices themselves. If
         * they need the devices, they can access them through getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is
         *     250 Hz on CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation in the form [x, y, theta]ᵀ,
         *     with units in meters and radians
         * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y, theta]ᵀ, with
         *     units in meters and radians
         * @param modules Constants for each specific module
         */
        public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                    TalonFX::new,
                    TalonFX::new,
                    CANcoder::new,
                    drivetrainConstants,
                    odometryUpdateFrequency,
                    odometryStandardDeviation,
                    visionStandardDeviation,
                    modules);
        }
    }
}
