package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.drive.kinematicConstraints.KinematicConstraints;
import frc.robot.util.GenericPIDConstants;
import frc.robot.util.tunables.TunablePID;

/**
 * A cleaned up TunerConstants file; most constants correspond to those in TunerConstants.java.
 * To tune the drivetrain, the following need to be changed:
 * - 
 */
public class DriveConstants {
    public static class SwerveModuleConfig {
        public final int index;
        public final int steerMotorId;
        public final int driveMotorId;
        public final int encoderId;
        public final Distance xPosition;
        public final Distance yPosition;
        public final boolean invertSide;
        public final boolean invertMotor;
        public final boolean invertEncoder;

        /**
         * @param index
         * @param driveId
         * @param steerId
         * @param encoderId
         * @param encoderOffset
         * @param yPosition The forward-backward coordinate. Positive X is the front of the robot.
         * @param xPosition The left-right coordinate. Poitive Y is toward the left of the robot.
         * @param invertSide
         */
        public SwerveModuleConfig(
            int index,
            int driveId, int steerId, int encoderId,
            Distance yPosition, Distance xPosition,
            boolean invertSide) {
            this.index = index;
            this.steerMotorId = steerId;
            this.driveMotorId = driveId;
            this.encoderId = encoderId;
            this.xPosition = xPosition;
            this.yPosition = yPosition;
            this.invertSide = invertSide;
            this.invertMotor = false;
            this.invertEncoder = false;
        }

        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createConstants() {
            SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = 
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorGearRatio(driveGearRatio)
                    .withSteerMotorGearRatio(steerGearRatio)
                    .withCouplingGearRatio(coupleRatio)
                    .withWheelRadius(Drive.tuningResults.wheelRadiusResults.radiusMeters)
                    .withSteerMotorGains(steerGains)
                    .withDriveMotorGains(driveGains
                        .withKS(Drive.tuningResults.feedforwardResults.kS)
                        .withKV(Drive.tuningResults.feedforwardResults.kV))
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
                    .withSlipCurrent(Drive.tuningResults.slipResults.slipCurrentAmps)
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
            return ConstantCreator.createModuleConstants(
                steerMotorId, driveMotorId, encoderId,
                Radians.of(Drive.tuningResults.moduleZeroingResults.moduleOffsetsRadians[index]),
                xPosition, yPosition,
                invertSide, invertMotor, invertEncoder
            );
        }

        Translation2d getTranslation() {
            return new Translation2d(xPosition, yPosition);
        }
    }

    // Constant tuned data

    // ""Tuned"" with.. scales
    public static final Mass[] wheelForceMasses = new Mass[] {
        Pound.of(39.2),
        Pound.of(38.13),
        Pound.of(19.63),
        Pound.of(37.55)
    };
    public static final Mass robotMass = Pound.of(
        Arrays.stream(wheelForceMasses).mapToDouble(m -> m.in(Pounds)).sum()
    );
    // ""Tuned"" through CAD
    public static final MomentOfInertia robotMomentOfInertia = KilogramSquareMeters.of(7.4702);
    // Effective free speed (m/s) at 12 V applied output; tuned with max speed measurement
    public static final LinearVelocity linearFreeSpeed = MetersPerSecond.of(4.572);

    // Tuned by hand
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(Constants.isSim ? 500 : 1600)
        .withKI(Constants.isSim ? 0   : 0)
        .withKD(Constants.isSim ? 5   : 20)
        .withKS(Constants.isSim ? 0.1 : 0.1)
        .withKV(Constants.isSim ? 0.0 : 0.0)
        .withKA(Constants.isSim ? 0   : 0)

        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    /**
     * Base drive gains. Intentionally doesn't include kS or kV - those are found in tuning.
     */
    public static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(Constants.isSim ? 100   : 3.0)
        .withKI(Constants.isSim ? 0     : 0)
        .withKD(Constants.isSim ? 0     : 0);

    /** The type of motor used for the drive motor */
    private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    /** The type of motor used for the steer motor */
    private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors
    private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null. Some configs will be overwritten.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            // Swerve azimuth does not require much torque output, so we can set a relatively low
            // stator current limit to help avoid brownouts without impacting performance.
            .withStatorCurrentLimit(Amps.of(30))
            .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    public static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on. All swerve devices must share the same CAN bus
    public static final CANBus CANBus = new CANBus("*");

    /** Spacing between wheel centers on the Y axis */
    public static final Distance trackWidthY = Inches.of(27.5);
    /** Spacing between wheel centers on the X axis */
    public static final Distance wheelBaseX = Inches.of(14.5);

    public static final double driveBaseRadius = Math.hypot(trackWidthY.in(Meters) / 2.0, wheelBaseX.in(Meters) / 2.0);

    public static final double maxSpeedMetersPerSec = linearFreeSpeed.in(MetersPerSecond);
    public static final double maxAngularSpeedRadPerSec = maxSpeedMetersPerSec / driveBaseRadius;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double coupleRatio = 3.5714285714285716;

    public static final double driveGearRatio = 6.746031746031747;
    public static final double steerGearRatio = 12.8;

    public static final int pigeonId = 9;

    // These are only used for simulation
    public static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.004);
    public static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.025);
    // Simulated voltage necessary to overcome friction
    public static final Voltage steerFrictionVoltage = Volts.of(0.2);
    public static final Voltage driveFrictionVoltage = Volts.of(0.2);
    
    public static final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    public static final DCMotor turnMotorModel = DCMotor.getKrakenX60Foc(1);
    
    public static final KinematicConstraints kinematicConstraints = new KinematicConstraints(
        MetersPerSecondPerSecond.of(18) /* measuered "magic value" - max linear acceleration */,
        RadiansPerSecondPerSecond.of(18 * (maxAngularSpeedRadPerSec / maxSpeedMetersPerSec)),
        MetersPerSecondPerSecond.of(17.5), /* Skid acceleration limit */
        MetersPerSecondPerSecond.of(8), /* Max tilt acceleration X */
        MetersPerSecondPerSecond.of(16) /* Max tilt acceleration Y */
    );
    
    // Encoder offsets measured with 
    public static final SwerveModuleConfig frontLeftConfig =
        new SwerveModuleConfig(0, 20, 21, 22, trackWidthY.div(2.0), wheelBaseX.div(2.0), false);
    public static final SwerveModuleConfig frontRightConfig =
        new SwerveModuleConfig(1, 30, 31, 32, trackWidthY.div(-2.0), wheelBaseX.div(2.0), false);
    public static final SwerveModuleConfig backLeftConfig =
        new SwerveModuleConfig(2, 10, 11, 12, trackWidthY.div(2.0), wheelBaseX.div(-2.0), false);
    public static final SwerveModuleConfig backRightConfig =
        new SwerveModuleConfig(3, 40, 41, 42, trackWidthY.div(-2.0), wheelBaseX.div(-2.0), false);

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
    

    static final double odometryFrequency = CANBus.isNetworkFD() ? 250.0 : 100.0;

    
    public static final Supplier<Translation2d[]> GET_MODULE_POSITIONS = () -> new Translation2d[] {
        new Translation2d(DriveConstants.frontLeftConfig.xPosition, DriveConstants.frontLeftConfig.yPosition),
        new Translation2d(DriveConstants.frontRightConfig.xPosition, DriveConstants.frontRightConfig.yPosition),
        new Translation2d(DriveConstants.backLeftConfig.xPosition, DriveConstants.backLeftConfig.yPosition),
        new Translation2d(DriveConstants.backRightConfig.xPosition, DriveConstants.backRightConfig.yPosition),
    };

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

    public static final TunablePID autoLinearPID = new TunablePID("Autos/Linear")
        .addRealRobotGains(new GenericPIDConstants(6.0, 0.0, 0.0))
        .copyRealGainsInSim();
    public static final TunablePID autoAngularPID = new TunablePID("Autos/Angular")
        .addRealRobotGains(new GenericPIDConstants(3.0, 0.0, 0.4))
        .copyRealGainsInSim();
}
