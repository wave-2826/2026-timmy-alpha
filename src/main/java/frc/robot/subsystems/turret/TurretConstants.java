package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.GenericPIDConstants;
import frc.robot.util.GenericPIDConstants.PIDSlot;
import frc.robot.util.tunables.TunableSimpleMotorFF;
import frc.robot.util.tunables.TunablePID;

public class TurretConstants {
    // CAN IDs
    public static final int flywheel1CanID = 51;
    public static final int flywheel2CanID = 52;
    public static final int azimuthCanID = 53;
    public static final int hoodCanID = 54;
    public static final int azimuthCancoderID = 55;

    public static final int azimuthZeroDIOPort = 1;
    public static final Rotation2d azimuthResetAngle = Rotation2d.fromDegrees(151.5); // clockwise relative to turret forward

    public static final CANBus CANBus = DriveConstants.CANBus;

    // Reductions; all are a ratio between output and input.
    // All stages have the same 31:200 reduction, but the hood and azimuth are further reduced by the bevel and planetary stages.
    public static final double flyMotorToRingReduction = (36. / 28.) * (35.0 / 200.0);
    public static final double aziMotorToRingReduction = 18.0 / 200.0;
    public static final double hoodMotorToRingReduction = 35.0 / 200.0;
    
    public static final double flywheelPlanetReduction = 213.0 / 25.0;
    public static final double hoodPlanetReduction = 213.0 / 25.0;
    
    public static final double flywheelRingToFlyReduction = 10.0 / 18.0 * TurretConstants.flywheelPlanetReduction;
    public static final double hoodRingToHoodReduction = 1.0 / 172 * TurretConstants.hoodPlanetReduction;

    // Calculated reductions
    // Total gearings; these are a ratio between output and input, so should be less than 1.
    public static final double totalFlywheelGearing = TurretConstants.flyMotorToRingReduction * TurretConstants.flywheelRingToFlyReduction;
    public static final double totalHoodGearing = TurretConstants.hoodMotorToRingReduction * TurretConstants.hoodRingToHoodReduction;
    public static final double totalAzimuthGearing = TurretConstants.aziMotorToRingReduction;

    // Couplings; these are a direct ratio between each motor and their coupled output
    public static final double azimuthFlyCoupling = TurretConstants.totalAzimuthGearing * TurretConstants.flywheelRingToFlyReduction;
    public static final double azimuthHoodCoupling = TurretConstants.totalAzimuthGearing * TurretConstants.hoodRingToHoodReduction;

    // Constraints
    public static final double hoodMinAngle = Units.degreesToRadians(26);
    public static final double hoodMaxAngle = Units.degreesToRadians(63);

    public static final double maxTrenchHoodAngle = Units.degreesToRadians(40);

    public static final double flywheelRadius = Units.inchesToMeters(2);

    // Inertias
    private static final double reflectInertia(double externalInertia, double ratioInternal) {
        return ratioInternal * ratioInternal * externalInertia;
    }
    private static final double parallelAxisInertia(double inertia, double mass, double radius) {
        return inertia + mass * radius * radius;
    }
    // This could be tuned instead of calculated, but... eh...
    public static final DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    public static final DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    public static final DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    /** The moment of inertia experienced by the two motors for the flywheel (reflected through the drivetrain) */
    public static final double flywheelMotorInertiaKgM2 = reflectInertia(
        reflectInertia(
            0.0004089093 + // Wheel
            0.000066745  + // Wheel shaft
            0.0000368726 + // Shaft stuff
            0.0000011706 + // Other shaft stuff
            0.0004667602 * 3, // Inertial plates per plate
            flywheelRingToFlyReduction
        ) + 0.0116297925, // Big ring
        flyMotorToRingReduction
    ) + 0.0000201921 + 0.0000011706 + // Motor shaft stuff
        0.000221388368; // Rev NEO vortex MOI (measured since Rev doesn't give it to us...)
    
    /** The moment of inertia experienced by the motor for azimuth rotation (reflected through the drivetrain) */
    public static final double azimuthMotorInertiaKgM2 = reflectInertia(
        0.0116297925 + // Big ring
        parallelAxisInertia(0.0265304183, 1.4442381, 0.0297434), // Full turret azimuth MOI around center of rotation
        aziMotorToRingReduction
    ) + 0.0000201921 + 0.0000011706 + // Motor shaft stuff
        0.000221388368; // Rev NEO vortex MOI (measured since Rev doesn't give it to us...)

    /** The moment of inertia experienced by the motor for hood rotation (reflected through the drivetrain) */
    public static final double hoodMotorInertiaKgM2 = reflectInertia(
        reflectInertia(
            2.92639653e-6, // Power transmission
            hoodPlanetReduction
        ) + 0.0116297925, // Big ring
        hoodMotorToRingReduction
    ) + 0.0000201921 + 0.0000011706 + // Motor shaft stuff
        0.000221388368; // Rev NEO vortex MOI (measured since Rev doesn't give it to us...)

    /** kA for the flywheel system in volts per (rad/s^2). Calculated using the motor inertia reflected through the entire drivetrain. */
    public static final double flywheelMotorKA = flywheelMotorInertiaKgM2 / (flywheelSimMotor.KtNMPerAmp * 12); // uhh maybe?
    
    /** The translation from the robot origin to the turret shot position. */
    public static final Translation3d robotToTurret = new Translation3d(
        Units.inchesToMeters(2.329), Units.inchesToMeters(-4.06), Units.inchesToMeters(16.3)
    );

    // Limits
    public static final double maxFlywheelSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000); // Tuned
    public static final double maxHoodRingSpeedRadPerSec = hoodSimMotor.freeSpeedRadPerSec * hoodMotorToRingReduction * hoodPlanetReduction * hoodRingToHoodReduction * 0.8;
    public static final double maxAzimuthSpeedRadPerSec = azimuthSimMotor.freeSpeedRadPerSec * aziMotorToRingReduction * 0.8;

    // Current limits
    public static final int flywheelCurrentLimit = 70; // amps each
    public static final int azimuthCurrentLimit = 45; // amps
    public static final int hoodCurrentLimit = 40; // amps

    // PIDs
    public static final TunableSimpleMotorFF flywheelMotorFF = new TunableSimpleMotorFF("Turret/FlywheelFF")
        .addGains(0.0, 12.0 / maxFlywheelSpeedRadPerSec, flywheelMotorKA);
    
    public static final TunablePID flywheelMotorPID = new TunablePID("Turret/Flywheel")
        .addRealRobotGains(new GenericPIDConstants(0.15, 0.0, 0.0, 0.124)) // velocity voltage
        .addRealRobotGains(new GenericPIDConstants(0.1, 0, 0, 0.2, PIDSlot.Slot1))
        .addSimGains(new GenericPIDConstants(0.3, 0.1, 0, 0.12));
    
    public static final TunablePID azimuthMotorPID = new TunablePID("Turret/Azimuth")
        .addRealRobotGains(new GenericPIDConstants(160, 200, 0).kV(3.7)) // position voltage
        .addRealRobotGains(new GenericPIDConstants(3.0, 3.0, 0, 3.0, PIDSlot.Slot1))
        .addSimGains(new GenericPIDConstants(100, 15, 2));
    
    public static final TunablePID hoodMotorPID = new TunablePID("Turret/Hood")
        .addRealRobotGains(new GenericPIDConstants(60, 1, 0)) // position voltage
        .addRealRobotGains(new GenericPIDConstants(1.0, 0, 0, 1.0, PIDSlot.Slot1))
        .addSimGains(new GenericPIDConstants(50, 0, 0));
    
    // Control tolerances
    public static final double flywheelToleranceRadPerSecEnter = Units.rotationsPerMinuteToRadiansPerSecond(180);
    public static final double flywheelToleranceRadPerSecExit = Units.rotationsPerMinuteToRadiansPerSecond(240);
    public static final double azimuthToleranceRad = Units.degreesToRadians(4);
    public static final double hoodToleranceRad = Units.degreesToRadians(2);
}
