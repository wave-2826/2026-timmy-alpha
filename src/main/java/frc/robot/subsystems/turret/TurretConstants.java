package frc.robot.subsystems.turret;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSimpleMotorFF;
import frc.robot.util.tunables.TunableSparkPID;

public class TurretConstants {
    // CAN IDs
    public static final int topFlywheelCanID = 51;
    public static final int bottomFlywheelCanID = 52;
    public static final int azimuthCanID = 53;
    public static final int hoodCanID = 54;

    // Reductions; all are a ratio between output and input.
    // All stages have the same 31:200 reduction, but the hood and azimuth are further reduced by the bevel and planetary stages.
    public static final double flywheelToRingReduction = 31.0 / 200.0;
    public static final double azimuthToRingReduction = 31.0 / 200.0;
    public static final double hoodToRingReduction = 31.0 / 200.0;
    
    public static final double flywheelPlanetReduction = 213.0 / 25.0;
    public static final double hoodPlanetReduction = 213.0 / 25.0;

    public static final double flywheelBevelReduction = -10.0 / 18.0;
    public static final double hoodBevelReduction = 20.0 / 35.0;

    // Calculated reductions
    // Total gearings; these are a ratio between output and input, so should be less than 1.
    public static final double totalFlywheelGearing = TurretConstants.flywheelToRingReduction * TurretConstants.flywheelPlanetReduction * TurretConstants.flywheelBevelReduction;
    public static final double totalHoodGearing = TurretConstants.hoodToRingReduction * TurretConstants.hoodPlanetReduction * TurretConstants.hoodBevelReduction;
    public static final double totalAzimuthGearing = TurretConstants.azimuthToRingReduction;

    public static final double azimuthFlyCoupling = TurretConstants.flywheelToRingReduction * TurretConstants.flywheelBevelReduction;
    public static final double azimuthHoodCoupling = TurretConstants.hoodToRingReduction * TurretConstants.hoodBevelReduction;

    // Constraints
    public static final double hoodMinAngle = Units.degreesToRadians(15);
    public static final double hoodMaxAngle = Units.degreesToRadians(43);

    // Inertias
    private static final double reflectInertia(double externalInertia, double ratioInternal) {
        return (1 / ratioInternal) * (1 / ratioInternal) * externalInertia;
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
            0.000066745 + // Wheel shaft
            0.0000368726 + // Shaft stuff
            0.0000011706 + // Other shaft stuff
            0.0004667602 * 3, // Inertial plates per plate
            flywheelPlanetReduction * flywheelBevelReduction
        ) + 0.0116297925, // Big ring
        flywheelToRingReduction
    ) + 0.0000201921 + 0.0000011706 + // Motor shaft stuff
        0.00221388368; // Rev NEO vortex MOI (measured since Rev doesn't give it to us...)
    
    /** The moment of inertia experienced by the motor for azimuth rotation (reflected through the drivetrain) */
    public static final double azimuthMotorInertiaKgM2 = reflectInertia(
        0.0116297925 + // Big ring
        parallelAxisInertia(0.0265304183, 1.4442381, 0.0297434), // Full turret azimuth MOI around center of rotation
        azimuthToRingReduction
    ) + 0.0000201921 + 0.0000011706 + // Motor shaft stuff
        0.00221388368; // Rev NEO vortex MOI (measured since Rev doesn't give it to us...)

    /** The moment of inertia experienced by the motor for hood rotation (reflected through the drivetrain) */
    public static final double hoodMotorInertiaKgM2 = reflectInertia(
        reflectInertia(
            reflectInertia(
                parallelAxisInertia(0.00199082756, 0.2853, 0.079629), // The actual hood doodad
                hoodBevelReduction
            ) + 2.92639653e-6, // Transmission before bevel gear
            hoodPlanetReduction
        ) + 0.0116297925, // Big ring
        hoodToRingReduction
    ) + 0.0000201921 + 0.0000011706 + // Motor shaft stuff
        0.00221388368; // Rev NEO vortex MOI (measured since Rev doesn't give it to us...)

    /** kA for the flywheel system in volts per (rad/s^2). Calculated using the motor inertia reflected through the entire drivetrain. */
    public static final double flywheelMotorKA = flywheelMotorInertiaKgM2 / (flywheelSimMotor.KtNMPerAmp * 12); // uhh maybe?
    
    // Limits
    public static final double maxFlywheelSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(5300); // Tuned
    public static final double maxHoodRingSpeedRadPerSec = hoodSimMotor.freeSpeedRadPerSec * hoodToRingReduction * hoodPlanetReduction * hoodBevelReduction * 0.8;
    public static final double maxAzimuthSpeedRadPerSec = azimuthSimMotor.freeSpeedRadPerSec * azimuthToRingReduction * 0.8;

    // Current limits
    public static final int flywheelCurrentLimit = 67; // amps
    public static final int azimuthCurrentLimit = 70; // amps
    public static final int hoodCurrentLimit = 70; // amps

    // PIDs
    public static final TunableSparkPID flywheelMotorPID = new TunableSparkPID("Turret/Flywheel")
        .addRealRobotGains(new SparkPIDConstants(0.0005, 0.0, 0.0))
        .addRealRobotGains(new SparkPIDConstants(0.005, 0.0, 0.0, ClosedLoopSlot.kSlot1))
        .copyRealGainsInSim();
    
    public static final TunableSimpleMotorFF flywheelMotorFF = new TunableSimpleMotorFF("Turret/FlywheelFF")
        .addGains(0.0, 12.0 / maxFlywheelSpeedRadPerSec, flywheelMotorKA);
    
    public static final TunableSparkPID azimuthMotorPID = new TunableSparkPID("Turret/Azimuth")
        .addRealRobotGains(new SparkPIDConstants(0.7, 0.0, 0.2))
        .addRealRobotGains(new SparkPIDConstants(0.005, 0.0, 0.0, ClosedLoopSlot.kSlot1))
        .copyRealGainsInSim();
    public static final TunableSparkPID hoodMotorPID = azimuthMotorPID;
}
