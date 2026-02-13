package frc.robot.subsystems.turret;

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

    // Reductions:
    // All stages have the same 31:200 reduction, but the hood and azimuth are further reduced by the bevel and planetary stages.
    public static final double flywheelToRingReduction = 31.0 / 200.0;
    public static final double azimuthToRingReduction = 31.0 / 200.0;
    public static final double hoodToRingReduction = 31.0 / 200.0;
    
    public static final double flywheelPlanetReduction = 213.0 / 25.0;
    public static final double hoodPlanetReduction = 213.0 / 25.0;

    public static final double flywheelBevelReduction = 10.0 / 18.0;
    public static final double hoodBevelReduction = 20.0 / 35.0;

    // Inertias
    private static final double reflectInertia(double externalInertia, double ratioInternal) {
        return (1 / ratioInternal) * (1 / ratioInternal) * externalInertia;
    }
    // This could be tuned instead of calculated, but... eh...
    public static final DCMotor flywheelSimMotor = DCMotor.getNeoVortex(1);
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
    /** kA for the flywheel system in volts per (rad/s^2). Calculated using the motor inertia reflected through the entire drivetrain. */
    public static final double flywheelMotorKA = flywheelMotorInertiaKgM2 / (flywheelSimMotor.KtNMPerAmp * 12); // uhh maybe?
     
    // Limits
    public static final double maxFlywheelSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(4966); // Tuned
    public static final double maxHoodRingSpeedRadPerSec = DCMotor.getNeoVortex(1).freeSpeedRadPerSec *
        hoodToRingReduction * hoodPlanetReduction * hoodBevelReduction * 0.8;
    public static final double maxAzimuthSpeedRadPerSec = DCMotor.getNeoVortex(1).freeSpeedRadPerSec *
        azimuthToRingReduction * 0.8;

    // Current limits
    public static final int flywheelCurrentLimit = 67; // amps
    public static final int azimuthCurrentLimit = 70; // amps
    public static final int hoodCurrentLimit = 70; // amps

    // PIDs
    public static final TunableSparkPID flywheelMotorPID = new TunableSparkPID("Turret/Flywheel")
        .addRealRobotGains(new SparkPIDConstants(0.0002, 0.0, 0.0, 0.0))
        .addSimGains(new SparkPIDConstants(0.0001, 0.0, 0.0, 0.0));
    public static final TunableSimpleMotorFF flywheelMotorFF = new TunableSimpleMotorFF("Turret/FlywheelFF")
        .addGains(0.0, 12.0 / maxFlywheelSpeedRadPerSec, 0.9); // TODO: Tune gains
    public static final TunableSparkPID azimuthMotorPID = new TunableSparkPID("Turret/Azimuth")
        .addRealRobotGains(new SparkPIDConstants(0.7, 0.0, 0.2, 0.0))
        .addSimGains(new SparkPIDConstants(0.05, 0.0, 0.0, 0.0));
    public static final TunableSparkPID hoodMotorPID = azimuthMotorPID;
}
