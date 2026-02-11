package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.SparkPIDConstants;
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

    // Limits
    public static final double maxFlywheelSpeedRadPerSec = DCMotor.getNeoVortex(1).freeSpeedRadPerSec *
        flywheelToRingReduction * flywheelPlanetReduction * flywheelBevelReduction * 0.8;
    public static final double maxHoodRingSpeedRadPerSec = DCMotor.getNeoVortex(1).freeSpeedRadPerSec *
        hoodToRingReduction * hoodPlanetReduction * hoodBevelReduction * 0.8;
    public static final double maxAzimuthSpeedRadPerSec = DCMotor.getNeoVortex(1).freeSpeedRadPerSec *
        azimuthToRingReduction * 0.8;

    // Current limits
    public static final int flywheelCurrentLimit = 40; // amps
    public static final int azimuthCurrentLimit = 40; // amps
    public static final int hoodCurrentLimit = 40; // amps

    // PIDs
    public static final TunableSparkPID flywheelMotorPID = new TunableSparkPID("Turret/Flywheel")
        .addRealRobotGains(new SparkPIDConstants(0.0002, 0.0, 0.0, 0.000015))
        .addSimGains(new SparkPIDConstants(0.0001, 0.0, 0.0, 0.00001));
    public static final TunableSparkPID azimuthMotorPID = new TunableSparkPID("Turret/Azimuth")
        .addRealRobotGains(new SparkPIDConstants(0.4, 0.0, 0.0, 0.0))
        .addSimGains(new SparkPIDConstants(0.05, 0.0, 0.0, 0.0));
    public static final TunableSparkPID hoodMotorPID = azimuthMotorPID;
}
