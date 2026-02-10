package frc.robot.subsystems.turret;

import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSparkPID;

public class TurretConstants {
    // CAN IDs
    public static final int topFlywheelCanID = 71;
    public static final int bottomFlywheelCanID = 72;
    public static final int azimuthCanID = 73;
    public static final int hoodCanID = 74;

    // Reductions:
    // All stages have the same 31:200 reduction, but the hood and azimuth are further reduced by the bevel and planetary stages.
    public static final double flywheelToRingReduction = 31.0 / 200.0;
    public static final double azimuthToRingReduction = 31.0 / 200.0;
    public static final double hoodToRingReduction = 31.0 / 200.0;
    
    public static final double flywheelPlanetReduction = 213.0 / 25.0;
    public static final double hoodPlanetReduction = 213.0 / 25.0;

    public static final double flywheelBevelReduction = 10.0 / 18.0;
    public static final double hoodBevelReduction = 20.0 / 35.0;

    // Current limits
    public static final int flywheelCurrentLimit = 40; // amps
    public static final int azimuthCurrentLimit = 15; // amps
    public static final int hoodCurrentLimit = 15; // amps

    // PIDs
    public static final TunableSparkPID flywheelMotorPID = new TunableSparkPID("Turret/Flywheel")
        .addRealRobotGains(new SparkPIDConstants(0.0002, 0.0, 0.0, 0.000015))
        .addSimGains(new SparkPIDConstants(0.0001, 0.0, 0.0, 0.00001));
    public static final TunableSparkPID azimuthMotorPID = new TunableSparkPID("Turret/Azimuth")
        .addRealRobotGains(new SparkPIDConstants(0.1, 0.0, 0.0, 0.0))
        .addSimGains(new SparkPIDConstants(0.05, 0.0, 0.0, 0.0));
    public static final TunableSparkPID hoodMotorPID = azimuthMotorPID;
}
