package frc.robot.subsystems.turret;

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
    
    public static final double hoodPlanetReduction = 213.0 / 25.0;
    public static final double flywheelPlanetReduction = 213.0 / 25.0;

    public static final double hoodBevelReduction = 20.0 / 35.0;
    public static final double azimuthBevelReduction = 10.0 / 18.0;
}
