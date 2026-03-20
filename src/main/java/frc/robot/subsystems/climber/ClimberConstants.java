package frc.robot.subsystems.climber;

import frc.robot.util.GenericPIDConstants;
import frc.robot.util.tunables.TunablePID;

public class ClimberConstants {
    public static final int leftCanId = 20;
    public static final int rightCanId = 21;

    public static final double motorReduction = 104.4;

    public static final int currentLimit = 60;

    public static final int leftServoPWM = 3;
    public static final int rightServoPWM = 4;

    public static final TunablePID climbPID =  new TunablePID("/Climb/Climb")
        .addRealRobotGains(new GenericPIDConstants(0.005, 0, 0))
        .addSimGains(new GenericPIDConstants(0.005, 0, 0));
}
