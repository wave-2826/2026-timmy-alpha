package frc.robot.subsystems.climber;

import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSparkPID;

public class ClimberConstants {
    public static final int leftCanId = 0;
    public static final int rightCanId = 0;
    public static final double motorReduction = 104.4;
    public static final int currentLimit = 60;

    public static final TunableSparkPID climbPID =  new TunableSparkPID("/Climb/Climb")
    .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
    .addSimGains(new SparkPIDConstants(0.005, 0, 0));
}
