package frc.robot.subsystems.spindexer;

import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSparkPID;

public class SpindexerConstants {
    public static final int transferCanId = 40;
    public static final int spinnerCanId = 41;

    public static final double spinnerMotorReduction = 25.0;
    public static final double transferMotorReduction = 15.0;

    public static final int spinnerCurrentLimit = 20;
    public static final int transferCurrentLimit = 10;

    public static final TunableSparkPID spinnerPID = new TunableSparkPID("/Spindexer/Spinner")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .copyRealGainsInSim();
    public static final TunableSparkPID transferPID = new TunableSparkPID("/Spindexer/Transfer")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .copyRealGainsInSim();
}
