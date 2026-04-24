package frc.robot.subsystems.spindexer;

import frc.robot.util.GenericPIDConstants;
import frc.robot.util.tunables.TunablePID;

public class SpindexerConstants {
    public static final int transferCanId = 40;
    public static final int spinnerCanId = 41;

    public static final double spinnerMotorReduction = 3.0;
    public static final double transferMotorReduction = 5.0;

    public static final int spinnerCurrentLimit = 40;
    public static final int transferCurrentLimit = 40;

    public static final int ballsInSpin = 5;

    public static final TunablePID spinnerPID = new TunablePID("Spindexer/Spinner")
        .addRealRobotGains(new GenericPIDConstants(0.005, 0, 0))
        .copyRealGainsInSim();
    public static final TunablePID transferPID = new TunablePID("Spindexer/Transfer")
        .addRealRobotGains(new GenericPIDConstants(0.005, 0, 0))
        .copyRealGainsInSim();
}
