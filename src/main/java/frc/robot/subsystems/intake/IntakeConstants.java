package frc.robot.subsystems.intake;

import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSparkPID;

public class IntakeConstants {
    public static final int intakeRollerCanId = 20;
    public static final int intakeDeployRCanId = 21;
    public static final int intakeDeployLCanId = 22;
    public static final int intakeDeployRLaserCanId = 23;
    public static final int intakeDeployLLaserCanId = 24;
    public static final double motorReduction = 1.0;
    public static final int currentLimit = 40;

    public static final TunableSparkPID rollerPID = new TunableSparkPID("Intake/Roller")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .addSimGains(new SparkPIDConstants(0.005, 0, 0));
}
