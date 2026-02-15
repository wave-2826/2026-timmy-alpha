package frc.robot.subsystems.intake;

import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSparkPID;

public class IntakeConstants {
    public static final int intakeRollerCanId = 20;
    public static final int intakeDeployRCanId = 21;
    public static final int intakeDeployLCanId = 22;
    public static final int intakeDeployRLaserCanId = 23;
    public static final int intakeDeployLLaserCanId = 24;
    public static final double rollerMotorReduction = 9.0;
    public static final int rollerCurrentLimit = 40;
    public static final int deployCurrentLimit = 25;
    public static final int opeThatsaResetCurrent = 20;
    public static final double pinionRadius = 0.0362855;



    public static final TunableSparkPID rollerPID = new TunableSparkPID("Intake/Roller")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .addSimGains(new SparkPIDConstants(0.005, 0, 0));
    public static final TunableSparkPID deployPID = new TunableSparkPID("Intake/Deploy")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .addSimGains(new SparkPIDConstants(0.005, 0, 0));
}
