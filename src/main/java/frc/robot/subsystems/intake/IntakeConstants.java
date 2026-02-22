package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkPIDConstants;
import frc.robot.util.tunables.TunableSparkPID;

public class IntakeConstants {
    public static final int intakeRollerCanId = 30;
    public static final int intakeDeployRCanId = 31;
    public static final int intakeDeployLCanId = 32;

    public static final double rollerMotorReduction = 1.0;
    
    public static final int rollerCurrentLimit = 40;
    public static final int deployCurrentLimit = 25;
    
    public static final double opeThatsaResetCurrent = 10.0;
    
    public static final double pinionRadiusMeters = 0.0362855;
    public static final double trackLengthMeters = Units.inchesToMeters(14.75);

    public static final TunableSparkPID rollerPID = new TunableSparkPID("Intake/Roller")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .addSimGains(new SparkPIDConstants(0.005, 0, 0));
    public static final TunableSparkPID deployPID = new TunableSparkPID("Intake/Deploy")
        .addRealRobotGains(new SparkPIDConstants(0.005, 0, 0))
        .addSimGains(new SparkPIDConstants(0.005, 0, 0));
}
