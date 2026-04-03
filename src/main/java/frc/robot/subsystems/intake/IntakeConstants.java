package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.GenericPIDConstants;
import frc.robot.util.tunables.TunablePID;

public class IntakeConstants {
    public static final int intakeRollerLCanId = 30;
    public static final int intakeRollerRCanId = 33;
    
    public static final int intakeDeployRCanId = 32;
    public static final int intakeDeployLCanId = 31;

    public static final double rollerMotorReduction = 1.0;
    
    public static final int rollerCurrentLimit = 50;
    public static final int deployCurrentLimit = 60;
    
    /** The current we use to detect the motors at the end of their travel */
    public static final double deployStallCurrent = 55;
    
    public static final double pinionReduction = 9. * 2.; // ??????
    public static final double pinionRadiusMeters = Units.inchesToMeters(2.857143/2);
    public static final double trackLengthMeters = Units.inchesToMeters(14.75);

    // Geometry
    public static final double fullyExtendedIntakeDepth = Units.inchesToMeters(20); // TODO: collect from cad

    public static final TunablePID rollerPID = new TunablePID("Intake/Roller")
        .addRealRobotGains(new GenericPIDConstants(0.002, 0, 0, 1. / DCMotor.getNeoVortex(1).KvRadPerSecPerVolt))
        .addSimGains(new GenericPIDConstants(0.005, 0, 0));
    public static final TunablePID deployPID = new TunablePID("Intake/Deploy")
        .addRealRobotGains(new GenericPIDConstants(1.25, 0, 0))
        .addSimGains(new GenericPIDConstants(0.005, 0, 0));
}
