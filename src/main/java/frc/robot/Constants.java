package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    /**
     * Defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change the
     * value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
     */
    public static final Mode simMode = Mode.REPLAY;
    /** If the robot should log data in simulation. */
    public static final boolean logInSimulation = false;

    /**
     * Whether to use NetworkTables instead of RLog for AdvantageScope logging. RLog _significantly_ reduces lag in
     * AdvantageScope.
     */
    public static final boolean useNTLogs = false;

    /** If the robot is in "tuning mode". When in tuning mode, tunable constants are added to NetworkTables. */
    public static boolean tuningMode = true;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    };

    public static boolean isSim = currentMode == Mode.SIM;

    /**
     * Maintains a real-time thread priority for the main robot thread throughout the entire program execution. This is
     * INCREDIBLY dangerous! Do NOT use this without understanding the consequences and EXTENSIVELY testing code with it
     * enabled. If loop times are longer than 20ms, this WILL cause all other threads (including important vendor ones,
     * AdvantageKit ones, and more) to be starved and not execute. This can cause issues with odometry, instability with
     * sending commands, and other issues. However, this has quite visible advantages with reducing loop time
     * inconsistency. Again, if you want to use this functionality, test with it on and understand its consequences! If
     * there are spooky issues going on with the robot, disabling this (if enabled) is a good first step. Only use this
     * as a last resort. Here be dragons.
     */
    public static boolean useSuperDangerousRTThreadPriority = true;

    public static final double voltageCompensation = 12.0;
}
