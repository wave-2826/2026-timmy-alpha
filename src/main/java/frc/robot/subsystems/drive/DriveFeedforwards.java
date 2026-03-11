package frc.robot.subsystems.drive;

/**
 * Small local adapter to replace the PathPlanner DriveFeedforwards type.
 * Keeps the same minimal API used in this codebase (accelerationsMPSSq()).
 */
public class DriveFeedforwards {
    private final double[] accelerations;

    public DriveFeedforwards(double[] accelerations) {
        this.accelerations = accelerations;
    }

    public double[] accelerationsMPSSq() {
        return accelerations;
    }
}
