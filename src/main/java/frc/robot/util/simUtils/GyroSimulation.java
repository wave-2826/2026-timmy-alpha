package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Simulates an IMU gyro.
 *
 * Integrates angular velocity per simulation sub-tick. Simulates measurement inaccuracy, drift at rest, and impact drift.
 */
public class GyroSimulation {
    // Threshold for angular acceleration to count as an "impact"
    private static final double angularAccelerationDriftThreshold = 500;
    // Drift per threshold multiple, in radians
    private static final double impactDriftCoefficient = Math.toRadians(1);
    
    private final double average30sDriftMotionless;
    private final double velocityMeasurementStdDevPercent;

    private Rotation2d gyroReading;
    private double measuredAngularVelocityRadPerSec, previousAngularVelocityRadPerSec;
    private final Queue<Rotation2d> cachedRotations;

    /**
     * Creates a Gyro Simulation.
     *
     * @param average30sDriftMotionless average drift in degrees over 30s at rest
     * @param velocityMeasurementStdDevPercent stddev of velocity measurement, e.g. 0.05
     */
    public GyroSimulation(double average30sDriftMotionless, double velocityMeasurementStdDevPercent) {
        this.average30sDriftMotionless = average30sDriftMotionless;
        this.velocityMeasurementStdDevPercent = velocityMeasurementStdDevPercent;

        gyroReading = new Rotation2d();
        this.previousAngularVelocityRadPerSec = this.measuredAngularVelocityRadPerSec = 0;
        this.cachedRotations = new ConcurrentLinkedQueue<>();
        for(int i = 0; i < Simulation.subTicks; i++) cachedRotations.offer(gyroReading);
    }

    public void setRotation(Rotation2d currentRotation) {
        this.gyroReading = currentRotation;
    }

    public Rotation2d getGyroReading() {
        return gyroReading;
    }

    public AngularVelocity getMeasuredAngularVelocity() {
        return RadiansPerSecond.of(measuredAngularVelocityRadPerSec);
    }

    public AngularAcceleration getMeasuredAngularAcceleration() {
        double angularAccelerationRadPerSecSq = (measuredAngularVelocityRadPerSec - previousAngularVelocityRadPerSec) / Simulation.simulationDtSeconds;
        return RadiansPerSecondPerSecond.of(angularAccelerationRadPerSecSq);
    }

    public Rotation2d[] getCachedGyroReadings() {
        return cachedRotations.toArray(Rotation2d[]::new);
    }

    public void updateSimulationSubTick(double actualAngularVelocityRadPerSec) {
        final Rotation2d driftingDueToImpact = getDriftingDueToImpact(actualAngularVelocityRadPerSec);
        gyroReading = gyroReading.plus(driftingDueToImpact);

        final Rotation2d dTheta = getGyroDTheta(actualAngularVelocityRadPerSec);
        gyroReading = gyroReading.plus(dTheta);

        final Rotation2d noMotionDrifting = getNoMotionDrifting();
        gyroReading = gyroReading.plus(noMotionDrifting);

        cachedRotations.poll();
        cachedRotations.offer(gyroReading);
    }

    private Rotation2d getDriftingDueToImpact(double actualAngularVelocityRadPerSec) {
        final double angularAccelerationRadPerSecSq = (actualAngularVelocityRadPerSec - previousAngularVelocityRadPerSec) / Simulation.simulationDtSeconds;
        final double driftingDueToImpactDegAbsVal = Math.abs(angularAccelerationRadPerSecSq) > angularAccelerationDriftThreshold
            ? Math.abs(angularAccelerationRadPerSecSq)
                / angularAccelerationDriftThreshold
                * impactDriftCoefficient
            : 0;
        final double driftingDueToImpactDeg = Math.copySign(driftingDueToImpactDegAbsVal, -angularAccelerationRadPerSecSq);

        previousAngularVelocityRadPerSec = actualAngularVelocityRadPerSec;

        return Rotation2d.fromRadians(driftingDueToImpactDeg);
    }

    private static final Random random = new Random();

    // Returns a random value from a normal distribution
    public static double generateRandomNormal(double mean, double stdDev) {
        double u1 = random.nextDouble();
        double u2 = random.nextDouble();
        // Box–Muller transform
        double z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        return z0 * stdDev + mean;
    }

    /**
     * Gets the measured delta-theta of the gyro.
     *
     * Simulates change in angle since last sub-tick, with random error.
     *
     * @param actualAngularVelocityRadPerSec actual angular velocity in rad/s
     * @return measured delta-theta with error
     */
    private Rotation2d getGyroDTheta(double actualAngularVelocityRadPerSec) {
        this.measuredAngularVelocityRadPerSec = generateRandomNormal(
            actualAngularVelocityRadPerSec,
            velocityMeasurementStdDevPercent * Math.abs(actualAngularVelocityRadPerSec));
        return Rotation2d.fromRadians(measuredAngularVelocityRadPerSec * Simulation.simulationDtSeconds);
    }

    private Rotation2d getNoMotionDrifting() {
        final double averageDriftingPerPeriod = this.average30sDriftMotionless
            / 30 * Simulation.simulationDtSeconds,
            driftingInThisPeriod = generateRandomNormal(0, averageDriftingPerPeriod);

        return Rotation2d.fromDegrees(driftingInThisPeriod);
    }
}
