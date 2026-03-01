package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import java.util.Random;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Simulation for a IMU module used as gyro.
 *
 * <p>The Simulation is basically an indefinite integral of the angular velocity during each simulation sub ticks. Above
 * that, it also simulates the measurement inaccuracy of the gyro, drifting in no-motion and drifting due to impacts.
 */
public class GyroSimulation {
    /* The threshold of instantaneous angular acceleration at which the chassis is considered to experience an "impact." */
    private static final double angularAccelerationDriftThreshold = 500;
    /* The amount of drift, in radians, that the gyro experiences as a result of each multiple of the angular acceleration threshold. */
    private static final double impactDriftCoefficient = Math.toRadians(1);
    
    private final double average30sDriftMotionless;
    private final double velocityMeasurementStdDevPercent;

    private Rotation2d gyroReading;
    private double measuredAngularVelocityRadPerSec, previousAngularVelocityRadPerSec;
    private final Queue<Rotation2d> cachedRotations;

    /**
     * <h2>Creates a Gyro Simulation.</h2>
     *
     * @param average30sDriftMotionless the average amount of drift, in degrees, the gyro experiences
     *     if it remains motionless for 30 seconds on a vibrating platform. This value can often be found in the user
     *     manual.
     * @param velocityMeasurementStdDevPercent the standard deviation of the velocity measurement,
     *     typically around 0.05
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

    public static double generateRandomNormal(double mean, double stdDev) {
        double u1 = random.nextDouble();
        double u2 = random.nextDouble();
        // Box–Muller transform https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
        double z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        return z0 * stdDev + mean;
    }

    /**
     * <h2>Gets the Measured ΔTheta of the Gyro.</h2>
     *
     * <p>This method simulates the change in the robot's angle (ΔTheta) since the last sub-tick, as measured by the
     * gyro.
     *
     * <p>The measurement includes random errors based on the configuration of the gyro.
     *
     * @param actualAngularVelocityRadPerSec the actual angular velocity in radians per second, used to calculate the
     *     ΔTheta
     * @return the measured ΔTheta, including any measurement errors
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