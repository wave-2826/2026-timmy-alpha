package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.simUtils.Simulation;

import java.util.function.Supplier;

public final class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static StatusCode tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        StatusCode lastError = StatusCode.OK;
        for(int i = 0; i < maxAttempts; i++) {
            lastError = command.get();
            if(lastError.isOK()) return lastError;
        }

        Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Phoenix tryUntilOk failed", "Likely failed to configure a controller!"));
        return lastError;
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps = new double[Simulation.subTicks];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp() - 0.02 + i * Simulation.simulationDtSeconds;
        }

        return odometryTimeStamps;
    }

    /**
     * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
     *
     * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation purposes. These
     * changes have no effect on real robot operations and address known simulation bugs:
     *
     * <ul>
     *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted configurations.
     *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization issues.
     *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve simulation
     *       performance.
     * </ul>
     *
     * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants used on real
     * robot hardware.</h4>
     */
    public static
        <A extends ParentConfiguration, B extends ParentConfiguration, C extends ParentConfiguration>
        SwerveModuleConstants<A, B, C> regulateModuleConstantForSimulation(SwerveModuleConstants<A, B, C> moduleConstants) {
        // Skip regulation if running on a real robot
        if(RobotBase.isReal()) return moduleConstants;

        // Apply simulation-specific adjustments to module constants
        return moduleConstants
            // Disable encoder offsets
            .withEncoderOffset(0)
            // Disable motor inversions for drive and steer motors
            .withDriveMotorInverted(false)
            .withSteerMotorInverted(false)
            // Disable CanCoder inversion
            .withEncoderInverted(false)
            // Adjust steer motor PID gains for simulation
            .withSteerMotorGains(new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
            .withSteerMotorGearRatio(16.0)
            // Adjust friction voltages
            .withDriveFrictionVoltage(Volts.of(0.1))
            .withSteerFrictionVoltage(Volts.of(0.05))
            // Adjust steer inertia
            .withSteerInertia(KilogramSquareMeters.of(0.05));
    }
}
