package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.turret.Turret.ControlMode;
import frc.robot.subsystems.turret.Turret.TurretTarget;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public record FlywheelMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /**
             * The measured flywheel angular velocity, not including coaxial coupling.
             * This is purely motor velocity times the mechanism ratio.
             */
            double velocityRadPerSec,
            /** The motor current draw. */
            double currentAmps,
            double temperatureCelsius
        ) {}
        public record AzimuthMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The azimuth motor's internal encoder angle in rad (in CCW mechanism radians). */
            double internalEncoderAngle,
            /** The azimuth motor's internal encoder velocity in rad/sec (in CCW mechanism radians/sec). */
            double internalEncoderVelocity,
            /** The motor current draw. */
            double currentAmps
        ) {}
        public record HoodMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** 
             * The measured hood motor ring angle (CCW positive).
             * The actual hood angle is the difference between this and the azimuth ring angle (and reductions).
             */
            double angleRad,
            /** The measured velocity of the hood ring in rad/sec. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double currentAmps
        ) {}

        public FlywheelMotorInputs flywheel1 = new FlywheelMotorInputs(false, 0.0, 0.0, 0.0);
        public FlywheelMotorInputs flywheel2 = new FlywheelMotorInputs(false, 0.0, 0.0, 0.0);

        public AzimuthMotorInputs azimuth = new AzimuthMotorInputs(false, 0.0, 0.0, 0.0);

        public HoodMotorInputs hood = new HoodMotorInputs(false, 0.0, 0.0, 0.0);

        /**
         * Only used when dealing with high-frequency controllers;
         * the number of loop updates since the last inputs update.
         */
        public int loopUpdates = 0;
        /** The state of the LQR kalman observer. */
        public double[] LQRKalmanState = new double[5];

        public boolean azimuthZeroTriggered = false;

        /**
         * Get the flywheel mechanism velocity in rad/s. Positive = shooting direction.
         */
        public double getFlywheelVelocityRadPerSecond() {
            return (
                flywheel1.velocityRadPerSec() + flywheel2.velocityRadPerSec()
            ) / 2 - azimuth.internalEncoderVelocity() * TurretConstants.azimuthFlyCoupling;
        }
        public double getHoodAngleRad() {
            return (azimuth.internalEncoderAngle - hood.angleRad) * TurretConstants.hoodRingToHoodReduction +
                TurretConstants.hoodMinAngle;
        }
        public double getHoodVelocityRadPerSec() {
            return hood.velocityRadPerSec() * TurretConstants.totalHoodGearing -
                azimuth.internalEncoderVelocity() * TurretConstants.azimuthHoodCoupling;
        }
        public double getAzimuthAngleRad() {
            return MathUtil.angleModulus(azimuth.internalEncoderAngle);
        }
        public double getAzimuthVelocityRadPerSec() {
            return azimuth.internalEncoderVelocity;
        }
    }

    public static record TurretIOPIDOutputs(
        /** The target flywheel speed at the wheel itself. */
        double flywheelSpeedRadPerSec,
        /** The turret azimuth angle relative to the robot base, in counterclockwise rotations */
        double azimuthAngleRad,
        /** The angle of the hood relative to its minimum. */
        double hoodAngleRad
    ) {}

    public static record TurretLQROutputs(
        /** Per-motor current */
        double flywheelCurrent,
        double azimuthCurrent,
        double hoodCurrent
    ) {}

    /** Update the set of loggable inputs - data measured from the turret and passed into code. */
    public default void updateInputs(TurretIOInputs inputs) {}

    /** Set the current turret control mode. This usually doesn't need to do anything. */
    public default void setControlMode(ControlMode mode) {}

    /** Run the turret with the given outputs in LQR control mode. */
    public default void setTarget(TurretTarget target) {}

    /** Run the turret with the given outputs in PID control mode. */
    public default void setPIDOutputs(TurretIOPIDOutputs outputs) {}

    /** Run the turret with the given velocities only. */
    public default void setVelocityOutputs(double flywheelVelocityRadPerSec, double azimuthVelocityRadPerSec, double hoodVelocityRadPerSec) {}

    public default void resetAzimuth(Rotation2d angle) {}
    public default void resetHoodTo(double angleRad) {}

    /** Stop all turret motion and hold position. */
    public default void stop() {}
}