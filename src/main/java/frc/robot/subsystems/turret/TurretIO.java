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
            /** The measured motor flywheel angular velocity. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double currentAmps
        ) {
            FlywheelMotorInputs half() {
                return new FlywheelMotorInputs(
                    connected,
                    velocityRadPerSec,
                    currentAmps / 2
                );
            }
        }
        public record AzimuthMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The azimuth motor's internal encoder angle in rad (in mechanism rotations). */
            double internalEncoderAngle,
            /** The azimuth motor's internal encoder velocity in rad/sec (in mechanism rotations). */
            double internalEncoderVelocity,
            /** The motor current draw. */
            double currentAmps
        ) {}
        public record HoodMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** 
             * The measured hood motor ring angle.
             * The actual hood angle is the difference between this and the azimuth ring angle (and reductions).
             */
            double angleRad,
            /** The measured velocity of the hood motor in rad/sec. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double currentAmps
        ) {}
        public record AzimuthEncoderInputs(
            /** Whether the encoder is connected */
            boolean connected,
            /** The measured azimuth motor angle, but based on the absolute encoder. */
            double angleRad,
            /** The measured absolute motor velocity in rad/sec, but based on the absolute encoder. */
            double velocityRadPerSec
        ) {}

        public FlywheelMotorInputs topFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);
        public FlywheelMotorInputs bottomFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);

        public AzimuthMotorInputs azimuth = new AzimuthMotorInputs(false, 0.0, 0.0, 0.0);
        public AzimuthEncoderInputs azimuthEncoder = new AzimuthEncoderInputs(false, 0.0, 0.0);

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
         * The physical gearing inverts the motor direction, so we negate totalFlywheelGearing
         * to keep a consistent positive-current to positive-velocity convention.
         */
        public double getFlywheelVelocityRadPerSecond() {
            return (
                topFlywheel.velocityRadPerSec() + bottomFlywheel.velocityRadPerSec()
            ) / 2 * TurretConstants.totalFlywheelGearing -
                azimuthEncoder.velocityRadPerSec() * TurretConstants.azimuthFlyCoupling;
        }
        public double getHoodAngleRad() {
            return -hood.angleRad() * TurretConstants.hoodRingToHoodReduction -
                azimuth.internalEncoderAngle / TurretConstants.totalAzimuthGearing * TurretConstants.azimuthHoodCoupling +
                TurretConstants.hoodMinAngle;
        }
        public double getHoodVelocityRadPerSec() {
            return hood.velocityRadPerSec() * TurretConstants.totalHoodGearing -
                azimuthEncoder.velocityRadPerSec() * TurretConstants.azimuthHoodCoupling;
        }
        public double getAzimuthAngleRad() {
            // return azimuthEncoder.angleRad() * TurretConstants.totalAzimuthGearing;
            return MathUtil.angleModulus(azimuth.internalEncoderAngle);
        }
        public double getAzimuthVelocityRadPerSec() {
            // return azimuthEncoder.velocityRadPerSec() * TurretConstants.totalAzimuthGearing;
            return azimuth.internalEncoderVelocity;
        }
    }

    public static record TurretIOPIDOutputs(
        /** The target flywheel speed. */
        double flywheelSpeedRadPerSec,
        /** The turret azimuth angle relative to the robot base. */
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
    public default void resetHoodToBottom() {}

    /** Stop all turret motion and hold position. */
    public default void stop() {}
}