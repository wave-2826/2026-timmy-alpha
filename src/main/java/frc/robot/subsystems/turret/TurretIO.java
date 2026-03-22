package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.MathUtil;

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
         * Get the flywheel mechanism velocity in rad/s. Positive = shooting direction.
         * The physical gearing inverts the motor direction, so we negate totalFlywheelGearing
         * to keep a consistent positive-current to positive-velocity convention.
         */
        public double getFlywheelVelocityRadPerSecond() {
            return (
                topFlywheel.velocityRadPerSec() + bottomFlywheel.velocityRadPerSec()
            ) / 2 * -TurretConstants.totalFlywheelGearing +
                azimuthEncoder.velocityRadPerSec() * -TurretConstants.azimuthFlyCoupling;
        }
        public double getHoodAngleRad() {
            return -hood.angleRad() * TurretConstants.hoodRingToHoodReduction -
                getAzimuthAngleRad()  * TurretConstants.azimuthHoodCoupling +
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
        /**  */
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

    /** Run the turret with the given outputs in PID control mode. */
    public default void setPIDOutputs(TurretIOPIDOutputs outputs) {}

    /** Run the turret with the given velocities only. */
    public default void setVelocityOutputs(double flywheelVelocityRadPerSec, double azimuthVelocityRadPerSec, double hoodVelocityRadPerSec) {}

    /** Run the turret with the given outputs in LQR control mode. */
    public default void setLQROutputs(TurretLQROutputs outputs) {}

    /** Reset the azimuth and hood to their zero position (flywheel facing directly toward pdh side, hood all the way down) */
    public default void resetAzimuthAndHood() {}

    /** Stop all turret motion and hold position. */
    public default void stop() {}
}