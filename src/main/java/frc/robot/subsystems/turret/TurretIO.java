package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public record FlywheelMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured motor flywheel angular velocity. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps
        ) {
            FlywheelMotorInputs half() {
                return new FlywheelMotorInputs(
                    connected,
                    velocityRadPerSec,
                    motorCurrentAmps / 2
                );
            }
        }
        public record AzimuthMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured azimuth motor angle, but based on the absolute encoder. */
            double angleRad,
            /** The azimuth motor's internal encoder angle. */
            double internalEncoderAngle,
            /** The measured absolute motor velocity in rad/sec, but based on the absolute encoder. */
            double velocityRadPerSec,
            /** The azimuth motor's internal encoder velocity in rad/sec. */
            double internalEncoderVelocity,
            /** The motor current draw. */
            double currentAmps,
            /** The applied output as a percentage. */
            double appliedOutput
        ) {
            public AzimuthMotorInputs withAngle(double azimuthANgleRad) {
                return new AzimuthMotorInputs(
                    connected,
                    angleRad,
                    internalEncoderAngle,
                    velocityRadPerSec,
                    internalEncoderVelocity,
                    currentAmps,
                    appliedOutput
                );
            }
        }
        public record HoodMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** 
             * The measured hood motor angle.
             * The actual hood angle is the difference between this and the azimuth ring angle (and a reduction).
             */
            double angleRad,
            /** The measured velocity of the hood motor in rad/sec. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double currentAmps,
            /** The applied output as a percentage. */
            double appliedOutput
        ) {}

        public FlywheelMotorInputs topFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);
        public FlywheelMotorInputs bottomFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);

        public AzimuthMotorInputs azimuth = new AzimuthMotorInputs(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        public HoodMotorInputs hood = new HoodMotorInputs(false, 0.0, 0.0, 0.0, 0.0);


        /**
         * Get the flywheel mechanism velocity in rad/s. Positive = shooting direction.
         * The physical gearing inverts the motor direction, so we negate totalFlywheelGearing
         * to keep a consistent positive-current to positive-velocity convention.
         */
        public double getFlywheelVelocityRadPerSecond() {
            return (
                topFlywheel.velocityRadPerSec() + bottomFlywheel.velocityRadPerSec()
            ) / 2 * -TurretConstants.totalFlywheelGearing +
                azimuth.velocityRadPerSec() * -TurretConstants.azimuthFlyCoupling;
        }
        public double getHoodAngleRad() {
            return hood.angleRad() * TurretConstants.totalHoodGearing -
                azimuth.angleRad() * TurretConstants.azimuthHoodCoupling +
                TurretConstants.hoodMinAngle;
            // TODO: Store an offset?
        }
        public double getHoodVelocityRadPerSec() {
            return hood.velocityRadPerSec() * TurretConstants.totalHoodGearing -
                azimuth.velocityRadPerSec() * TurretConstants.azimuthHoodCoupling;
        }
        public double getAzimuthAngleRad() {
            return azimuth.angleRad() * TurretConstants.totalAzimuthGearing;
        }
        public double getAzimuthVelocityRadPerSec() {
            return azimuth.velocityRadPerSec() * TurretConstants.totalAzimuthGearing;
        }
    }

    public static record TurretIOPIDOutputs(
        /** The target flywheel speed. */
        double flywheelSpeedRadPerSec,
        /** The turret azimuth angle relative to the robot base. */
        double azimuthAngleRad,
        /** The angle of the hood outer ring relative to the azimuth position. */
        double hoodAngleRad
    ) {}

    public static record TurretLQROutputs(
        double flywheelCurrent,
        double azimuthCurrent,
        double hoodCurrent
    ) {}

    /** Update the set of loggable inputs - data measured from the turret and passed into code. */
    public default void updateInputs(TurretIOInputs inputs) {}

    /** Run the turret with the given outputs in PID control mode. */
    public default void setPIDOutputs(TurretIOPIDOutputs outputs) {}

    /** Run the turret with the given outputs in LQR control mode. */
    public default void setLQROutputs(TurretLQROutputs outputs) {}

    /** Stop all turret motion and hold position. */
    public default void stop() {}
}