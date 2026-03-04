package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.turret.controller.TurretControllerIO.TurretMPCOutputs;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public record FlywheelMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured flywheel angular velocity at the flywheel. */
            double flywheelVelocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps
        ) {
            FlywheelMotorInputs half() {
                return new FlywheelMotorInputs(
                    connected,
                    flywheelVelocityRadPerSec,
                    motorCurrentAmps / 2
                );
            }
        }
        public record AzimuthMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured absolute azimuth angle. */
            double azimuthAngleRad,
            /** The azimuth motor's internal encoder angle. */
            double azimuthInternalEncoderAngle,
            /** The measured absolute velocity in rad/sec. */
            double azimuthVelocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps,
            /** The applied output as a percentage. */
            double appliedOutput
        ) {
            public AzimuthMotorInputs withAngle(double azimuthANgleRad) {
                return new AzimuthMotorInputs(
                    connected,
                    azimuthANgleRad,
                    azimuthInternalEncoderAngle,
                    azimuthVelocityRadPerSec,
                    motorCurrentAmps,
                    appliedOutput
                );
            }
        }
        public record HoodMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** 
             * The measured hood ring angle.
             * The actual hood angle is the difference between this and the azimuth ring angle (and a reduction).
             */
            double hoodRingAngleRad,
            /** The meas ured velocity of the hood ring in rad/sec. */
            double hoodRingVelocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps,
            /** The applied output as a percentage. */
            double appliedOutput
        ) {}

        public FlywheelMotorInputs topFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);
        public FlywheelMotorInputs bottomFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);

        public AzimuthMotorInputs azimuth = new AzimuthMotorInputs(false, 0.0, 0.0, 0.0, 0.0, 0.0);

        public HoodMotorInputs hood = new HoodMotorInputs(false, 0.0, 0.0, 0.0, 0.0);


        public double getFlywheelVelocityRadPerSecond() {
            return (
                topFlywheel.flywheelVelocityRadPerSec() + bottomFlywheel.flywheelVelocityRadPerSec()
            ) / 2 + azimuth.azimuthVelocityRadPerSec() * TurretConstants.azimuthFlyCoupling;
        }
        public double getHoodAngleRad() {
            return hood.hoodRingAngleRad() * TurretConstants.hoodRingToHoodReduction -
                azimuth.azimuthAngleRad() * TurretConstants.azimuthHoodCoupling +
                TurretConstants.hoodMinAngle;
            // TODO: Store an offset?
        }
        public double getHoodVelocityRadPerSec() {
            return hood.hoodRingVelocityRadPerSec() -
                azimuth.azimuthVelocityRadPerSec() * TurretConstants.azimuthHoodCoupling;
        }
        public double getAzimuthAngleRad() {
            return azimuth.azimuthAngleRad();
        }
        public double getAzimuthVelocityRadPerSec() {
            return azimuth.azimuthVelocityRadPerSec();
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

    /** Update the set of loggable inputs - data measured from the turret and passed into code. */
    public default void updateInputs(TurretIOInputs inputs) {}

    /** Run the turret with the given outputs in PID control mode. */
    public default void setPIDOutputs(TurretIOPIDOutputs outputs) {}

    /** Run the turret with the given outputs in MPC control mode. */
    public default void setMPCOutputs(TurretMPCOutputs outputs) {}

    /** Stop all turret motion and hold position. */
    public default void stop() {}
}