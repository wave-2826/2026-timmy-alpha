package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public record FlywheelMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured flywheel angular velocity. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps
        ) {}
        public record AzimuthMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured azimuth outer ring angle. */
            double azimuthAngleRad,
            /** The azimuth motor's internal encoder angle. */
            double azimuthInternalEncoderAngle,
            /** The measured velocity in rad/sec. */
            double azimuthVelocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps,
            /** The applied output as a percentage. */
            double appliedOutput
        ) {}
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

        FlywheelMotorInputs topFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);
        FlywheelMotorInputs bottomFlywheel = new FlywheelMotorInputs(false, 0.0, 0.0);

        AzimuthMotorInputs azimuth = new AzimuthMotorInputs(false, 0.0, 0.0, 0.0, 0.0, 0.0);

        HoodMotorInputs hood = new HoodMotorInputs(false, 0.0, 0.0, 0.0, 0.0);
    }

    public static record TurretIOPIDOutputs(
        /** The target flywheel speed. */
        double flywheelSpeedRadPerSec,
        /** The turret azimuth angle relative to the robot base. */
        double azimuthAngleRad,
        /** The angle of the hood outer ring relative to the azimuth position. */
        double hoodAngleRad
    ) {}

    public static record TurretIOMPCOutputs(
        /** The current to apply to the flywheel motors (half to each), in amps. */
        double flywheelCurrent,
        /** The current to apply to the hood motor, in amps. */
        double hoodCurrent,
        /** The current to apply to the azimuth motor, in amps. */
        double azimuthCurrent
    ) {}

    /** Update the set of loggable inputs - data measured from the turret and passed into code. */
    public default void updateInputs(TurretIOInputs inputs) {}

    /** Run the turret with the given outputs in PID control mode. */
    public default void setPIDOutputs(TurretIOPIDOutputs outputs) {}

    /** Run the turret with the given outputs in MPC control mode. */
    public default void setMPCOutputs(TurretIOMPCOutputs outputs) {}

    /** Stop all turret motion and hold position. */
    public default void stop() {}
}