package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec, double accelerationRadPerSec2) {}

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {}

    /** Set P, I, and D gains for closed loop control on drive motor. */
    public default void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {}

    /** Set P gain, I gain, D gain, and derivative filter for closed loop control on turn motor. */
    public default void setTurnPID(double kP, double kI, double kD) {}

    /** Temporarily override the drive motor current limit for slip current characterization. */
    public default void setSlipMeasurementCurrentLimit(Current current) {}
}
