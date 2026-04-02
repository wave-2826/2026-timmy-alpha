package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveConnected = false;
        /* The drive position as radians of the wheel. */
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAccelerationRadPerSec2 = 0.0;
        public double driveAppliedVolts = 0.0;
        /* Stator current */
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d uncorrectedTurnAbsolute = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        /* Stator current */
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop current in amps. */
    public default void setDriveOpenLoopCurrent(double currentAmps) {}
    public default void setDriveOpenLoopVoltage(double voltageVolts) {}
    public default void setTurnOpenLoopVoltage(double voltageVolts) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec, double ffForceNM) {}

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {}

    /** Set P, I, and D gains for closed loop control on drive motor. */
    public default void setDrivePID(double kP, double kI, double kD, double kS, double kV, double kA) {}

    /** Set P gain, I gain, D gain, and derivative filter for closed loop control on turn motor. */
    public default void setTurnPID(double kP, double kI, double kD) {}
    
    /** Set the neutral mode of the drive motor; used so we can set them to coast when the robot is disabled. */
    public default void setDriveNeutralModeCoast(boolean coast) {}

    /** Set the neutral mode of the turn motor; used so we can set them to coast when the robot is disabled. */
    public default void setTurnNeutralModeCoast(boolean coast) {}

    /** Temporarily override the drive motor current limit for slip current characterization. */
    public default void setSlipMeasurementCurrentLimit(Current current) {}
}
