package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public record SpinnerMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured spinner angular velocity. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps
        ) {}
        public record TransferMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The measured transfer angular velocity. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double motorCurrentAmps
        ) {}
        
        SpinnerMotorInputs spinner = new SpinnerMotorInputs(false, 0.0, 0.0);
        TransferMotorInputs transfer = new TransferMotorInputs(false, 0.0, 0.0);
    }
    
    /** Update the set of loggable inputs. */
    public default void updateInputs(SpindexerIOInputs inputs) {}
    
    public default void setSpinnerPower(double percent) {}
    public default void setTransferPower(double percent) {}
}