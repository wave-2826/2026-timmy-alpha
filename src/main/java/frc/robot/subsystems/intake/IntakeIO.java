package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public record RollerMotorInputs( 
            /** Whether the motor is connected */
            boolean connected,
            /** The measured intake angular velocity. */
            double velocityRadPerSec,
            /** The motor current draw. */
            double currentAmps
        ) {}
        public record DeployMotorInputs(
            /** Whether the motor is connected */
            boolean connected,
            /** The motor current draw. */
            double currentAmps,
            /** The motor's position according to the encoder in meters */
            double motorPosition
        ) {}
        
        RollerMotorInputs roller = new RollerMotorInputs(false, 0.0, 0.0);
        DeployMotorInputs deployL = new DeployMotorInputs(false, 0.0, 0.0);
        DeployMotorInputs deployR = new DeployMotorInputs(false, 0.0, 0.0);
    }
    
    
    
    /** Update the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    /** Run open loop at the specified voltage. */
    public default void setRollerVoltage(double volts) {}
    public default void setDeployVoltageL(double volts) {}
    public default void setDeployVoltageR(double volts) {}
    
    public default void resetDeployEncoders() {}
    /** Set the deploy position relative to when the deploy encoders were last reset. */
    public default void setDeployPosition(double position) {}
    public default void stopDeploy() {}
}