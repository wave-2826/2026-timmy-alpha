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
            /** The motor's position according to the encoder in meters. Positive numbers are outward. */
            double positionMeters
        ) {}
        
        RollerMotorInputs rollerL = new RollerMotorInputs(false, 0.0, 0.0);
        RollerMotorInputs rollerR = new RollerMotorInputs(false, 0.0, 0.0);
        DeployMotorInputs deployL = new DeployMotorInputs(false, 0.0, 0.0);
        DeployMotorInputs deployR = new DeployMotorInputs(false, 0.0, 0.0);
    }
    
    
    
    /** Update the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    /** Run open loop at the specified velocity in RPM. */
    public default void setRollerSpeed(double velocityRPM) {}
    /** Run open loop at the specified duty cycle. Positive numbers are outward. */
    public default void setDeployPowerL(double power) {}
    /** Run open loop at the specified duty cycle. Positive numbers are outward. */
    public default void setDeployPowerR(double power) {}
    
    public default void resetDeployEncoders() {}
    /** Set the deploy position relative to when the deploy encoders were last reset. Positive numbers are inward. */
    public default void setDeployPosition(double position) {}
    /** Stops deploy. */
    public default void stopDeploy() {}
}