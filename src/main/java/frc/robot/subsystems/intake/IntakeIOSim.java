package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

// TODO: intake sim implementation

public class IntakeIOSim implements IntakeIO {
    private double appliedVolts = 0.0;
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        
    }
    
    @Override
    public void setRollerVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}