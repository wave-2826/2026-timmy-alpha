package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;

public class IntakeIOSim implements IntakeIO {
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

  }

  @Override
  public void setRollerVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setDeployVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}