package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.rollerMotorReduction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs.DeployMotorInputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs.RollerMotorInputs;

// TODO: better intake sim implementation

public class IntakeIOSim implements IntakeIO {
    private DCMotor rollerMotor = DCMotor.getNeoVortex(1);
    private DCMotorSim roller = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(rollerMotor, 0.2, IntakeConstants.rollerMotorReduction),
        rollerMotor
    );

    private double rollerVoltage = 0.0;
    private double deployPos = 0.0;
    
    @Override
    public void setDeployPosition(double position) {
        deployPos = position;
    }

    @Override
    public void resetDeployEncoders() {
        deployPos = 0.0;
    }

    @Override
    public void setDeployPowerL(double power) {
        //
    }
    @Override
    public void setDeployPowerR(double power) {
        //
    }
    @Override
    public void setRollerPower(double power) {
        //
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if(!DriverStation.isEnabled()) rollerVoltage = 0.0;
        
        roller.setInputVoltage(rollerVoltage);
        roller.update(0.02);

        inputs.deployL = new DeployMotorInputs(
            true,
            IntakeConstants.deployStallCurrent + 1.0,
            deployPos
        );
        inputs.deployR = inputs.deployL;
        inputs.roller = new RollerMotorInputs(
            true,
            roller.getAngularVelocityRadPerSec(),
            roller.getCurrentDrawAmps()
        );
    }
}