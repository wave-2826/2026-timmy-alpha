package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs.DeployMotorInputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs.RollerMotorInputs;

// TODO: better intake sim implementation

public class IntakeIOSim implements IntakeIO {
    // private DCMotor rollerMotor = DCMotor.getNeoVortex(1);
    // private DCMotorSim roller = new DCMotorSim(
    //     LinearSystemId.createDCMotorSystem(rollerMotor, 0.2, IntakeConstants.rollerMotorReduction),
    //     rollerMotor
    // );

    private double rollerSpeed = 0.0;
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
    public void setRollerSpeed(double velocityRPM) {
        rollerSpeed = Units.radiansPerSecondToRotationsPerMinute(velocityRPM);
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // if(!DriverStation.isEnabled()) rollerVoltage = 0.0;
        // roller.setInputVoltage(rollerVoltage);
        // roller.update(0.02);

        inputs.deployL = new DeployMotorInputs(
            true,
            IntakeConstants.deployStallCurrent + 1.0,
            deployPos
        );
        inputs.deployR = inputs.deployL;
        inputs.roller = new RollerMotorInputs(
            true,
            rollerSpeed,
            0.0
        );
    }
}