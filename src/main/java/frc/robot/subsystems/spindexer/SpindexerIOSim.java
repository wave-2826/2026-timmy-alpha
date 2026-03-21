package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOInputs.SpinnerMotorInputs;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOInputs.TransferMotorInputs;

public class SpindexerIOSim implements SpindexerIO {
    DCMotor spinnerMotor = DCMotor.getNeoVortex(1);
    DCMotor transferMotor = DCMotor.getNeo550(1);

    DCMotorSim spinner = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(spinnerMotor, 0.05, SpindexerConstants.spinnerMotorReduction),
        spinnerMotor
    );
    DCMotorSim transfer = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(transferMotor, 0.05, SpindexerConstants.transferMotorReduction),
        transferMotor
    );

    double spinnerVoltage = 0.0;
    double transportVoltage = 0.0;

    public SpindexerIOSim() {

    }

    @Override
    public void setSpinnerVoltage(double volts) {
        spinnerVoltage = volts;
    }

    @Override
    public void setTransferVoltage(double volts) {
        transportVoltage = volts;
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        if(!DriverStation.isEnabled()) {
            spinnerVoltage = 0.0;
            transportVoltage = 0.0;
        }
        
        spinner.setInputVoltage(spinnerVoltage);
        transfer.setInputVoltage(transportVoltage);

        spinner.update(0.02);
        transfer.update(0.02);

        inputs.spinner = new SpinnerMotorInputs(
            true,
            spinner.getAngularVelocityRadPerSec(),
            spinner.getCurrentDrawAmps()
        );
        inputs.transfer = new TransferMotorInputs(
            true,
            transfer.getAngularVelocityRadPerSec(),
            transfer.getCurrentDrawAmps()
        );
    }
}
