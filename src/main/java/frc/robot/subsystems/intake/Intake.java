package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    
    public Command runRollerPercent(double percent) {
        return runEnd(() -> io.setRollerVoltage(percent * 12.0), () -> io.setRollerVoltage(0.0));
    }
    
    public Command runRollerTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
        return runEnd(
        () -> io.setRollerVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
        () -> io.setRollerVoltage(0.0));
    }
    
    public Command deployIntake() {
        return Commands.runEnd(
        () -> io.setDeployVoltage(2.4),
        () -> io.setDeployVoltage(0.0)
        )
        .until(() -> (inputs.deployL.motorCurrentAmps() + inputs.deployR.motorCurrentAmps()) / 2 > IntakeConstants.opeThatsaResetCurrent)
        .andThen(io::resetDeployEncoders);
    }
    
    public Command setIntakePosition(DoubleSupplier position) {
        return Commands.run(
        () -> {
            io.setDeployPosition(position.getAsDouble());
        }
        );
    }
    
    public Command bringIntakeIn(DoubleSupplier triggerPosition) {
        return setIntakePosition(() -> {
            return IntakeConstants.trackLengthMeters * triggerPosition.getAsDouble();
        });
    }
}