package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer(SpindexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
    }

    public Command runSpinnerPercent(DoubleSupplier percent) {
        return runEnd(() -> io.setSpinnerVoltage(percent.getAsDouble() * 12.0), () -> io.setSpinnerVoltage(0.0));
    }

    public Command runTransferPercent(DoubleSupplier percent) {
        return runEnd(() -> io.setTransferVoltage(percent.getAsDouble() * 12.0), () -> io.setTransferVoltage(0.0));
    }

    public Command runAllPercent(DoubleSupplier percent) {
        return runEnd(() -> {
            double volts = percent.getAsDouble() * 12.0;
            io.setSpinnerVoltage(volts);
            io.setTransferVoltage(volts);
        }, () -> {
            io.setSpinnerVoltage(0.0);
            io.setTransferVoltage(0.0);
        });
    }
}