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

    public Command runPercent(DoubleSupplier percent) {
        return runEnd(() -> {
            setPower(percent.getAsDouble());
        }, () -> {
            setPower(0.0);
        });
    }

    public void setPower(double percent) {
        double volts = percent * 10.0;
        io.setSpinnerVoltage(volts);
        io.setTransferVoltage(volts);
    }
}