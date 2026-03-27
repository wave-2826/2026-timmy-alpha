package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    public Spindexer(SpindexerIO io) {
        this.io = io;
    }

    @AutoLogOutput(key = "Spindexer/BallsPerSecond")
    public double getBallsPerSecond() {
        return SpindexerConstants.ballsInSpin * inputs.spinner.velocityRadPerSec() / (2 * Math.PI);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);
    }

    public Command runManual(DoubleSupplier percent) {
        return runPercent(
            () -> percent.getAsDouble(),
            () -> Math.pow(Math.abs(percent.getAsDouble()), 0.1)
        );
    }

    public Command runPercent(DoubleSupplier spinPercent, DoubleSupplier transferPercent) {
        return runEnd(() -> {
            setPower(spinPercent.getAsDouble(), transferPercent.getAsDouble());
        }, () -> {
            setPower(0.0, 0.0);
        });
    }

    public void setPower(double spinPercent, double transferPercent) {
        io.setSpinnerVoltage(spinPercent * 10.0);
        io.setTransferVoltage(transferPercent * 12.0);
    }
}