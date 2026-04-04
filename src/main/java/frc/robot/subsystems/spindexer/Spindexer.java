package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private final Alert spinDisconnectedAlert = new Alert("Spin motor disconnected!", AlertType.kError);
    private final Alert transferDisconnectedAlert = new Alert("Dexter motor disconnected!", AlertType.kError);

    public Spindexer(SpindexerIO io) {
        this.io = io;
    }

    @AutoLogOutput(key = "Spindexer/BallsPerSecond")
    public double getBallsPerSecond() {
        return SpindexerConstants.ballsInSpin * inputs.spinner.velocityRadPerSec() / (2 * Math.PI)
            / 2.0 // half of the ball is spun
            / 5.0; // 400% loss oops
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Spindexer", inputs);

        spinDisconnectedAlert.set(!inputs.spinner.connected());
        transferDisconnectedAlert.set(!inputs.transfer.connected());
    }

    public Command runManual(DoubleSupplier percent, DoubleSupplier percentOverride, BooleanSupplier runDexter) {
        return runPercent(
            () -> percent.getAsDouble() + percentOverride.getAsDouble(),
            () -> (
                runDexter.getAsBoolean() ? Math.pow(Math.abs(percent.getAsDouble()), 0.1) * 0.7 : 0.
            ) + percentOverride.getAsDouble()
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
        io.setSpinnerVoltage(spinPercent * 12.0);
        io.setTransferVoltage(transferPercent * 12.0);
    }
}