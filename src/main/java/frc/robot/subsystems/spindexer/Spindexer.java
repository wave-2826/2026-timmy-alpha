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

    public Command runPercent(double percent) {
        return runEnd(() -> io.setPower(percent * 12.0), () -> io.setPower(0.0));
    }
}