package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public Command runRightPercent(double percent) {
        return runEnd(() -> io.setRightPower(percent * 12.0), () -> io.setRightPower(0.0));
    }

    public Command runLeftPercent(double percent) {
        return runEnd(() -> io.setLeftPower(percent * 12.0), () -> io.setLeftPower(0.0));
    }
}