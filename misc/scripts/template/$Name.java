package frc.robot.subsystems.$name;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class $Name extends SubsystemBase {
    private final $NameIO io;
    private final $NameIOInputsAutoLogged inputs = new $NameIOInputsAutoLogged();

    public $Name($NameIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("$Name", inputs);
    }

    public Command runPercent(double percent) {
        return runEnd(() -> io.setPower(percent * 12.0), () -> io.setPower(0.0));
    }
}