package frc.robot.subsystems.hopperVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperVision extends SubsystemBase {
    private HopperVisionIOInputsAutoLogged inputs = new HopperVisionIOInputsAutoLogged();
    private HopperVisionIO io;

    private static Alert disconnectedAlert = new Alert("Hopper vision camera disconnected!", AlertType.kError);

    public Command waitForNoPieces(double waitAfter, double fallbackWait, double timeout) {
        double startTime = Timer.getFPGATimestamp();
        Debouncer hasPiecesDebouncer = new Debouncer(0.5, DebounceType.kFalling);
        return Commands.sequence(
            Commands.waitUntil(() -> {
                if(!inputs.connected) {
                    // If we're not connected, wait the fallback time and hope for the best
                    return Timer.getFPGATimestamp() - startTime > fallbackWait;
                } else {
                    // If we're connected, wait until we see no targets
                    return !hasPiecesDebouncer.calculate(inputs.targets != 0);
                }
            }),
            Commands.waitSeconds(waitAfter)
        ).withTimeout(timeout);
    }

    public HopperVision(HopperVisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperVision", inputs);

        disconnectedAlert.set(!inputs.connected);
    }
}
