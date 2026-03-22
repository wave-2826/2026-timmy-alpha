package frc.robot.subsystems.hopperVision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperVision extends SubsystemBase {
    private HopperVisionIOInputsAutoLogged inputs = new HopperVisionIOInputsAutoLogged();
    private HopperVisionIO io;

    public Command waitForNoPieces(double waitAfter, double fallbackWait, double timeout) {
        double startTime = Timer.getFPGATimestamp();
        return Commands.sequence(
            Commands.waitUntil(() -> {
                if(!inputs.connected) {
                    // If we're not connected, wait the fallback time and hope for the best
                    return Timer.getFPGATimestamp() - startTime > fallbackWait;
                } else {
                    // If we're connected, wait until we see no targets
                    return inputs.targets == 0;
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
    }
}
