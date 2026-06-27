package frc.robot.subsystems.hopperVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Container;

public class HopperVision extends SubsystemBase {
    private HopperVisionIOInputsAutoLogged inputs = new HopperVisionIOInputsAutoLogged();
    private HopperVisionIO io;

    private static Alert disconnectedAlert = new Alert("Hopper vision camera disconnected!", AlertType.kError);

    public Command waitForFewerThanNPieces(int pieces, double waitAfter, double fallbackWait, double timeout) {
        Container<Double> startTime = new Container<Double>(0.);
        Debouncer ranOutDebouncer = new Debouncer(1.0, DebounceType.kRising);
        return Commands.sequence(
            Commands.runOnce(() -> {
                ranOutDebouncer.calculate(false);
                startTime.value = Timer.getFPGATimestamp();
            }),
            Commands.waitUntil(() -> {
                if(!inputs.connected) {
                    // If we're not connected, wait the fallback time and hope for the best
                    return Timer.getFPGATimestamp() - startTime.value > fallbackWait;
                } else {
                    // If we're connected, wait until we see no targets
                    return ranOutDebouncer.calculate(inputs.targets < pieces);
                }
            }),
            Commands.waitSeconds(waitAfter)
        ).withTimeout(timeout).withName("HopperVisionWait");
    }

    public HopperVision(HopperVisionIO io) {
        this.io = io;
        
        setFPSLimit(VisionConstants.disabledFPSLimit);
        RobotModeTriggers.disabled().onChange(Commands.runOnce(() -> {
            setFPSLimit(DriverStation.isEnabled() ? -1 : (int)Vision.overrideFPSLimit.get());
        }).ignoringDisable(true));
    }

    private void setFPSLimit(int fps) {
        io.setFPSLimit(fps);
    }

    @Override
    public void periodic() {
        if(Vision.overrideFPSLimit.hasChanged(hashCode())) {
            setFPSLimit(DriverStation.isEnabled() ? -1 : (int)Vision.overrideFPSLimit.get());
        }

        io.updateInputs(inputs);
        Logger.processInputs("HopperVision", inputs);

        disconnectedAlert.set(!inputs.connected);
    }
}
