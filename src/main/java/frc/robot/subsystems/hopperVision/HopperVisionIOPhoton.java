package frc.robot.subsystems.hopperVision;

public class HopperVisionIOPhoton implements HopperVisionIO {
    @Override
    public void updateInputs(HopperVisionIOInputs inputs) {
        // TODO
        inputs.connected = false;
        inputs.targets = 0;
    }
}
