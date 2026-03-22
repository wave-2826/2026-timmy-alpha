package frc.robot.subsystems.hopperVision;

import frc.robot.util.simUtils.Simulation;

public class HopperVisionIOSim implements HopperVisionIO {
    @Override
    public void updateInputs(HopperVisionIOInputs inputs) {
        inputs.connected = true;
        inputs.targets = Math.min(10, Simulation.getInstance().getHopperFuel());
    }
}
