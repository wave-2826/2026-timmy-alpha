package frc.robot.subsystems.hopperVision;

import org.photonvision.PhotonCamera;

public class HopperVisionIOPhoton implements HopperVisionIO {
    private final PhotonCamera camera;

    public HopperVisionIOPhoton() {
        camera = new PhotonCamera("SpinCam");
    }

    @Override
    public void updateInputs(HopperVisionIOInputs inputs) {
        // TODO
        inputs.connected = camera.isConnected();
        inputs.targets = 0;
        // for()
    }
}
