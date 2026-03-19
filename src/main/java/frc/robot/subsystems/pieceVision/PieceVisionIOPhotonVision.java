package frc.robot.subsystems.pieceVision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class PieceVisionIOPhotonVision implements PieceVisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    public final String name;
    
    /**
     * Creates a new PieceVisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public PieceVisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);

        this.robotToCamera = robotToCamera;
        this.name = name;
    }

    @Override
    public void updateInputs(PieceVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();
        
        var results = camera.getAllUnreadResults();
        for(var result : results) {
            // do something i guess
        }
    }
}
