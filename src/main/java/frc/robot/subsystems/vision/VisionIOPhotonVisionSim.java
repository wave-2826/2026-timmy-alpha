package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if(visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setFPS(25.);

        cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(79.1));
        cameraProperties.setCalibError(0.35, 0.10);
        cameraProperties.setAvgLatencyMs(50);
        cameraProperties.setLatencyStdDevMs(15);

        cameraSim = new PhotonCameraSim(camera, cameraProperties, VisionConstants.aprilTagLayout);
        visionSim.addCamera(cameraSim, robotToCamera);

        // Enable the raw and processed streams. These are enabled by default.
        cameraSim.enableRawStream(VisionConstants.enableRawStreams);
        cameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        if(VisionConstants.enableWireframeDrawing) {
            cameraSim.enableDrawWireframe(true);
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if(VisionConstants.enableVisionSimulation) {
            visionSim.update(poseSupplier.get());
            super.updateInputs(inputs);
        }
    }
}