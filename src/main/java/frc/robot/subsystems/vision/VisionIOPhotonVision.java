package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    public final String name;

    /**
     * Creates a new VisionIOPhotonVision.
     */
    public VisionIOPhotonVision(CameraConfiguration config) {
        camera = new PhotonCamera(config.name());

        this.robotToCamera = config.position();
        this.name = config.name();

        if(robotToCamera == null) {
            DriverStation.reportWarning("Warning: camera " + config.name() + " does not have a configured position! This camera will be disabled.", false);
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        if(robotToCamera == null) {
            return;
        }

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        var results = camera.getAllUnreadResults();
        for(var result : results) {
            // Update latest target observation
            if(result.hasTargets()) {
                inputs.bestTagTransform = result.getBestTarget().getBestCameraToTarget();
            } else {
                inputs.bestTagTransform = null;
            }

            // Add pose observation
            if(result.multitagResult.isPresent()) { // Multitag result
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for(var target : result.targets)
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(new PoseObservation(
                    result.getTimestampSeconds(), // Timestamp
                    robotPose, // 3D pose estimate
                    multitagResult.estimatedPose.ambiguity, // Ambiguity
                    multitagResult.fiducialIDsUsed.size(), // Tag count
                    totalTagDistance / result.targets.size() // Average tag distance
                ));
            } else if(!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if(tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                        tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add observation
                    poseObservations.add(new PoseObservation(result.getTimestampSeconds(), // Timestamp
                        robotPose, // 3D pose estimate
                        target.poseAmbiguity, // Ambiguity
                        1, // Tag count
                        cameraToTarget.getTranslation().getNorm() // Average tag distance
                    ));
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for(int i = 0; i < poseObservations.size(); i++) inputs.poseObservations[i] = poseObservations.get(i);
    }

    @Override
    public String getName() {
        return name;
    }
}