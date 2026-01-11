package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
     *
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);

        this.robotToCamera = robotToCamera;
        this.name = name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        var results = camera.getAllUnreadResults();
        for(var result : results) {
            // Update latest target observation
            if(result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
                inputs.bestTagTransform = result.getBestTarget().getBestCameraToTarget();
            } else {
                inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
                inputs.bestTagTransform = new Transform3d();
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
                poseObservations.add(new PoseObservation(result.getTimestampSeconds(), // Timestamp
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

        if(results.size() > 0) {
            var latestResult = results.get(results.size() - 1);
            if(latestResult.hasTargets()) {
                inputs.individualTags = new SingleApriltagResult[latestResult.targets.size()];
                for(int i = 0; i < latestResult.targets.size(); i++) {
                    var target = latestResult.targets.get(i);
                    Transform3d robotToTarget = robotToCamera.plus(target.bestCameraToTarget);

                    inputs.individualTags[i] = new SingleApriltagResult(target.fiducialId, robotToTarget,
                        target.poseAmbiguity, latestResult.getTimestampSeconds());
                }
            } else {
                inputs.individualTags = null;
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