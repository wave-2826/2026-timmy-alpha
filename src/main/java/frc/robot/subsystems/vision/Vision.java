package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.SingleApriltagResult;
import frc.robot.util.tunables.LoggedTunableNumber;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private static final boolean ENABLE_SINGLE_TAG_OBSERVATIONS = false;

    public static final LoggedTunableNumber perTagPersistenceTime = new LoggedTunableNumber(
        "Vision/PerTagPersistenceTime", 0.05);

    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public static record IndividualTagEstimate(Pose2d robotPose, double ambiguity, double timestamp) {
    }

    public Vision(VisionIO... io) {
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for(int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for(int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision camera " + io[i].getName() + " is disconnected.",
                AlertType.kWarning);
        }
    }

    public int getCameraCount() {
        return io.length;
    }

    @Override
    @SuppressWarnings("unused")
    public void periodic() {
        if(Constants.isSim && !VisionConstants.enableVisionSimulation) { return; }

        for(int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        List<Pose3d> allIndividualTagRobotPoses = new LinkedList<>();
        List<Pose3d> allIndividualTagRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allIndividualTagRobotPosesRejected = new LinkedList<>();

        var robotState = RobotState.getInstance();

        // Loop over cameras
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            List<Pose3d> individualTagRobotPoses = new LinkedList<>();
            List<Pose3d> individualTagRobotPosesAccepted = new LinkedList<>();
            List<Pose3d> individualTagRobotPosesRejected = new LinkedList<>();

            // Add tag poses
            if(inputs[cameraIndex].individualTags != null) {
                for(SingleApriltagResult result : inputs[cameraIndex].individualTags) {
                    int id = result.fiducialId();

                    var tagPoseMaybe = aprilTagLayout.getTagPose(id);
                    if(!tagPoseMaybe.isPresent()) continue;
                    var tagPose = tagPoseMaybe.get();
                    tagPoses.add(tagPose);

                    // Add to robotToIndividualTags map if it isn't obviously wrong
                    Transform3d robotToTag = result.robotToTarget();
                    Pose3d robotPose = tagPose.plus(robotToTag.inverse());
                    individualTagRobotPoses.add(robotPose);

                    boolean rejectPose = result.ambiguity() > maxAmbiguity || // Cannot be high ambiguity
                        Math.abs(robotPose.getZ()) > maxZError // Must have realistic Z coordinate
                        // Must be within the field boundaries
                        || robotPose.getX() < 0.0 || robotPose.getX() > aprilTagLayout.getFieldLength()
                        || robotPose.getY() < 0.0 || robotPose.getY() > aprilTagLayout.getFieldWidth();

                    if(rejectPose) {
                        individualTagRobotPosesRejected.add(robotPose);
                        continue;
                    }

                    Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotToTag" + id,
                        robotToTag);
                    individualTagRobotPosesAccepted.add(robotPose);

                    if(ENABLE_SINGLE_TAG_OBSERVATIONS) {
                        robotState.addIndividualTagObservation(robotPose.toPose2d(), result.captureTimestamp(),
                            result.ambiguity(), id);
                    }
                }
            }

            // Loop over pose observations
            for(var observation : inputs[cameraIndex].poseObservations) {
                var pose = observation.pose();

                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                    || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) // Cannot be high
                    // Ambiguity
                    || Math.abs(pose.getZ()) > maxZError // Must have realistic Z coordinate

                    // Must be within the field boundaries
                    || pose.getX() < 0.0 || pose.getX() > aprilTagLayout.getFieldLength() || pose.getY() < 0.0
                    || pose.getY() > aprilTagLayout.getFieldWidth()

                    || Math.abs(pose.getRotation().toRotation2d().minus(robotState.getRotation())
                        .getDegrees()) > maxRotationError;

                // Add pose to log
                robotPoses.add(pose);
                if(rejectPose) robotPosesRejected.add(pose);
                else robotPosesAccepted.add(pose);

                // Skip if rejected
                if(rejectPose) continue;

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if(cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                robotState.addVisionMeasurement(pose.toPose2d(), observation.timestamp(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/IndividualTagRobotPoses",
                individualTagRobotPoses.toArray(new Pose3d[individualTagRobotPoses.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/IndividualTagRobotPosesAccepted",
                individualTagRobotPosesAccepted.toArray(new Pose3d[individualTagRobotPosesAccepted.size()]));
            Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/IndividualTagRobotPosesRejected",
                individualTagRobotPosesRejected.toArray(new Pose3d[individualTagRobotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);

            allIndividualTagRobotPoses.addAll(individualTagRobotPoses);
            allIndividualTagRobotPosesAccepted.addAll(individualTagRobotPosesAccepted);
            allIndividualTagRobotPosesRejected.addAll(individualTagRobotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        Logger.recordOutput("Vision/Summary/IndividualTagRobotPoses",
            allIndividualTagRobotPoses.toArray(new Pose3d[allIndividualTagRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/IndividualTagRobotPosesRejected",
            allIndividualTagRobotPosesRejected.toArray(new Pose3d[allIndividualTagRobotPosesRejected.size()]));
        Logger.recordOutput("Vision/Summary/IndividualTagRobotPosesAccepted",
            allIndividualTagRobotPosesAccepted.toArray(new Pose3d[allIndividualTagRobotPosesAccepted.size()]));
    }

    public Transform3d[] getBestTagTransforms() {
        Transform3d[] results = new Transform3d[io.length];
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            results[cameraIndex] = inputs[cameraIndex].bestTagTransform;
        }
        return results;
    }

    public String[] getCameraNames() {
        String[] names = new String[io.length];
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            names[cameraIndex] = io[cameraIndex].getName();
        }
        return names;
    }
}