package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.simUtils.Simulation;

import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Vision extends SubsystemBase {
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final static LoggedNetworkBoolean overrideFPSLimitField = new LoggedNetworkBoolean("Tuning/Vision/OverrideFPSLimit", false);
    public final static Trigger overrideFPSLimit = new Trigger(overrideFPSLimitField::getAsBoolean);

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

        setFPSLimit(VisionConstants.disabledFPSLimit);
        RobotModeTriggers.disabled().or(overrideFPSLimit).onChange(Commands.runOnce(() -> {
            setFPSLimit(DriverStation.isEnabled() || overrideFPSLimit.getAsBoolean() ? -1 : VisionConstants.disabledFPSLimit);
        }).ignoringDisable(true));
    }

    private void setFPSLimit(int toFPS) {
        for(var camera : io) {
            camera.limitFPS(toFPS);
        }
    }

    public int getCameraCount() {
        return io.length;
    }

    @Override
    @SuppressWarnings("unused")
    public void periodic() {
        if(Constants.isSim && !VisionConstants.enableVisionSimulation) {
            RobotState.getInstance().addVisionMeasurement(
                Simulation.getInstance().driveSimulation.getSimulatedDriveTrainPose(),
                (double)NetworkTablesJNI.now() * 1e-6,
                VecBuilder.fill(0.01, 0.01, 5.0)
            );
            return;
        }

        for(int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        var robotState = RobotState.getInstance();

        int disconnectedCameras = 0;

        // Loop over cameras
        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            disconnectedCameras += inputs[cameraIndex].connected ? 0 : 1;

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

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

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        robotState.setDroppedCameraCount(disconnectedCameras);

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted",
            allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected",
            allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    /**
     * Get the best known transforms based on tags for each camera.
     */
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