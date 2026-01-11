package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public Transform3d bestTagTransform = new Transform3d();
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public PoseObservation[] poseObservations = new PoseObservation[0];
        /**
         * Individual tag results. Only stores the latest results because this is used for drive feedback, not odometry.
         */
        public SingleApriltagResult[] individualTags = new SingleApriltagResult[0];
    }

    /** Represents a robot transform based on an individual Apriltag. */
    public static record SingleApriltagResult(int fiducialId, Transform3d robotToTarget, double ambiguity,
        double captureTimestamp) {
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
        double averageTagDistance) {
    }
    
    default void updateInputs(VisionIOInputs inputs) {}

    default String getName() {
        return "";
    }
}
