package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        /**
         * The best tag transform that maps camera space to object space. 
         * Null if there are no tracked targets.
         */
        public Transform3d bestTagTransform = null;
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public PoseObservation[] poseObservations = new PoseObservation[0];
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
