package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record FieldBounds(double minX, double maxX, double minY, double maxY) {
    /** Whether the translation is contained within the bounds. */
    public boolean contains(Translation2d translation) {
        return translation.getX() >= minX()
            && translation.getX() <= maxX()
            && translation.getY() >= minY()
            && translation.getY() <= maxY();
    }

    /** Whether the pose is contained within the bounds. */
    public boolean contains(Pose2d pose) {
        return contains(pose.getTranslation());
    }

    /** Clamps the translation to the bounds. */
    public Translation2d clamp(Translation2d translation) {
        return new Translation2d(
            MathUtil.clamp(translation.getX(), minX(), maxX()),
            MathUtil.clamp(translation.getY(), minY(), maxY()));
    }
}