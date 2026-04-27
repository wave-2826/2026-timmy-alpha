package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public record FieldBounds(double minX, double maxX, double minY, double maxY) {
    public FieldBounds(Translation2d center, double sizeX, double sizeY) {
        this(
            center.getX() - sizeX / 2,
            center.getX() + sizeX / 2,
            center.getY() - sizeY / 2,
            center.getY() + sizeY / 2
        );
    }

    public FieldBounds(Translation2d from, Translation2d to) {
        this(
            Math.min(from.getX(), to.getX()),
            Math.max(from.getX(), to.getX()),
            Math.min(from.getY(), to.getY()),
            Math.max(from.getY(), to.getY())
        );
    }

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

    private FieldBounds canonicalize() {
        double newMinX = Math.min(minX(), maxX());
        double newMaxX = Math.max(minX(), maxX());
        double newMinY = Math.min(minY(), maxY());
        double newMaxY = Math.max(minY(), maxY());
        return new FieldBounds(newMinX, newMaxX, newMinY, newMaxY);
    }
    public FieldBounds flipped() {
        return new FieldBounds(
            AllianceFlipUtil.flipX(minX()),
            AllianceFlipUtil.flipX(maxX()),
            AllianceFlipUtil.flipY(minY()),
            AllianceFlipUtil.flipY(maxY())
        ).canonicalize();
    }
}