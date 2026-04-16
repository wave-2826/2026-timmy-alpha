package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

public class AllianceFlipUtil {
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.fieldLengthX - x : x;
    }

    public static double flipX(double x) {
        return FieldConstants.fieldLengthX - x;
    }
    
    public static double applyY(double y) {
        return shouldFlip() ? FieldConstants.fieldWidthY - y : y;
    }

    public static double flipY(double y) {
        return FieldConstants.fieldWidthY - y;
    }
    
    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
    }
    
    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
    }
    
    public static Pose2d apply(Pose2d pose) {
        return shouldFlip()
            ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
            : pose;
    }
    
    public static Translation3d apply(Translation3d translation) {
        return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
    }
    
    public static Rotation3d apply(Rotation3d rotation) {
        return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
    }
    
    public static Pose3d apply(Pose3d pose) {
        return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }
    
    public static FieldBounds apply(FieldBounds bounds) {
        if(shouldFlip()) {
            return new FieldBounds(
                applyX(bounds.maxX()),
                applyX(bounds.minX()),
                applyY(bounds.maxY()),
                applyY(bounds.minY())
            );
        } else {
            return bounds;
        }
    }
}