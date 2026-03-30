package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;

public class VisionConstants {
    /****** Simulation ******/
    /**
     * If we should enable vision simulation. Turning off vision sim can dramatically improve loop times, but it's
     * obviously far less representative of real robot odometry.
     */
    public static final boolean enableVisionSimulation = false;

    /**
     * Enable drawing a wireframe visualization of the field to the camera streams in simulation mode. This is extremely
     * resource-intensive!
     */
    public static final boolean enableWireframeDrawing = true;

    /**
     * Enable raw streams for simulated cameras. This can increase loop times slightly.
     */
    public static final boolean enableRawStreams = true;
    /************************/

    public static AprilTagFieldLayout aprilTagLayout = FieldConstants.defaultAprilTagType.getLayout();

    public static record CameraConfiguration(
        String name,
        /** Nullable (if not calibrated; turns off camera) */
        Transform3d position
    ) {}

    public static CameraConfiguration cameraLeftmost = new CameraConfiguration(
        "2826_OV9281_Ena",
        new Transform3d(new Translation3d(0.18868012181167004, 0.37056625122226144, 0.46341750211881333), new Rotation3d(0.030098053016044974, -0.325774366531199, 1.2441301090023986))
    );
    public static CameraConfiguration cameraFrontLeft = new CameraConfiguration(
        "2826_OV9281_Fin",
        new Transform3d(new Translation3d(0.22778657178131428, 0.3163912857925228, 0.4614040424243781), new Rotation3d(0.039016142820737766, -0.31813074444347167, 0.18745931081144462))
    );
    public static CameraConfiguration cameraFrontRight = new CameraConfiguration(
        "2826_OV9281_Abe",
        null
    );
    public static CameraConfiguration cameraRightmost = new CameraConfiguration(
        "2826_OV9281_Gem",
        new Transform3d(new Translation3d(0.1519442869732049, -0.40257497173741424, 0.4614851538303224), new Rotation3d(0.033264102269778766, -0.3840198165501535, -1.3709157027823682))
    );

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;
    /** The maximum error in an estimate's rotation in degrees. */
    public static final double maxRotationError = 20;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.07; // Meters
    public static double angularStdDevBaseline = 1000.0; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };
}
