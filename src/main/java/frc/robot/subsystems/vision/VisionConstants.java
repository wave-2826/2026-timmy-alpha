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
        new Transform3d(new Translation3d(0.19256967501449876, 0.3712312236275484, 0.44682618493003523),   new Rotation3d(0.0176787805305168, -0.33323618663786675, 1.282041104538141))
    );
    public static CameraConfiguration cameraFrontLeft = new CameraConfiguration(
        "2826_OV9281_Fin",
        new Transform3d(new Translation3d(0.22474450640968285, 0.3049297701228512, 0.44524781934992275),   new Rotation3d(0.01634319241023896, -0.32089240568016286, 0.22462314214613913))
    );
    public static CameraConfiguration cameraFrontRight = new CameraConfiguration(
        "2826_OV9281_Abe",
        new Transform3d(new Translation3d(0.20128437247693873, -0.3801166684770375, 0.4402597122016706),   new Rotation3d(0.023205301792742668, -0.30938538337397925, -0.304323942193334))
    );
    public static CameraConfiguration cameraRightmost = new CameraConfiguration(
        "2826_OV9281_Gem",
        new Transform3d(new Translation3d(0.12435679572761617, -0.38808136052967684, 0.43761855661407406), new Rotation3d(0.01568510376516497, -0.32001531584503656, -1.3666162015603929))
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
