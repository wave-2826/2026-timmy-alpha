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
    public static final boolean enableVisionSimulation = true;

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
        // TODO: Recalibrate
        new Transform3d(new Translation3d(0.15582344984686222, 0.4341147342006587, 0.4534495927084479), new Rotation3d(0.02516561369000825, -0.4091266208993857, 1.289064962108927))
        // new Transform3d(new Translation3d(0.26299228917216877, 0.37463897857529405, 0.46437706895675346), new Rotation3d(0.019078617921454894, -0.32843832829994896, 0.22190482336538536))
    );
    public static CameraConfiguration cameraFrontLeft = new CameraConfiguration(
        "2826_OV9281_Fin",
        // TODO: Recalibrate
        new Transform3d(new Translation3d(0.2686987017716467, 0.38871703824933784, 0.4624331960863525), new Rotation3d(-0.0019470998534024731, -0.4229848498952987, 0.20816958409573905))
        // new Transform3d(new Translation3d(0.19297534359942536, 0.43660469052086318, 0.4836127596115337), new Rotation3d(0.015612826506477587, -0.30580379815927905, 1.3097503000462243))
    );
    public static CameraConfiguration cameraFrontRight = new CameraConfiguration(
        "2826_OV9281_Abe",
        new Transform3d(new Translation3d(0.2686987017716467, -0.38871703824933784, 0.4624331960863525), new Rotation3d(-0.0019470998534024731, -0.4229848498952987, -0.20816958409573905))
    );
    public static CameraConfiguration cameraRightmost = new CameraConfiguration(
        "2826_OV9281_Gem",
        new Transform3d(new Translation3d(0.15582344984686222, -0.4341147342006587, 0.4534495927084479), new Rotation3d(0.02516561369000825, -0.4091266208993857, -1.289064962108927))
    );

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;
    /** The maximum error in an estimate's rotation in degrees. */
    public static final double maxRotationError = 20;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };
}
