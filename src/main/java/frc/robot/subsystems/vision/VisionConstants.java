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

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = FieldConstants.defaultAprilTagType.getLayout();

    private static record CameraConfiguration(
        String name,
        Transform3d position
    ) {}

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "2826_OV9281_Ena";
    public static String camera1Name = "2826_OV9281_Fin";

    // Robot to camera transforms
    public static Transform3d robotToCamera0 = new Transform3d(new Translation3d(0.26299228917216877, 0.37463897857529405, 0.46437706895675346), new Rotation3d(0.019078617921454894, -0.32843832829994896, 0.22190482336538536));
    public static Transform3d robotToCamera1 = new Transform3d(new Translation3d(0.19297534359942536, 0.22660469052086318, 0.4836127596115337), new Rotation3d(0.015612826506477587, -0.30580379815927905, 1.3097503000462243));

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
