package frc.robot.commands.tuning;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.tunables.LoggedTunableNumber;

public class VisionTuningCommands {
    private static class TransformAverage {
        private double xSum = 0;
        private double ySum = 0;
        private double zSum = 0;
        private double pitchSum = 0;
        private double rollSum = 0;
        private double yawSum = 0;
        private int datapoints = 0;

        public TransformAverage() {
        }

        public void add(Transform3d transform) {
            xSum += transform.getX();
            ySum += transform.getY();
            zSum += transform.getZ();
            pitchSum += transform.getRotation().getY();
            rollSum += transform.getRotation().getX();
            yawSum += transform.getRotation().getZ();
            datapoints += 1;
        }

        public Transform3d getAverage() {
            if(datapoints == 0) return new Transform3d();
            return new Transform3d(new Translation3d(xSum / datapoints, ySum / datapoints, zSum / datapoints),
                new Rotation3d(rollSum / datapoints, pitchSum / datapoints, yawSum / datapoints));
        }
    }

    /** Adds the drive tuning commands to the auto chooser. */
    public static void addTuningCommandsToChooser(Vision vision, LoggedDashboardChooser<Command> chooser) {
        // We may want to run this at a competition
        chooser.addOption("TUNING | Vision Camera Position Measurement", measureCameraPositions(vision));
    }

    // Distance out from PDH side
    private static LoggedTunableNumber heldTagXIn = new LoggedTunableNumber("Vision/CalibrationTag/XInches", 0.0);
    // Distance left toward rio side
    private static LoggedTunableNumber heldTagYIn = new LoggedTunableNumber("Vision/CalibrationTag/YInches", 60.75);
    // Distance up
    private static LoggedTunableNumber heldTagZIn = new LoggedTunableNumber("Vision/CalibrationTag/ZInches", 23.0 + 6.5 / 2.);
    // Rotation clockwise from forward (facing PDH)
    private static LoggedTunableNumber heldTagYawDeg = new LoggedTunableNumber("Vision/CalibrationTag/YawDegrees", 270.0);

    /** The transform of the calibration tag, relative to the robot base. */
    private static Transform3d getHeldTagTransform() {
        return new Transform3d(new Translation3d(
            Units.inchesToMeters(heldTagXIn.get()),
            Units.inchesToMeters(heldTagYIn.get()),
            Units.inchesToMeters(heldTagZIn.get())
        ), new Rotation3d(0., 0., Units.degreesToRadians(heldTagYawDeg.get())));
    }

    public static Command measureCameraPositions(Vision vision) {
        TransformAverage[] averages = new TransformAverage[vision.getCameraCount()];
        return Commands.startRun(() -> {
            for(int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                averages[cameraIndex] = new TransformAverage();
            }
            System.out.println("********** Vision camera position measurement started. **********");
        }, () -> {
            Transform3d[] transforms = vision.getBestTagTransforms();
            System.out.print("Cameras seeing tags: [");
            int seen = 0;
            for(int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                var transform = transforms[cameraIndex];
                if(transform != null) {
                    averages[cameraIndex].add(transform);
                    seen++;
                    if(seen > 1) System.out.print(", ");
                    System.out.print(vision.getCameraNames()[cameraIndex]);
                }
            }
            System.out.println("]");
        }, vision).finallyDo(() -> {
            System.out.flush();
            System.out.println("********** Vision camera position measurement results **********");
            Transform3d[] adjustedTransforms = new Transform3d[4];
            for(int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                Transform3d averageTransform = averages[cameraIndex].getAverage();
                Transform3d transform = getHeldTagTransform().plus(averageTransform.inverse());
                adjustedTransforms[cameraIndex] = transform;

                System.out.print("Robot to camera " + cameraIndex + " (" + vision.getCameraNames()[cameraIndex] + "): ");
                printTransform(transform);
            }
            System.out.flush();
        });
    }

    private static void printTransform(Transform3d transform) {
        // new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw))
        Rotation3d rotation = transform.getRotation();
        System.out.println(
            "`new Transform3d(new Translation3d(" + transform.getX() + ", " + transform.getY() + ", " + transform.getZ()
                + "), new Rotation3d(" + rotation.getX() + ", " + rotation.getY() + ", " + rotation.getZ() + "))`");
    }
}