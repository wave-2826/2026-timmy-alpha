package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;

import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.Supplier;

/**
 * Simplified swerve drive simulation - based on that of MapleSim - without dyn4j.
 */
public class SwerveDriveSimulation {
    public static class COTS {
        public static Supplier<GyroSimulation> pigeon2() {
            return () -> new GyroSimulation(0.5, 0.02);
        }
    }

    public static class DriveTrainSimulationConfig {
        public Mass robotMass;
        public Distance bumperLengthX, bumperWidthY;
        public Supplier<GyroSimulation> gyroSimulationFactory;
        public Translation2d[] moduleTranslations;

        /**
         * <h2>Ordinary Constructor</h2>
         *
         * <p>Creates an instance of {@link DriveTrainSimulationConfig} with specified parameters.
         *
         * @param robotMass the mass of the robot, including bumpers.
         * @param bumperLengthX the length of the bumper (distance from front to back).
         * @param bumperWidthY the width of the bumper (distance from left to right).
         * @param trackLengthX the distance between the front and rear wheels.
         * @param trackWidthY the distance between the left and right wheels.
         * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the drivetrain.
         */
        public DriveTrainSimulationConfig(
            Mass robotMass,
            Distance bumperLengthX,
            Distance bumperWidthY,
            Distance trackLengthX,
            Distance trackWidthY,
            Supplier<GyroSimulation> gyroSimulationFactory
        ) {
            this.robotMass = robotMass;
            this.bumperLengthX = bumperLengthX;
            this.bumperWidthY = bumperWidthY;
            this.withTrackLengthTrackWidth(trackLengthX, trackWidthY);
            this.gyroSimulationFactory = gyroSimulationFactory;
        }

        public static DriveTrainSimulationConfig Default() {
            return new DriveTrainSimulationConfig(
                Kilograms.of(45),
                Meters.of(0.76),
                Meters.of(.76),
                Meters.of(0.52),
                Meters.of(0.52),
                COTS.pigeon2());
        }

        public DriveTrainSimulationConfig withRobotMass(Mass robotMass) {
            this.robotMass = robotMass;
            return this;
        }
        
        public DriveTrainSimulationConfig withBumperSize(Distance bumperLengthX, Distance bumperWidthY) {
            // TODO: don't be half an inch off
            this.bumperLengthX = bumperLengthX;
            this.bumperWidthY = bumperWidthY;

            return this;
        }

        public DriveTrainSimulationConfig withTrackLengthTrackWidth(Distance trackLengthX, Distance trackWidthY) {
            this.moduleTranslations = new Translation2d[] {
                new Translation2d(trackLengthX.in(Meters) / 2, trackWidthY.in(Meters) / 2),
                new Translation2d(trackLengthX.in(Meters) / 2, -trackWidthY.in(Meters) / 2),
                new Translation2d(-trackLengthX.in(Meters) / 2, trackWidthY.in(Meters) / 2),
                new Translation2d(-trackLengthX.in(Meters) / 2, -trackWidthY.in(Meters) / 2)
            };
            return this;
        }

        public DriveTrainSimulationConfig withCustomModuleTranslations(Translation2d[] moduleTranslations) {
            this.moduleTranslations = moduleTranslations;
            return this;
        }

        public DriveTrainSimulationConfig withGyro(Supplier<GyroSimulation> gyroSimulationFactory) {
            this.gyroSimulationFactory = gyroSimulationFactory;
            return this;
        }

        public double getDensityKgPerSquaredMeters() {
            return robotMass.in(Kilograms) / (bumperLengthX.in(Meters) * bumperWidthY.in(Meters));
        }

        public Distance trackLengthX() {
            final OptionalDouble maxModuleX = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getX).max();
            final OptionalDouble minModuleX = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getX).min();
            if(maxModuleX.isEmpty() || minModuleX.isEmpty()) throw new IllegalStateException("Modules translations are empty");
            return Meters.of(maxModuleX.getAsDouble() - minModuleX.getAsDouble());
        }

        public Distance trackWidthY() {
            final OptionalDouble maxModuleY = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getY)
                .max();
            final OptionalDouble minModuleY = Arrays.stream(moduleTranslations)
                .mapToDouble(Translation2d::getY)
                .min();
            if(maxModuleY.isEmpty() || minModuleY.isEmpty()) throw new IllegalStateException("Modules translations are empty");
            return Meters.of(maxModuleY.getAsDouble() - minModuleY.getAsDouble());
        }

        public Distance driveBaseRadius() {
            return Meters.of(Math.hypot(trackLengthX().in(Meters), trackWidthY().in(Meters)));
        }
    }

    protected final GyroSimulation gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;
    public final DriveTrainSimulationConfig config;

    // Simple physics state
    private Pose2d pose;
    private Translation2d velocity = new Translation2d();
    private double angularVelocity = 0.0;

    protected Supplier<SwerveModuleState[]> getModuleStates = null;

    public SwerveDriveSimulation(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        this.config = config;
        
        this.pose = initialPoseOnField;
        this.gyroSimulation = config.gyroSimulationFactory.get();
        this.gyroSimulation.setRotation(initialPoseOnField.getRotation());

        this.moduleTranslations = config.moduleTranslations;
        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    public void setModuleStateSupplier(Supplier<SwerveModuleState[]> getModuleStates) {
        this.getModuleStates = getModuleStates;
    }

    /**
     * Call this every simulation tick with the simulation timestep
     */
    public void update(double dtSeconds) {
        // Get robot-relative speeds from module states
        ChassisSpeeds robotRel = getDriveTrainSimulatedChassisSpeedsRobotRelative();
        setRobotSpeeds(robotRel);
        pose = pose.exp(robotRel.toTwist2d(dtSeconds));

        // Wall collision based on corners in field coordinates
        double halfLength = config.bumperLengthX.in(Meters) / 2.0;
        double halfWidth = config.bumperWidthY.in(Meters) / 2.0;
        Rotation2d rot = pose.getRotation();
        Translation2d center = pose.getTranslation();

        // Find min/max x/y among corners
        Translation2d[] corners = new Translation2d[] {
            new Translation2d(+halfLength, +halfWidth).rotateBy(rot).plus(center),
            new Translation2d(+halfLength, -halfWidth).rotateBy(rot).plus(center),
            new Translation2d(-halfLength, +halfWidth).rotateBy(rot).plus(center),
            new Translation2d(-halfLength, -halfWidth).rotateBy(rot).plus(center)
        };
        double minX = Arrays.stream(corners).mapToDouble(Translation2d::getX).min().orElse(center.getX());
        double maxX = Arrays.stream(corners).mapToDouble(Translation2d::getX).max().orElse(center.getX());
        double minY = Arrays.stream(corners).mapToDouble(Translation2d::getY).min().orElse(center.getY());
        double maxY = Arrays.stream(corners).mapToDouble(Translation2d::getY).max().orElse(center.getY());

        double dx = 0, dy = 0;
        boolean hitWall = false;

        if(minX < 0.0) {
            dx = -minX;
            hitWall = true;
        } else if(maxX > FieldConstants.fieldLengthX) {
            dx = FieldConstants.fieldLengthX - maxX;
            hitWall = true;
        }
        if(minY < 0.0) {
            dy = -minY;
            hitWall = true;
        } else if(maxY > FieldConstants.fieldWidthY) {
            dy = FieldConstants.fieldWidthY - maxY;
            hitWall = true;
        }

        if(hitWall) {
            // Move robot back inside field
            Translation2d newCenter = center.plus(new Translation2d(dx, dy));
            pose = new Pose2d(newCenter, pose.getRotation());

            // Zero velocity in direction(s) of collision
            if(dx != 0) velocity = new Translation2d(0, velocity.getY());
            if(dy != 0) velocity = new Translation2d(velocity.getX(), 0);
        }

        // Update gyro
        gyroSimulation.updateSimulationSubTick(angularVelocity);
    }

    // TODO: Make odom work this is just a band aid solution
    public void updateOdom(Drive drive) {
        drive.setPose(pose);
    }

    public void setSimulationWorldPose(Pose2d robotPose) {
        this.pose = robotPose;
        this.velocity = new Translation2d();
        this.angularVelocity = 0.0;
        // Sync the gyro to the new orientation so field-relative conversions
        // in the rest of the codebase stay consistent
        gyroSimulation.setRotation(robotPose.getRotation());
    }

    private void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        this.velocity = new Translation2d(givenSpeeds.vxMetersPerSecond, givenSpeeds.vyMetersPerSecond);
        this.angularVelocity = givenSpeeds.omegaRadiansPerSecond;
    }

    public Pose2d getSimulatedDriveTrainPose() {
        return pose;
    }

    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
        return kinematics.toChassisSpeeds(getModuleStates.get());
    }

    public GyroSimulation getGyroSimulation() {
        return this.gyroSimulation;
    }

    public Distance driveBaseRadius() {
        return config.driveBaseRadius();
    }
}
