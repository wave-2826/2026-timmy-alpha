package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.util.simUtils.SwerveModuleSimulation.SwerveModuleSimulationConfig;

import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.Supplier;

/**
 * Simplified SwerveDriveSimulation without dyn4j.
 */
public class SwerveDriveSimulation {
    public static class COTS {
        public static SwerveModuleSimulationConfig ofMark4(
            DCMotor driveMotor, DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
            return new SwerveModuleSimulationConfig(
                driveMotor,
                steerMotor,
                switch (gearRatioLevel) {
                    case 1 -> 8.14;
                    case 2 -> 6.75;
                    case 3 -> 6.12;
                    case 4 -> 5.14;
                    default -> throw new IllegalStateException("Unknown gearing level: " + gearRatioLevel);
                },
                12.8,
                Volts.of(0.1),
                Volts.of(0.2),
                Inches.of(2),
                KilogramSquareMeters.of(0.03),
                wheelCOF);
        }

        public static Supplier<GyroSimulation> ofPigeon2() {
            return () -> new GyroSimulation(0.5, 0.02);
        }

    }

    public static class DriveTrainSimulationConfig {
        public Mass robotMass;
        public Distance bumperLengthX, bumperWidthY;
        public Supplier<SwerveModuleSimulation>[] swerveModuleSimulationFactories;
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
         * @param swerveModuleSimulationFactory the factory that creates appropriate swerve module simulation for the
         *     drivetrain. You can specify one factory to apply the same configuration over all modules or specify four
         *     factories in the order (FL, FR, BL, BR).
         * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the drivetrain.
         */
        public DriveTrainSimulationConfig(
                Mass robotMass,
                Distance bumperLengthX,
                Distance bumperWidthY,
                Distance trackLengthX,
                Distance trackWidthY,
                Supplier<GyroSimulation> gyroSimulationFactory,
                @SuppressWarnings("unchecked")
                Supplier<SwerveModuleSimulation>... swerveModuleSimulationFactory
        ) {
            this.robotMass = robotMass;
            this.bumperLengthX = bumperLengthX;
            this.bumperWidthY = bumperWidthY;
            this.withTrackLengthTrackWidth(trackLengthX, trackWidthY);

            if(swerveModuleSimulationFactory.length == 1) this.withSwerveModule(swerveModuleSimulationFactory[0]);
            else if(swerveModuleSimulationFactory.length == 4) this.withSwerveModules(swerveModuleSimulationFactory);
            else throw new IllegalArgumentException("Module simulation factories length must be 1 or 4, provided " + swerveModuleSimulationFactory.length);
            this.gyroSimulationFactory = gyroSimulationFactory;
        }

        @SuppressWarnings("unchecked")
        public static DriveTrainSimulationConfig Default() {
            return new DriveTrainSimulationConfig(
                Kilograms.of(45),
                Meters.of(0.76),
                Meters.of(.76),
                Meters.of(0.52),
                Meters.of(0.52),
                COTS.ofPigeon2(),
                COTS.ofMark4(DCMotor.getFalcon500(1), DCMotor.getFalcon500(1), 1.9, 2));
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

        public DriveTrainSimulationConfig withSwerveModules(
            @SuppressWarnings("unchecked")
            Supplier<SwerveModuleSimulation>... swerveModuleSimulationFactory
        ) {
            if(swerveModuleSimulationFactory.length == 1) return withSwerveModule(swerveModuleSimulationFactory[0]);

            if(swerveModuleSimulationFactory.length != moduleTranslations.length) throw new IllegalArgumentException("Module simulation factories length must be 1 or 4, provided " + swerveModuleSimulationFactory.length);

            this.swerveModuleSimulationFactories = swerveModuleSimulationFactory;
            return this;
        }

        @SuppressWarnings("unchecked")
        public DriveTrainSimulationConfig withSwerveModule(Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory) {
            this.swerveModuleSimulationFactories = new Supplier[moduleTranslations.length];
            Arrays.fill(this.swerveModuleSimulationFactories, swerveModuleSimulationFactory);
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
            if (maxModuleY.isEmpty() || minModuleY.isEmpty())
                throw new IllegalStateException("Modules translations are empty");
            return Meters.of(maxModuleY.getAsDouble() - minModuleY.getAsDouble());
        }

        public Distance driveBaseRadius() {
            return Meters.of(Math.hypot(trackLengthX().in(Meters), trackWidthY().in(Meters)));
        }
    }

    private final SwerveModuleSimulation[] moduleSimulations;
    protected final GyroSimulation gyroSimulation;
    protected final Translation2d[] moduleTranslations;
    protected final SwerveDriveKinematics kinematics;
    private final double gravityForceOnEachModule;
    public final DriveTrainSimulationConfig config;

    // Simple physics state
    private Pose2d pose;
    private Translation2d velocity = new Translation2d();
    private double angularVelocity = 0.0;

    // Field boundaries (meters)
    private static final double FIELD_MIN_X = 0.0, FIELD_MAX_X = 16.54;
    private static final double FIELD_MIN_Y = 0.0, FIELD_MAX_Y = 8.02;

    public SwerveDriveSimulation(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        this.config = config;
        this.pose = initialPoseOnField;
        this.moduleTranslations = config.moduleTranslations;
        this.moduleSimulations = Arrays.stream(config.swerveModuleSimulationFactories)
                .map(Supplier::get)
                .toArray(SwerveModuleSimulation[]::new);
        this.gyroSimulation = config.gyroSimulationFactory.get();
        this.kinematics = new SwerveDriveKinematics(moduleTranslations);
        this.gravityForceOnEachModule = config.robotMass.in(Kilograms) * 9.8 / moduleSimulations.length;
    }

    /**
     * Call this every simulation tick with the simulation timestep (seconds).
     */
    public void update(double dtSeconds) {
        // 1. Update modules and sum forces
        Translation2d totalForce = new Translation2d();
        double totalTorque = 0.0;

        for (int i = 0; i < moduleSimulations.length; i++) {
            SwerveModuleSimulation module = moduleSimulations[i];
            Translation2d moduleVel = velocity.plus(
                new Translation2d(-angularVelocity * (moduleTranslations[i].getY()), angularVelocity * (moduleTranslations[i].getX()))
            );
            Translation2d moduleForce = module.updateSimulationSubTickGetModuleForce(
                moduleVel, pose.getRotation(), gravityForceOnEachModule);

            totalForce = new Translation2d(
                totalForce.getX() + moduleForce.getX(),
                totalForce.getY() + moduleForce.getY()
            );

            // Torque = r x F (2D cross product)
            double dx = moduleTranslations[i].getX();
            double dy = moduleTranslations[i].getY();
            totalTorque += dx * moduleForce.getY() - dy * moduleForce.getX();
        }

        // 2. Simple friction (linear and angular damping)
        double linearDamping = 1.4;
        double angularDamping = 1.4;
        Translation2d friction = new Translation2d(-velocity.getX() * linearDamping, -velocity.getY() * linearDamping);
        double frictionTorque = -angularVelocity * angularDamping;

        totalForce = new Translation2d(
            totalForce.getX() + friction.getX(),
            totalForce.getY() + friction.getY()
        );
        totalTorque += frictionTorque;

        // 3. Update velocities (F = m*a)
        double mass = config.robotMass.in(Kilograms);
        velocity = new Translation2d(
            velocity.getX() + (totalForce.getX() / mass) * dtSeconds,
            velocity.getY() + (totalForce.getY() / mass) * dtSeconds
        );

        // 4. Update angular velocity (T = I*alpha)
        double inertia = mass * Math.pow(config.driveBaseRadius().in(Meters), 2);
        angularVelocity += (totalTorque / inertia) * dtSeconds;

        // 5. Update pose
        pose = new Pose2d(
            pose.getTranslation().getX() + velocity.getX() * dtSeconds,
            pose.getTranslation().getY() + velocity.getY() * dtSeconds,
            pose.getRotation().plus(Rotation2d.fromRadians(angularVelocity * dtSeconds))
        );

        // 6. Wall collision: clamp position and zero velocity if hit
        // Compute robot corners in field coordinates
        double halfLength = config.bumperLengthX.in(Meters) / 2.0;
        double halfWidth = config.bumperWidthY.in(Meters) / 2.0;
        Rotation2d rot = pose.getRotation();
        Translation2d center = pose.getTranslation();

        // Robot corners relative to center
        Translation2d[] corners = new Translation2d[] {
            new Translation2d(+halfLength, +halfWidth).rotateBy(rot).plus(center),
            new Translation2d(+halfLength, -halfWidth).rotateBy(rot).plus(center),
            new Translation2d(-halfLength, +halfWidth).rotateBy(rot).plus(center),
            new Translation2d(-halfLength, -halfWidth).rotateBy(rot).plus(center)
        };

        // Find min/max x/y among corners
        double minX = Arrays.stream(corners).mapToDouble(Translation2d::getX).min().orElse(center.getX());
        double maxX = Arrays.stream(corners).mapToDouble(Translation2d::getX).max().orElse(center.getX());
        double minY = Arrays.stream(corners).mapToDouble(Translation2d::getY).min().orElse(center.getY());
        double maxY = Arrays.stream(corners).mapToDouble(Translation2d::getY).max().orElse(center.getY());

        double dx = 0, dy = 0;
        boolean hitWall = false;

        if (minX < FIELD_MIN_X) {
            dx = FIELD_MIN_X - minX;
            hitWall = true;
        } else if (maxX > FIELD_MAX_X) {
            dx = FIELD_MAX_X - maxX;
            hitWall = true;
        }
        if (minY < FIELD_MIN_Y) {
            dy = FIELD_MIN_Y - minY;
            hitWall = true;
        } else if (maxY > FIELD_MAX_Y) {
            dy = FIELD_MAX_Y - maxY;
            hitWall = true;
        }

        if (hitWall) {
            // Move robot back inside field
            Translation2d newCenter = center.plus(new Translation2d(dx, dy));
            pose = new Pose2d(newCenter, pose.getRotation());

            // Zero velocity in direction(s) of collision
            if (dx != 0) velocity = new Translation2d(0, velocity.getY());
            if (dy != 0) velocity = new Translation2d(velocity.getX(), 0);
        }

        // 7. Update gyro
        gyroSimulation.updateSimulationSubTick(angularVelocity);
    }

    public void setSimulationWorldPose(Pose2d robotPose) {
        this.pose = robotPose;
        this.velocity = new Translation2d();
        this.angularVelocity = 0.0;
    }

    public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        this.velocity = new Translation2d(givenSpeeds.vxMetersPerSecond, givenSpeeds.vyMetersPerSecond);
        this.angularVelocity = givenSpeeds.omegaRadiansPerSecond;
    }

    public Pose2d getSimulatedDriveTrainPose() {
        return pose;
    }

    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), angularVelocity, pose.getRotation()
        );
    }

    public SwerveModuleSimulation[] getModules() {
        return moduleSimulations;
    }

    public GyroSimulation getGyroSimulation() {
        return this.gyroSimulation;
    }

    // The rest of your methods (maxLinearVelocity, etc.) can remain unchanged.
    public LinearVelocity maxLinearVelocity() {
        return moduleSimulations[0].config.maximumGroundSpeed();
    }

    public LinearAcceleration maxLinearAcceleration(Current statorCurrentLimit) {
        return moduleSimulations[0].config.maxAcceleration(config.robotMass, moduleSimulations.length, statorCurrentLimit);
    }

    public Distance driveBaseRadius() {
        return config.driveBaseRadius();
    }

    public AngularVelocity maxAngularVelocity() {
        return RadiansPerSecond.of(maxLinearVelocity().in(MetersPerSecond) / config.driveBaseRadius().in(Meters));
    }

    public AngularAcceleration maxAngularAcceleration(Current statorCurrentLimit) {
        return RadiansPerSecondPerSecond.of(moduleSimulations[0]
            .config
            .getTheoreticalPropellingForcePerModule(
                config.robotMass, moduleSimulations.length, statorCurrentLimit)
            .in(Newtons)
            * moduleTranslations[0].getNorm()
            * moduleSimulations.length
            / (config.robotMass.in(Kilograms) * Math.pow(config.driveBaseRadius().in(Meters), 2)));
    }
}
