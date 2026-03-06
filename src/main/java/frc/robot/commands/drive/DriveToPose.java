package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.tunables.LoggedTunableNumber;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Heavily inspired (copied) from [6328 Mechanical Advantage's
 * implementation](https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/commands/DriveToPose.java).
 */
public class DriveToPose extends Command {
    // Drive and turn PID gains
    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber(//
        "DriveToPose/DrivekP", 2.75);
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber(//
        "DriveToPose/DrivekD", 0.0);
    private static final LoggedTunableNumber thetakP = new LoggedTunableNumber(//
        "DriveToPose/ThetakP", 2.5);
    private static final LoggedTunableNumber thetakD = new LoggedTunableNumber(//
        "DriveToPose/ThetakD", 0.0);

    // Tolerances for drive and theta
    private static final LoggedTunableNumber driveTolerance = new LoggedTunableNumber(//
        "DriveToPose/DriveTolerance", 0.4);
    private static final LoggedTunableNumber thetaTolerance = new LoggedTunableNumber(//
        "DriveToPose/ThetaTolerance", 1);

    // Drive and turn constraints when the elevator is at the bottom
    private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber(//
        "DriveToPose/DriveMaxVelocity", DriveConstants.maxSpeedMetersPerSec);
    private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber(//
        "DriveToPose/DriveMaxAcceleration", 5);
    private static final LoggedTunableNumber thetaMaxVelocity = new LoggedTunableNumber(//
        "DriveToPose/ThetaMaxVelocity", Units.degreesToRadians(720.0));
    private static final LoggedTunableNumber thetaMaxAcceleration = new LoggedTunableNumber(//
        "DriveToPose/ThetaMaxAcceleration", Units.degreesToRadians(500));

    // Drive and turn constraints when the elevator is at the top
    private static final LoggedTunableNumber driveMaxVelocityTop = new LoggedTunableNumber(//
        "DriveToPose/DriveMaxVelocityTop", DriveConstants.maxSpeedMetersPerSec);
    private static final LoggedTunableNumber driveMaxAccelerationTop = new LoggedTunableNumber(//
        "DriveToPose/DriveMaxAccelerationTop", 2.5);
    private static final LoggedTunableNumber thetaMaxVelocityTop = new LoggedTunableNumber(//
        "DriveToPose/ThetaMaxVelocityTop", Units.degreesToRadians(300.0));
    private static final LoggedTunableNumber thetaMaxAccelerationTop = new LoggedTunableNumber(//
        "DriveToPose/ThetaMaxAccelerationTop", Units.degreesToRadians(400));

    // The minimum height (as a percentage) of the elevator before we start interpolating towards the top constraints
    private static final LoggedTunableNumber elevatorMinExtension = new LoggedTunableNumber(//
        "DriveToPose/ElevatorMinExtension", 0.4);

    private static final LoggedTunableNumber setpointMinVelocity = new LoggedTunableNumber(//
        "DriveToPose/SetpointMinVelocity", -0.5); // The most negative velocity we will command after velocity correction.
    private static final LoggedTunableNumber minDistanceVelocityCorrection = new LoggedTunableNumber(//
        "DriveToPose/MinDistanceVelocityCorrection", Units.inchesToMeters(0.4)); // When below this distance, we stop correcting the velocity based on the target direction delta.

    // The range that we start to interpolate from feedforward setpoints using our trapezoidal profile
    private static final LoggedTunableNumber linearFFMinRadius = new LoggedTunableNumber(//
        "DriveToPose/LinearFFMinRadius", 0.01);
    private static final LoggedTunableNumber linearFFMaxRadius = new LoggedTunableNumber(//
        "DriveToPose/LinearFFMaxRadius", 0.05);
    private static final LoggedTunableNumber thetaFFMinError = new LoggedTunableNumber(//
        "DriveToPose/ThetaFFMinError", 0.0);
    private static final LoggedTunableNumber thetaFFMaxError = new LoggedTunableNumber(//
        "DriveToPose/ThetaFFMaxError", 0.0);

    private final Drive drive;

    private final Supplier<Pose2d> target;
    private Supplier<Pose2d> pose = RobotState.getInstance()::getPose;

    private TrapezoidProfile driveProfile;
    private final PIDController driveController = new PIDController(0.0, 0.0, 0.0);
    protected final ProfiledPIDController thetaController = new ProfiledPIDController(0.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(0.0, 0.0));

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private Translation2d lastSetpointVelocity = Translation2d.kZero;
    private Rotation2d lastGoalRotation = Rotation2d.kZero;

    private double absoluteTranslationError = 0.0;
    private double absoluteThetaError = 0.0;

    private boolean running = false;

    private final boolean endWhenAtTarget;
    private final OptionalDouble endLinearTolerance;
    private final OptionalDouble endThetaTolerance;

    private Supplier<Translation2d> linearDriverFeedforward = () -> Translation2d.kZero;
    private DoubleSupplier thetaDriverFeedforward = () -> 0.0;

    /**
     * A command that drives to the given pose and ends once it gets there. Defaults to a tolerance of 5 inches and 5
     * degrees.
     */
    public DriveToPose(Drive drive, Pose2d target) {
        this(drive, () -> target, true);
    }

    /**
     * A command that drives to the given pose and ends once it gets there if endWhenAtTarget is true. Defaults to a
     * tolerance of 5 inches and 5 degrees.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> getTarget, boolean endWhenAtTarget) {
        this(drive, getTarget, endWhenAtTarget, OptionalDouble.of(Units.inchesToMeters(5.)),
            OptionalDouble.of(Units.degreesToRadians(5.)));
    }

    /**
     * A command that drives to the given pose and ends once it gets there. Tolerances are set in meters and radians.
     */
    public DriveToPose(Drive drive, Pose2d target, double endLinearTolerance, double endThetaTolerance) {
        this(drive, () -> target, true, OptionalDouble.of(endLinearTolerance), OptionalDouble.of(endThetaTolerance));
    }

    /**
     * A command that drives to the given pose and ends once it gets there. Tolerances are set in meters and radians. If
     * not present, they will use the default tunable values.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> getTarget, boolean endWhenAtTarget,
        OptionalDouble endLinearTolerance, OptionalDouble endThetaTolerance) {
        this.drive = drive;
        this.target = getTarget;

        this.endWhenAtTarget = endWhenAtTarget;
        this.endLinearTolerance = endLinearTolerance;
        this.endThetaTolerance = endThetaTolerance;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    /**
     * A command that drives to the given pose and never ends.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> getTarget) {
        this(drive, getTarget, false, OptionalDouble.empty(), OptionalDouble.empty());
    }

    /**
     * A command that drives to the given pose and never ends. Includes a feedforward for driver controls added to the
     * commanded velocity.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> getTarget, Supplier<Translation2d> linearDriverFeedforward,
        DoubleSupplier thetaDriverFeedforward) {
        this(drive, getTarget, false, OptionalDouble.empty(), OptionalDouble.empty());
        this.linearDriverFeedforward = linearDriverFeedforward;
        this.thetaDriverFeedforward = thetaDriverFeedforward;
    }

    /**
     * A command that drives to the given pose and never ends. This constructor is used when we want to use a different
     * pose supplier than the default vision-based odometry estimate.
     */
    public DriveToPose(Drive drive, Supplier<Pose2d> target, Supplier<Pose2d> getPose) {
        this(drive, target);
        this.pose = getPose;
    }

    @Override
    public void initialize() {
        resetProfile();
    }

    @Override
    public void execute() {
        running = true;

        // Update from tunable numbers
        if(driveTolerance.hasChanged(hashCode()) || thetaTolerance.hasChanged(hashCode())
            || drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode()) || thetakP.hasChanged(hashCode())
            || thetakD.hasChanged(hashCode())) {
            driveController.setP(drivekP.get());
            driveController.setD(drivekD.get());
            driveController.setTolerance(Units.inchesToMeters(driveTolerance.get()));
            thetaController.setP(thetakP.get());
            thetaController.setD(thetakD.get());
            thetaController.setTolerance(Units.degreesToRadians(thetaTolerance.get()));
        }

        updateConstraints();

        // Get current pose and target pose
        Pose2d currentPose = pose.get();
        Pose2d targetPose = target.get();

        Pose2d poseError = currentPose.relativeTo(targetPose);
        absoluteTranslationError = poseError.getTranslation().getNorm();
        absoluteThetaError = Math.abs(poseError.getRotation().getRadians());

        Translation2d driveVelocity = calculateDriveVelocity(currentPose, targetPose);
        double thetaVelocity = calculateThetaVelocity(currentPose, targetPose);

        // Incorporate driver feedforward -- half of full speed will take over control entirely.
        final double linearInterp = MathUtil.clamp(linearDriverFeedforward.get().getNorm() * 2.0, 0.0, 1.0);
        final double thetaInterp = MathUtil.clamp(Math.abs(thetaDriverFeedforward.getAsDouble()) * 2.0, 0.0, 1.0);

        driveVelocity = driveVelocity
            .interpolate(linearDriverFeedforward.get().times(DriveConstants.maxSpeedMetersPerSec), linearInterp);
        thetaVelocity = MathUtil.interpolate(thetaVelocity,
            thetaDriverFeedforward.getAsDouble() * DriveConstants.maxAngularSpeedRadPerSec, thetaInterp);

        // Reset profiles if there's enough driver input
        if(linearInterp >= 0.3 || thetaInterp >= 0.3) {
            resetProfile();
        }

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(//
            driveVelocity.getX(), driveVelocity.getY(), //
            thetaVelocity, //
            currentPose.getRotation() //
        ), true);

        // Log
        Logger.recordOutput("DriveToPose/CurrentPose", currentPose);

        Logger.recordOutput("DriveToPose/DistanceMeasured", absoluteTranslationError);

        Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
        Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);

        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {
            new Pose2d(lastSetpointTranslation, Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
        Logger.recordOutput("DriveToPose/TargetPose", new Pose2d[] {
            targetPose
        });
    }

    /**
     * Updates our profile constraints based on our possible acceleration.
     */
    private void updateConstraints() {
        driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            MathUtil.interpolate(driveMaxVelocity.get(), driveMaxVelocityTop.get(), 1),
            MathUtil.interpolate(driveMaxAcceleration.get(), driveMaxAccelerationTop.get(), 1)));

        thetaController.setConstraints(new TrapezoidProfile.Constraints(
            MathUtil.interpolate(thetaMaxVelocity.get(), thetaMaxVelocityTop.get(), 1),
            MathUtil.interpolate(thetaMaxAcceleration.get(), thetaMaxAccelerationTop.get(), 1)));
    }

    /**
     * Calculates our target field-relative drive velocity based on the current pose and target pose.
     */
    private Translation2d calculateDriveVelocity(Pose2d currentPose, Pose2d targetPose) {
        double linearFeedforwardScalar = MathUtil.clamp(
            (absoluteTranslationError - linearFFMinRadius.get()) / (linearFFMaxRadius.get() - linearFFMinRadius.get()),
            0.0, 1.0);

        var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();

        double setpointVelocity = direction.norm() <= minDistanceVelocityCorrection.get() // Don't calculate velocity in direction when really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();

        setpointVelocity = Math.max(setpointVelocity, setpointMinVelocity.get());
        State driveSetpoint = driveProfile.calculate(0.02, new State(direction.norm(), -setpointVelocity), // Negative because the profile has zero at the target
            new State(0.0, 0.0));

        Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
        Logger.recordOutput("DriveToPose/VelocitySetpoint", driveSetpoint.velocity);

        double driveVelocityScalar = driveController.calculate(absoluteTranslationError, driveSetpoint.position)
            + driveSetpoint.velocity * linearFeedforwardScalar;
        if(absoluteTranslationError < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;

        Rotation2d targetToCurrentAngle = currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);

        lastSetpointTranslation = new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero)).getTranslation();
        lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

        return driveVelocity;
    }

    /**
     * Calculates our target theta velocity based on the current pose and target pose.

    private double calculateThetaVelocity(Pose2d currentPose, Pose2d targetPose) {
        double thetaFeedforwardScalar = MathUtil
            .clamp((Units.radiansToDegrees(absoluteThetaError) - thetaFFMinError.get())
                / (thetaFFMaxError.get() - thetaFFMinError.get()), 0.0, 1.0);

        double thetaSetpointVelocity = Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians() / 0.02
            : thetaController.getSetpoint().velocity;

        double thetaVelocity = thetaController.calculate(currentPose.getRotation().getRadians(),
            new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity * thetaFeedforwardScalar;

        if(absoluteThetaError < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
        lastGoalRotation = targetPose.getRotation();

        return thetaVelocity;
    }

    /**
     * Resets the profile and the controllers to the current pose and target pose.
     */
    private void resetProfile() {
        Pose2d currentPose = pose.get();
        Pose2d targetPose = target.get();
        ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
        Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
            fieldVelocity.vyMetersPerSecond);

        driveProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));

        driveController.reset();
        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);

        lastSetpointTranslation = currentPose.getTranslation();
        lastSetpointVelocity = linearFieldVelocity;
        lastGoalRotation = targetPose.getRotation();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        running = false;

        // Clear logs
        Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        Logger.recordOutput("DriveToPose/TargetPose", new Pose2d[] {});
    }

    @Override
    public boolean isFinished() {
        if(endWhenAtTarget) { return atSetpoint(); }
        return false;
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean atSetpoint(double driveTolerance, Rotation2d thetaTolerance) {
        return running && Math.abs(absoluteTranslationError) < driveTolerance
            && Math.abs(absoluteThetaError) < thetaTolerance.getRadians();
    }

    /** Checks if the robot pose is within the allowed drive and theta tolerances. */
    public boolean atSetpoint() {
        return atSetpoint(endLinearTolerance.orElse(Units.inchesToMeters(driveTolerance.get())),
            Rotation2d.fromRadians(endThetaTolerance.orElse(Units.degreesToRadians(thetaTolerance.get()))));
    }
}
