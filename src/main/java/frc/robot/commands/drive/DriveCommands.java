package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldBounds;
import frc.robot.util.GenericPIDConstants;
import frc.robot.util.GenericPIDConstants.PIDSlot;
import frc.robot.util.tunables.TunablePID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double DEADBAND = 0.1;

    private interface DriverAssist {
        public ChassisSpeeds apply(ChassisSpeeds fieldSpeeds, double joystickFieldX, double joystickFieldY);
        public default void log() {};
    }

    /**
     * A driver assist feature to keep the robot on a line if:
     * - Near the line
     * - Commanding movement roughly along the line
     * - OR actively moving into the line with clear intention to be on it
     */
    private static class LineDriverAssist implements DriverAssist {
        protected Translation2d lineStart;
        protected Translation2d lineEnd;

        protected static TunablePID lineAssistGains = new TunablePID("Drive/LineAssist")
            .addRealRobotGains(new GenericPIDConstants(2.0, 0.0, 0.1))
            .copyRealGainsInSim();
        protected PIDController lineCenterController = new PIDController(0., 0., 0.);

        private static Translation3d[][] assistLines = new Translation3d[][] {};

        public LineDriverAssist(Translation2d lineStart, Translation2d lineEnd) {
            this.lineStart = lineStart;
            this.lineEnd = lineEnd;
            // note: leaks controller
            lineAssistGains.configureController(lineCenterController, PIDSlot.Slot0);
        }

        protected Translation2d nearestPointOnLine(Translation2d point) {
            Translation2d lineVector = lineEnd.minus(lineStart);
            double t = point.minus(lineStart).dot(lineVector) / lineVector.getSquaredNorm();
            t = MathUtil.clamp(t, 0.0, 1.0);
            return lineStart.plus(lineVector.times(t));
        }

        public void flip() {
            lineStart = AllianceFlipUtil.flip(lineStart);
            lineEnd = AllianceFlipUtil.flip(lineEnd);
        }

        public void log() {
            var newLines = new Translation3d[assistLines.length + 1][2];
            for(int i = 0; i < assistLines.length; i++) {
                newLines[i] = assistLines[i];
            }
            newLines[assistLines.length] = new Translation3d[] {
                new Translation3d(lineStart.getX(), lineStart.getY(), 0.1),
                new Translation3d(lineEnd.getX(), lineEnd.getY(), 0.1)
            };
            assistLines = newLines;

            Logger.recordOutput("Drive/AssistLines", assistLines);
        }

        @Override
        public ChassisSpeeds apply(ChassisSpeeds fieldSpeeds, double joystickFieldX, double joystickFieldY) {
            Translation2d robotPos = RobotState.getInstance().getEstimatedPose().getTranslation();
            
            Translation2d nearestPoint = nearestPointOnLine(robotPos);
            double distanceFromLine = robotPos.getDistance(nearestPoint);

            // If the robot is close enough to the line, and the driver is commanding movement roughly along the line,
            // or is intentionally moving toward the line, apply assist to keep the robot centered on the line.

            final double LINE_PROXIMITY_THRESHOLD = 0.4; // meters
            final double ANGLE_ALLOWANCE = Units.degreesToRadians(25);
            final double INTENT_TOWARD_LINE_THRESHOLD = 0.2; // joystick magnitude toward line
            final double INTENT_ALONG_LINE_THRESHOLD = 0.2; // joystick magnitude along line
            final double MAX_CORRECTION_VEL = 1.5; // max correction velocity in m/s

            if(distanceFromLine > LINE_PROXIMITY_THRESHOLD) {
                return fieldSpeeds; // too far, no assist
            }

            // Vector along the line
            Translation2d lineVec = lineEnd.minus(lineStart);
            lineVec = lineVec.div(lineVec.getNorm()); // normalize
            // Vector from robot to nearest point on line
            Translation2d toLine = nearestPoint.minus(robotPos);
            Translation2d toLineNorm = toLine.div(toLine.getNorm() == 0 ? 1 : toLine.getNorm()); // normalize
            // Joystick vector (field-relative)
            Translation2d joystickVec = new Translation2d(joystickFieldX, joystickFieldY);

            // Project joystick vector onto line direction
            double alongLine = joystickVec.dot(lineVec);
            double towardLine = joystickVec.dot(toLineNorm);

            double intentHeuristic = 0.0;
            if(alongLine > 0 && Math.acos(lineVec.dot(joystickVec.div(joystickVec.getNorm() == 0 ? 1 : joystickVec.getNorm()))) < ANGLE_ALLOWANCE) {
                // Moving along the line in the correct direction
                intentHeuristic = Math.max(intentHeuristic, Math.min(alongLine / INTENT_ALONG_LINE_THRESHOLD, 1.0));
            }
            if(towardLine > 0 && Math.acos(toLineNorm.dot(joystickVec.div(joystickVec.getNorm() == 0 ? 1 : joystickVec.getNorm()))) < ANGLE_ALLOWANCE) {
                // Moving toward the line
                intentHeuristic = Math.max(intentHeuristic, Math.min(towardLine / INTENT_TOWARD_LINE_THRESHOLD, 1.0));
            }
            intentHeuristic *= MathUtil.clamp(1.0 - (distanceFromLine / LINE_PROXIMITY_THRESHOLD), 0.0, 1.0); // scale down intention based on distance
            intentHeuristic *= MathUtil.clamp(joystickVec.getNorm() / 0.5, 0.0, 1.0); // scale down intention if joystick input is small

            // Only assist if close to line and moving along or toward it
            if(intentHeuristic > 0.25) {
                // Amount of correction depends on how close we are to "full intention"
                double correctionMagnitude = Math.max(
                    Math.abs(alongLine) / INTENT_ALONG_LINE_THRESHOLD,
                    towardLine / INTENT_TOWARD_LINE_THRESHOLD
                );

                // PID correction to center on the line (perpendicular direction)
                double correction = lineCenterController.calculate(distanceFromLine, 0) * correctionMagnitude;
                
                // Apply correction perpendicular to the line
                Translation2d correctionVec = toLineNorm.times(
                    Math.min(correction, MAX_CORRECTION_VEL) * -1
                );

                double speed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
                // Project speed along the line direction
                Translation2d alongLineVec = lineVec.times(speed);

                double lineDirectionFactor = (intentHeuristic - 0.25) / (1.0 - 0.25);
                Translation2d usedSpeed = new Translation2d(
                    MathUtil.interpolate(fieldSpeeds.vxMetersPerSecond, alongLineVec.getX(), lineDirectionFactor),
                    MathUtil.interpolate(fieldSpeeds.vyMetersPerSecond, alongLineVec.getY(), lineDirectionFactor)
                );

                return new ChassisSpeeds(
                    usedSpeed.getX() + correctionVec.getX(),
                    usedSpeed.getY() + correctionVec.getY(),
                    fieldSpeeds.omegaRadiansPerSecond
                );
            } else {
                // No assist
                return fieldSpeeds;
            }
        }
    }

    private static class TrenchDriverAssist extends LineDriverAssist {
        private static Translation2d lineOffset = new Translation2d(Units.inchesToMeters(35), 0.);

        public TrenchDriverAssist(boolean red, boolean leftTrench) {
            super(
                (leftTrench ? FieldConstants.LeftTrench.center : FieldConstants.RightTrench.center).plus(lineOffset),
                (leftTrench ? FieldConstants.LeftTrench.center : FieldConstants.RightTrench.center).minus(lineOffset)
            );
            if(red) flip();
        }
    }

    private static class ZoneAssist implements DriverAssist {
        protected FieldBounds bounds;
        protected double lookaheadSeconds;

        private static Translation3d[][] assistZones = new Translation3d[][] {};
        
        public ZoneAssist(FieldBounds bounds, double lookaheadSeconds) {
            this.bounds = bounds;
            this.lookaheadSeconds = lookaheadSeconds;
        }

        @Override
        public ChassisSpeeds apply(ChassisSpeeds fieldSpeeds, double joystickFieldX, double joystickFieldY) {
            Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
            Translation2d futurePos = robotPose.exp(fieldSpeeds.toTwist2d(lookaheadSeconds)).getTranslation();

            if(bounds.contains(futurePos)) {
                return applyInZone(fieldSpeeds);
            }
            return fieldSpeeds;
        }

        public ChassisSpeeds applyInZone(ChassisSpeeds fieldSpeeds) {
            throw new UnsupportedOperationException("Must implement applyInZone for ZoneAssist");
        }

        @Override
        public void log() {
            var newZones = new Translation3d[assistZones.length + 1][4];
            for(int i = 0; i < assistZones.length; i++) {
                newZones[i] = assistZones[i];
            }
            newZones[assistZones.length] = new Translation3d[] {
                new Translation3d(bounds.minX(), bounds.minY(), 0.2),
                new Translation3d(bounds.maxX(), bounds.minY(), 0.2),
                new Translation3d(bounds.maxX(), bounds.maxY(), 0.2),
                new Translation3d(bounds.minX(), bounds.maxY(), 0.2),
                new Translation3d(bounds.minX(), bounds.minY(), 0.2)
            };
            assistZones = newZones;

            Logger.recordOutput("Drive/AssistZones", assistZones);
        }
    }

    private static class BumpDriverAssist extends ZoneAssist {
        // Turn the robot so it's not hotdog when traveling toward the bump or on it
        private PIDController thetaController = new PIDController(0.0, 0.0, 0.0);
        private TunablePID thetaGains = new TunablePID("Drive/BumpAssist")
            .addRealRobotGains(new GenericPIDConstants(3.0, 0.0, 0.1))
            .copyRealGainsInSim();

        public BumpDriverAssist(boolean red, boolean leftBump) {
            super(
                new FieldBounds(
                    leftBump ? FieldConstants.LeftBump.center : FieldConstants.RightBump.center,
                    Units.inchesToMeters(65),
                    FieldConstants.LeftBump.width
                ),
                0.25
            );
            if(red) {
                bounds = bounds.flipped();
            }

            // note: leaks controller
            thetaGains.configureController(thetaController, PIDSlot.Slot0);
            thetaController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
        }

        @Override
        public ChassisSpeeds applyInZone(ChassisSpeeds fieldSpeeds) {
            // If theta is greater than 30deg off from straight forward or straight backward, correct

            Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
            Rotation2d angle = robotPose.getRotation();

            if(Math.abs(angle.getCos()) < 0.866) { // more than 30deg off from straight forward/backward
                double targetAngle = angle.getCos() > 0 ? 0 : Math.PI; // target straight forward or backward
                double correction = thetaController.calculate(angle.getRadians(), targetAngle);
                return new ChassisSpeeds(
                    fieldSpeeds.vxMetersPerSecond,
                    fieldSpeeds.vyMetersPerSecond,
                    fieldSpeeds.omegaRadiansPerSecond + correction
                );
            }
            return fieldSpeeds;
        }
    }
    
    private DriveCommands() {}

    private static DriverAssist[] driverAssists = new DriverAssist[] {
        new TrenchDriverAssist(true, true),
        new TrenchDriverAssist(true, false),
        new TrenchDriverAssist(false, true),
        new TrenchDriverAssist(false, false),
        new BumpDriverAssist(true, true),
        new BumpDriverAssist(true, false),
        new BumpDriverAssist(false, true),
        new BumpDriverAssist(false, false)
    };

    static {
        if(Constants.isSim) {
            for(DriverAssist assist : driverAssists) {
                assist.log();
            }
        }
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y, double speedScalar) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude * speedScalar, 0.0, new Rotation2d()))
            .getTranslation();
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier,
        BooleanSupplier driveSlow
    ) {
        RobotState robotState = RobotState.getInstance();
        return Commands.run(() -> {
            double speedScalar = driveSlow.getAsBoolean() ? 0.5 : 1.0;
            
            // Get linear velocity
            Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble(), speedScalar);

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega) * speedScalar * (1 + linearVelocity.getNorm() * 0.3);

            // Convert to field relative speeds & send command
            ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * DriveConstants.linearFreeSpeed.in(MetersPerSecond),
                linearVelocity.getY() * DriveConstants.linearFreeSpeed.in(MetersPerSecond),
                omega * DriveConstants.maxAngularSpeedRadPerSec * 0.5);
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

            double fieldStickX = isFlipped ? -xSupplier.getAsDouble() : xSupplier.getAsDouble();
            double fieldStickY = isFlipped ? -ySupplier.getAsDouble() : ySupplier.getAsDouble();
            if(!robotState.odometryImpaired()) {
                for(DriverAssist assist : DriveCommands.driverAssists) {
                    speeds = assist.apply(speeds, fieldStickX, fieldStickY);
                }
            }
            
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped ? robotState.getRotation().plus(new Rotation2d(Math.PI)) : robotState.getRotation()),
                true);
        }, drive);
    }
}
