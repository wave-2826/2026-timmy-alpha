package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LeftTrench;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.Turret.TurretTarget;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldBounds;
import frc.robot.util.tunables.LoggedTunableNumber;

public class ShotCalculator {
    private static ShotCalculator instance = null;
    public static ShotCalculator getInstance() {
        if(instance == null) {
            instance = new ShotCalculator();
        }
        return instance;
    }

    private static LoggedTunableNumber fudgeSpeedScale = new LoggedTunableNumber("ShotCalculator/FudgeSpeedScale", 1.0);
    private static LoggedTunableNumber fudgeAngleOffsetDeg = new LoggedTunableNumber("ShotCalculator/FudgeAngleOffsetDeg", 0.0);

    private static class ShotMapData {
        /** Distance (m)-hood angle (deg) map */
        InterpolatingDoubleTreeMap hoodAngleMap;
        /** Distance (m)-flywheel speed (rpm) map */
        InterpolatingDoubleTreeMap flywheelSpeedMap;
        /** Distance (m)-ToF map (at the interpolated turret state) */
        InterpolatingDoubleTreeMap timeOfFlightMap;

        public ShotMapData() {
            hoodAngleMap = new InterpolatingDoubleTreeMap();
            flywheelSpeedMap = new InterpolatingDoubleTreeMap();
            timeOfFlightMap = new InterpolatingDoubleTreeMap();
        }

        /** Get the hood angle in rad */
        public double getHood(double distanceMeters) {
            return Units.degreesToRadians(
                hoodAngleMap.get(distanceMeters) + fudgeAngleOffsetDeg.get()
            );
        }
        /** Get the flywheel velocity in RPM */
        public double getFlywheel(double distanceMeters) {
            return Units.rotationsPerMinuteToRadiansPerSecond(
                flywheelSpeedMap.get(distanceMeters)
            ) * fudgeSpeedScale.get();
        }
        /** Get the ToF in seconds */
        public double getTimeOfFlight(double distanceMeters) {
            return timeOfFlightMap.get(distanceMeters);
        }
    }

    private static ShotMapData hubShots = new ShotMapData();
    private static ShotMapData passShots = new ShotMapData();
    
    private static LoggedTunableNumber phaseDelay = new LoggedTunableNumber("ShotCalculator/PhaseDelay", 0.03);
    /**
     * See https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html#the-drag-constant-k.
     * For fuel, we found that the piece lost 19.4% of its velocity over 6.3s. The linear velocity drag can be represented as v(t) = v_0 * e^-kt or
     * r = e^-kt => 0.806 = e^-k(6.3) => k = 0.0342
     */
    private static LoggedTunableNumber dragConstant = new LoggedTunableNumber("ShotCalculator/DragConstant", 0.0342);
    /**
     * Bias on our shot target away from the robot position. Can make fuel bounces more consistent.
     */
    private static LoggedTunableNumber hubOutwardBiasInches = new LoggedTunableNumber("ShotCalculator/HubOutwardBiasInches", 6);

    static {
        // TODO: These are just directly stolen from 6328... Tune ourselves!

        // 43deg / 5.69m

        // Hub shots
        hubShots.hoodAngleMap.put(0.96, 15.0);
        hubShots.hoodAngleMap.put(1.16, 20.0);
        hubShots.hoodAngleMap.put(1.58, 28.0);
        hubShots.hoodAngleMap.put(2.94, 35.0);
        hubShots.hoodAngleMap.put(5.69, 43.0);

        hubShots.flywheelSpeedMap.put(0.96, 2690.4);
        hubShots.flywheelSpeedMap.put(1.16, 2860.0);
        hubShots.flywheelSpeedMap.put(1.58, 3029.8);
        hubShots.flywheelSpeedMap.put(2.07, 3199.4);
        hubShots.flywheelSpeedMap.put(2.37, 3369.1);
        hubShots.flywheelSpeedMap.put(2.47, 3369.1);
        hubShots.flywheelSpeedMap.put(2.70, 3369.1);
        hubShots.flywheelSpeedMap.put(2.94, 3538.8);
        hubShots.flywheelSpeedMap.put(3.48, 3538.8);
        hubShots.flywheelSpeedMap.put(3.92, 3708.5);
        hubShots.flywheelSpeedMap.put(4.35, 3878.2);
        hubShots.flywheelSpeedMap.put(4.84, 4047.9);

        hubShots.timeOfFlightMap.put(5.68, 1.16);
        hubShots.timeOfFlightMap.put(4.55, 1.12);
        hubShots.timeOfFlightMap.put(3.15, 1.11);
        hubShots.timeOfFlightMap.put(1.88, 1.09);
        hubShots.timeOfFlightMap.put(1.38, 0.90);

        // Passing shots
        passShots.hoodAngleMap.put(5.46,  40.0);
        passShots.hoodAngleMap.put(17.16, 40.0);

        passShots.flywheelSpeedMap.put(5.46, 4583.662);
        passShots.flywheelSpeedMap.put(6.62, 5156.620);
        passShots.flywheelSpeedMap.put(7.80, 5729.577);

        passShots.timeOfFlightMap.put(5.46,  1.27);
        passShots.timeOfFlightMap.put(6.62,  1.39);
        passShots.timeOfFlightMap.put(7.8,   1.49);
        passShots.timeOfFlightMap.put(11.0,  1.75);
        passShots.timeOfFlightMap.put(13.0,  1.76);
        passShots.timeOfFlightMap.put(17.16, 2.16);
    }

    public enum ShotType {
        NONE(null),
        HUB(hubShots),
        PASS_LEFT(passShots),
        PASS_RIGHT(passShots);

        private ShotMapData shotMapData;
        ShotType(ShotMapData shotMapData) {
            this.shotMapData = shotMapData;
        }
    }

    public record ShotParameters(
        ShotType shotType,
        TurretTarget target
    ) {}

    private FieldBounds leftPassBounds = new FieldBounds(
        LinesVertical.hubCenter + LeftTrench.depth / 2, FieldConstants.fieldLengthX,
        FieldConstants.LinesHorizontal.center, FieldConstants.fieldWidthY
    );
    private FieldBounds rightPassBounds = new FieldBounds(
        LinesVertical.hubCenter + LeftTrench.depth / 2, FieldConstants.fieldLengthX,
        0, FieldConstants.LinesHorizontal.center
    );
    private FieldBounds oppHubNoShotZone = new FieldBounds(
        FieldConstants.LinesVertical.neutralZoneFar, FieldConstants.fieldLengthX,
        FieldConstants.LinesHorizontal.leftBumpEnd, FieldConstants.LinesHorizontal.rightBumpStart
    );

    private Translation2d getTargetPosition(ShotType type, Translation2d turretPosition) {
        switch(type) {
            case PASS_LEFT:
                return AllianceFlipUtil.apply(new Translation2d(2.094, 1.372));
            case PASS_RIGHT:
                return AllianceFlipUtil.apply(new Translation2d(2.094, FieldConstants.fieldWidthY - 1.372));
            default: // Hub shot
                Translation2d hubCenter = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
                return hubCenter.plus(
                    new Translation2d(Units.inchesToMeters(hubOutwardBiasInches.get()), 0.0)
                        .rotateBy(hubCenter.minus(turretPosition).getAngle())
                );
        }
    }

    public ShotParameters calculate() {
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

        // Distance from turret to target
        Pose2d turretPosition = estimatedPose.transformBy(
            new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero)
        );

        Pose2d zoneCheckPosition = AllianceFlipUtil.apply(turretPosition);
        ShotType type = ShotType.HUB;
        if(FieldConstants.Tower.bounds.contains(zoneCheckPosition)) {
            type = ShotType.NONE;
        } else if(FieldConstants.zoneSeparatorBounds.contains(zoneCheckPosition) || FieldConstants.oppZoneSeparatorBounds.contains(zoneCheckPosition)) {
            type = ShotType.NONE;
        } else if(oppHubNoShotZone.contains(zoneCheckPosition)) {
            type = ShotType.NONE;
        } else if(leftPassBounds.contains(zoneCheckPosition)) {
            type = ShotType.PASS_LEFT;
        } else if(rightPassBounds.contains(zoneCheckPosition)) {
            type = ShotType.PASS_RIGHT;
        }

        Logger.recordOutput("LaunchCalculator/ShotType", type);
        if(type == ShotType.NONE) {
            return new ShotParameters(ShotType.NONE, new TurretTarget(0, 0, 0));
        }

        ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
        estimatedPose = estimatedPose.exp(new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * phaseDelay.get(),
            robotRelativeVelocity.vyMetersPerSecond * phaseDelay.get(),
            robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay.get()
        ));

        Translation2d target = getTargetPosition(type, turretPosition.getTranslation());
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = robotVelocity.vxMetersPerSecond + robotVelocity.omegaRadiansPerSecond
            * (TurretConstants.robotToTurret.getY() * Math.cos(robotAngle)
            - TurretConstants.robotToTurret.getX() * Math.sin(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond + robotVelocity.omegaRadiansPerSecond
            * (TurretConstants.robotToTurret.getX() * Math.cos(robotAngle)
            - TurretConstants.robotToTurret.getY() * Math.sin(robotAngle));

        double timeOfFlight, effectiveTimeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = type.shotMapData.getTimeOfFlight(lookaheadTurretToTargetDistance);
            // Calculate the effective time of flight, including induced linear drag. See
            // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
            // (Described in https://www.chiefdelphi.com/t/recursive-time-of-flight-fire-control-simulator-for-frc-docs-preview/513819/10)
            effectiveTimeOfFlight = (1 - Math.exp(-dragConstant.get() * timeOfFlight)) / dragConstant.get();

            Translation2d offset = new Translation2d(turretVelocityX * effectiveTimeOfFlight, turretVelocityY * effectiveTimeOfFlight);
            lookaheadPose = new Pose2d(
                turretPosition.getTranslation().plus(offset),
                turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }
        
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        Rotation2d turretAngleAbsolute = target.minus(lookaheadPose.getTranslation()).getAngle();
        Rotation2d turretAngleRobotRelative = turretAngleAbsolute.minus(RobotState.getInstance().getEstimatedPose().getRotation());
        double hoodAngleRad = type.shotMapData.getHood(lookaheadTurretToTargetDistance);
  
        double flywheelVelocity = type.shotMapData.getFlywheel(lookaheadTurretToTargetDistance);
        Logger.recordOutput("LaunchCalculator/Calculated/Flywheel", flywheelVelocity);
        Logger.recordOutput("LaunchCalculator/Calculated/HoodAngle", hoodAngleRad);
        Logger.recordOutput("LaunchCalculator/Calculated/Azimuth", turretAngleRobotRelative);

        return new ShotParameters(
            type,
            new TurretTarget(
                flywheelVelocity,
                turretAngleRobotRelative.getRadians(),
                hoodAngleRad
            )
        );
    }
}