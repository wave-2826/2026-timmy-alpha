package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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
    private static LoggedTunableNumber fudgeAzimuthOffsetDegCCW = new LoggedTunableNumber("ShotCalculator/FudgeAzimuthOffsetDegCCW", 0.0);
    private static LoggedTunableNumber fudgeHoodOffsetDeg = new LoggedTunableNumber("ShotCalculator/FudgeHoodOffsetDeg", 0.0);
    private static LoggedTunableNumber fudgeTimeOfFlightScale = new LoggedTunableNumber("ShotCalculator/FudgeTOFScale", 1.0);
    private static LoggedTunableNumber secondOrderCompensation = new LoggedTunableNumber("ShotCalculator/SecondOrderCompensation", 0.0);
    
    private static LoggedTunableNumber fudgeHubX = new LoggedTunableNumber("ShotCalculator/FudgeHubXInches", 0.0);
    private static LoggedTunableNumber fudgeHubY = new LoggedTunableNumber("ShotCalculator/FudgeHubYInches", 0.0);

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
                hoodAngleMap.get(distanceMeters) + fudgeHoodOffsetDeg.get()
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
            return timeOfFlightMap.get(distanceMeters) * fudgeTimeOfFlightScale.get();
        }
    }

    private static ShotMapData hubShots = new ShotMapData();
    private static ShotMapData passShots = new ShotMapData();
    
    private static LoggedTunableNumber phaseDelay = new LoggedTunableNumber("ShotCalculator/PhaseDelay", 0.04);
    /**
     * See https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html#the-drag-constant-k.
     * For fuel, we found that the piece lost 19.4% of its velocity over 6.3s. The linear velocity drag can be represented as v(t) = v_0 * e^-kt or
     * r = e^-kt => 0.806 = e^-k(6.3) => k = 0.0342
     */
    private static LoggedTunableNumber dragConstant = new LoggedTunableNumber("ShotCalculator/DragConstant", 0.43);
    /**
     * Bias on our shot target away from the robot position. Can make fuel bounces more consistent.
     */
    private static LoggedTunableNumber hubOutwardBiasInches = new LoggedTunableNumber("ShotCalculator/HubOutwardBiasInches", 6);

    private ShotParameters latestResult = null;

    static {
        // Hub shots

        // 1.486m: 2219.9 rpm / 29.9 deg / 1.10s
        // 2.28m:  2294.4 rpm / 37.4 deg / 1.04s
        // 2.64m:  2327.6 rpm / 40.7 deg / 0.93s
        // 3.41m:  2679.6 rpm / 40.9 deg / 1.17s
        // 4.33m:  2924.0 rpm / 41.9 deg / 1.39s
        // 5.35m:  3476.5 rpm / 41.9 deg / 1.45s

        hubShots.hoodAngleMap.put(1.486, 29.9);
        hubShots.hoodAngleMap.put(2.28,  37.4);
        hubShots.hoodAngleMap.put(2.64,  40.7);
        hubShots.hoodAngleMap.put(3.71,  42.2);
        hubShots.hoodAngleMap.put(4.4,   42.9);
        hubShots.hoodAngleMap.put(5.35,  44.0);
        hubShots.hoodAngleMap.put(6.41,  44.0);

        hubShots.flywheelSpeedMap.put(1.486, 2996.8);
        hubShots.flywheelSpeedMap.put(2.28,  3097.4);
        hubShots.flywheelSpeedMap.put(2.64,  3142.2);
        hubShots.flywheelSpeedMap.put(3.71,  3640.4);
        hubShots.flywheelSpeedMap.put(4.33,  3860.4);
        hubShots.flywheelSpeedMap.put(5.35,  3827.6);
        hubShots.flywheelSpeedMap.put(6.41,  4158.3);

        hubShots.timeOfFlightMap.put(1.486, 1.10);
        hubShots.timeOfFlightMap.put(2.28,  1.04);
        hubShots.timeOfFlightMap.put(2.64,  0.93);
        hubShots.timeOfFlightMap.put(3.41,  1.17);
        hubShots.timeOfFlightMap.put(4.33,  1.39);
        hubShots.timeOfFlightMap.put(5.35,  1.45);
        hubShots.timeOfFlightMap.put(6.41,  1.58);

        // Passing shots
        passShots.hoodAngleMap.put(5.46,  40.0);
        passShots.hoodAngleMap.put(17.16, 40.0);

        passShots.flywheelSpeedMap.put(5.46, 3274.0);
        passShots.flywheelSpeedMap.put(6.62, 3683.3);
        passShots.flywheelSpeedMap.put(7.80, 4092.5);

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
                return AllianceFlipUtil.apply(new Translation2d(2.094, FieldConstants.fieldWidthY - 1.372));
            case PASS_RIGHT:
                return AllianceFlipUtil.apply(new Translation2d(2.094, 1.372));
            default: // Hub shot
                Translation2d hubCenter = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
                return hubCenter.plus(
                    new Translation2d(Units.inchesToMeters(hubOutwardBiasInches.get()), 0.0)
                        .rotateBy(hubCenter.minus(turretPosition).getAngle())
                ).plus(new Translation2d(
                    Units.inchesToMeters(fudgeHubX.get()),
                    Units.inchesToMeters(fudgeHubY.get())
                ));
        }
    }

    // velocities from the previous cycle for acceleration estimation
    private double lastRobotVx = 0;
    private double lastRobotVy = 0;
    private double lastRobotOmega = 0;

    private Translation2d previousVirtualTarget;
    private ShotType previousShotType;
    private LinearFilter virtualVelocityFilterX = LinearFilter.movingAverage(4);
    private LinearFilter virtualVelocityFilterY = LinearFilter.movingAverage(4);

    private double applyPhaseDelay(double vel, double accel) {
        double phaseDelayDt = phaseDelay.get();
        if(Constants.isSim) return vel * phaseDelayDt * 0.4; // Hacky but oh well
        double firstOrder = vel * phaseDelayDt;
        double seconndOrder = vel * phaseDelayDt + 0.5 * accel * phaseDelayDt * phaseDelayDt;
        return MathUtil.interpolate(firstOrder, seconndOrder, secondOrderCompensation.get());
    }

    private double getEffectiveTOF(double tof) {
        return (1 - Math.exp(-dragConstant.get() * tof)) / dragConstant.get();
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

        // Second-order pose prediction based on estimated acceleration
        double ax = (robotRelativeVelocity.vxMetersPerSecond - lastRobotVx) / 0.02;
        double ay = (robotRelativeVelocity.vyMetersPerSecond - lastRobotVy) / 0.02;
        double aOmega = (robotRelativeVelocity.omegaRadiansPerSecond - lastRobotOmega) / 0.02;

        estimatedPose = estimatedPose.exp(new Twist2d(
            applyPhaseDelay(robotRelativeVelocity.vxMetersPerSecond, ax),
            applyPhaseDelay(robotRelativeVelocity.vyMetersPerSecond, ay),
            applyPhaseDelay(robotRelativeVelocity.omegaRadiansPerSecond, aOmega)
        ));
        lastRobotVx = robotRelativeVelocity.vxMetersPerSecond;
        lastRobotVy = robotRelativeVelocity.vyMetersPerSecond;
        lastRobotOmega = robotRelativeVelocity.omegaRadiansPerSecond;
        
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

        double timeOfFlight = type.shotMapData.getTimeOfFlight(turretToTargetDistance), effectiveTimeOfFlight = 0;
        Translation2d virtualTarget = target;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        double initialToEffectiveTOFScalar = timeOfFlight / getEffectiveTOF(timeOfFlight); // hack but it works?
        for(int i = 0; i < 20; i++) {
            timeOfFlight = type.shotMapData.getTimeOfFlight(lookaheadTurretToTargetDistance);
            // Calculate the effective time of flight, including induced linear drag. See
            // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
            // (Described in https://www.chiefdelphi.com/t/recursive-time-of-flight-fire-control-simulator-for-frc-docs-preview/513819/10)
            effectiveTimeOfFlight = getEffectiveTOF(timeOfFlight) * initialToEffectiveTOFScalar;

            Translation2d offset = new Translation2d(turretVelocityX * effectiveTimeOfFlight, turretVelocityY * effectiveTimeOfFlight);
            virtualTarget = target.minus(offset);
            lookaheadTurretToTargetDistance = virtualTarget.getDistance(turretPosition.getTranslation());
        }
        
        Logger.recordOutput("LaunchCalculator/RealTarget", target);
        Logger.recordOutput("LaunchCalculator/VirtualTarget", virtualTarget);
        Logger.recordOutput("LaunchCalculator/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("LaunchCalculator/EffectiveTimeOfFlight", effectiveTimeOfFlight);
        Logger.recordOutput("LaunchCalculator/TurretPosition", turretPosition);
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        var shotDirection = virtualTarget.minus(turretPosition.getTranslation());
        Rotation2d turretAngleAbsolute = shotDirection.getAngle().plus(
            Rotation2d.fromDegrees(fudgeAzimuthOffsetDegCCW.get())
        );
        Rotation2d turretAngleRobotRelative = turretAngleAbsolute.minus(RobotState.getInstance().getEstimatedPose().getRotation());
        double hoodAngleRad = MathUtil.clamp(
            type.shotMapData.getHood(lookaheadTurretToTargetDistance),
            TurretConstants.hoodMinAngle,
            TurretConstants.hoodMaxAngle
        );

        // TODO: incorporate virtual target velocity
        if(previousShotType != type) {
            previousShotType = type;
            previousVirtualTarget = virtualTarget;
        }
        var filteredVirtualTarget = new Translation2d(
            virtualVelocityFilterX.calculate(virtualTarget.getX()),
            virtualVelocityFilterY.calculate(virtualTarget.getY())
        );

        var discreteVirtualTargetVelocity = filteredVirtualTarget.minus(previousVirtualTarget).div(0.02);
        previousVirtualTarget = filteredVirtualTarget;

        var turretVelocity = new Translation2d(turretVelocityX, turretVelocityY);
        var relativeVelocity = discreteVirtualTargetVelocity.minus(turretVelocity);

        var azimuthVelocity = (
            shotDirection.cross(relativeVelocity) / shotDirection.getSquaredNorm()
        ) - robotRelativeVelocity.omegaRadiansPerSecond;
  
        double flywheelVelocity = type.shotMapData.getFlywheel(lookaheadTurretToTargetDistance);
        Logger.recordOutput("LaunchCalculator/Calculated/Flywheel", flywheelVelocity);
        Logger.recordOutput("LaunchCalculator/Calculated/HoodAngle", hoodAngleRad);
        Logger.recordOutput("LaunchCalculator/Calculated/AzimuthVelocity", azimuthVelocity);
        Logger.recordOutput("LaunchCalculator/Calculated/Azimuth", turretAngleRobotRelative);

        latestResult = new ShotParameters(
            type,
            new TurretTarget(
                flywheelVelocity,
                turretAngleRobotRelative.getRadians(),
                azimuthVelocity,
                hoodAngleRad
            )
        );
        return latestResult;
    }

    public ShotParameters getLatestResult() {
        return latestResult;
    }
}