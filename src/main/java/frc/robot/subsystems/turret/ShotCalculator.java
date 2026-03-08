package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.Turret.TurretTarget;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.tunables.LoggedTunableNumber;

public class ShotCalculator {
    private static ShotCalculator instance = null;
    public static ShotCalculator getInstance() {
        if(instance == null) {
            instance = new ShotCalculator();
        }
        return instance;
    }

    private static class ShotMapData {
        /** Distance (m)-hood angle (rad) map */
        InterpolatingDoubleTreeMap hoodAngleMap;
        /** Distance (m)-flywheel speed (rad/s) map */
        InterpolatingDoubleTreeMap flywheelSpeedMap;
        /** Distance (m)-ToF map (at the interpolated turret state) */
        InterpolatingDoubleTreeMap timeOfFlightMap;

        ShotMapData() {
            hoodAngleMap = new InterpolatingDoubleTreeMap();
            flywheelSpeedMap = new InterpolatingDoubleTreeMap();
            timeOfFlightMap = new InterpolatingDoubleTreeMap();
        }

        public double getHood(double distanceMeters) {
            return hoodAngleMap.get(distanceMeters);
        }
        public double getFlywheel(double distanceMeters) {
            return flywheelSpeedMap.get(distanceMeters);
        }
        public double getTimeOfFlight(double distanceMeters) {
            return timeOfFlightMap.get(distanceMeters);
        }
    }

    private static ShotMapData hubShots = new ShotMapData();
    private static ShotMapData passShots = new ShotMapData();

    static {
        // TODO: These are just directly stolen from 6328... Tune ourselves!

        // Hub shots
        hubShots.hoodAngleMap.put(0.96, 0.0044209);
        hubShots.hoodAngleMap.put(1.16, 0.0053051);
        hubShots.hoodAngleMap.put(1.58, 0.0061893);
        hubShots.hoodAngleMap.put(2.07, 0.0081787);
        hubShots.hoodAngleMap.put(2.37, 0.0097261);
        hubShots.hoodAngleMap.put(2.47, 0.0101682);
        hubShots.hoodAngleMap.put(2.70, 0.0106103);
        hubShots.hoodAngleMap.put(2.94, 0.0110524);
        hubShots.hoodAngleMap.put(3.48, 0.0119366);
        hubShots.hoodAngleMap.put(3.92, 0.0141471);
        hubShots.hoodAngleMap.put(4.35, 0.0150313);
        hubShots.hoodAngleMap.put(4.84, 0.0167996);

        hubShots.flywheelSpeedMap.put(0.96, 150.0);
        hubShots.flywheelSpeedMap.put(1.16, 155.0);
        hubShots.flywheelSpeedMap.put(1.58, 160.0);
        hubShots.flywheelSpeedMap.put(2.07, 165.0);
        hubShots.flywheelSpeedMap.put(2.37, 170.0);
        hubShots.flywheelSpeedMap.put(2.47, 170.0);
        hubShots.flywheelSpeedMap.put(2.70, 170.0);
        hubShots.flywheelSpeedMap.put(2.94, 175.0);
        hubShots.flywheelSpeedMap.put(3.48, 175.0);
        hubShots.flywheelSpeedMap.put(3.92, 180.0);
        hubShots.flywheelSpeedMap.put(4.35, 185.0);
        hubShots.flywheelSpeedMap.put(4.84, 190.0);

        hubShots.timeOfFlightMap.put(5.68, 1.16);
        hubShots.timeOfFlightMap.put(4.55, 1.12);
        hubShots.timeOfFlightMap.put(3.15, 1.11);
        hubShots.timeOfFlightMap.put(1.88, 1.09);
        hubShots.timeOfFlightMap.put(1.38, 0.90);

        // Passing shots
        passShots.hoodAngleMap.put(5.46,  0.6632251);
        passShots.hoodAngleMap.put(17.16, 0.6632251);

        passShots.flywheelSpeedMap.put(5.46,  160.0);
        passShots.flywheelSpeedMap.put(6.62,  180.0);
        passShots.flywheelSpeedMap.put(7.80,  200.0);
        passShots.flywheelSpeedMap.put(17.16, 360.0);

        passShots.timeOfFlightMap.put(5.46, 1.27);
        passShots.timeOfFlightMap.put(6.62, 1.39);
        passShots.timeOfFlightMap.put(7.8, 1.49);
        passShots.timeOfFlightMap.put(11.0, 1.75);
        passShots.timeOfFlightMap.put(13.0, 1.76);
        passShots.timeOfFlightMap.put(17.16, 2.16);
    }

    public enum ShotType {
        NONE(null),
        HUB(hubShots),
        PASS(passShots);

        private ShotMapData shotMapData;
        ShotType(ShotMapData shotMapData) {
            this.shotMapData = shotMapData;
        }
    }

    public record ShotParameters(
        ShotType shotType,
        TurretTarget target
    ) {}

    private static LoggedTunableNumber phaseDelay = new LoggedTunableNumber("ShotCalculator/PhaseDelay", 0.02);

    private Translation2d getTargetPosition() {
        return AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    }

    public ShotParameters calculate() {
        ShotType type = ShotType.HUB;

        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
        ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
        estimatedPose = estimatedPose.exp(new Twist2d(
            robotRelativeVelocity.vxMetersPerSecond * phaseDelay.get(),
            robotRelativeVelocity.vyMetersPerSecond * phaseDelay.get(),
            robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay.get()
        ));

        // Distance from turret to target
        Translation2d target = getTargetPosition();
        Pose2d turretPosition = estimatedPose.transformBy(
            new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero)
        );
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

        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = type.shotMapData.getTimeOfFlight(lookaheadTurretToTargetDistance);
            Translation2d offset = new Translation2d(turretVelocityX * timeOfFlight, turretVelocityY * timeOfFlight);
            lookaheadPose = new Pose2d(
                turretPosition.getTranslation().plus(offset),
                turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }
    
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);    

        Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        double hoodAngleRad = type.shotMapData.getHood(lookaheadTurretToTargetDistance);

        return new ShotParameters(
            type,
            new TurretTarget(
                type.shotMapData.getFlywheel(lookaheadTurretToTargetDistance),
                turretAngle.getRadians(),
                hoodAngleRad
            )
        );
    }
}