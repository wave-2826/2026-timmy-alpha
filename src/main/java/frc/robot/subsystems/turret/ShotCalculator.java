package frc.robot.subsystems.turret;

import java.nio.file.Path;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

    public static void warmUp() {
        // Warm up the shot calculator to construct it and
        // hopefully JIT some of the hot paths
        for(int i = 0; i < 100; i++) {
            getInstance().calculate();
        }
    }

    private static LoggedTunableNumber fudgeSpeedScale = new LoggedTunableNumber("ShotCalculator/FudgeSpeedScale", 1.0);
    private static LoggedTunableNumber fudgeAzimuthOffsetDegCCW = new LoggedTunableNumber("ShotCalculator/FudgeAzimuthOffsetDegCCW", 0.0);
    private static LoggedTunableNumber fudgeHoodOffsetDeg = new LoggedTunableNumber("ShotCalculator/FudgeHoodOffsetDeg", 0.0);
    private static LoggedTunableNumber fudgeTimeOfFlightScale = new LoggedTunableNumber("ShotCalculator/FudgeTOFScale", 1.8);
    private static LoggedTunableNumber secondOrderCompensation = new LoggedTunableNumber("ShotCalculator/SecondOrderCompensation", 0.0);
    
    private static LoggedTunableNumber fudgeHubX = new LoggedTunableNumber("ShotCalculator/FudgeHubXInches", 0.0);
    private static LoggedTunableNumber fudgeHubY = new LoggedTunableNumber("ShotCalculator/FudgeHubYInches", 0.0);

    private static class ShotMapData {
        /** Distance (m)-hood angle (deg) map */
        public InterpolatingDoubleTreeMap hoodAngleMap;
        /** Distance (m)-flywheel speed (rpm) map */
        public InterpolatingDoubleTreeMap flywheelSpeedMap;
        /** Distance (m)-ToF map (at the interpolated turret state) */
        public InterpolatingDoubleTreeMap timeOfFlightMap;

        public double maxDistance = 0.;
        public double minDistance = 100;

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

        private double getFlywheelVelocityRPM(double linearSpeedMPS) {
            return Units.radiansPerSecondToRotationsPerMinute(linearSpeedMPS / TurretConstants.flywheelRadius * 2. * 1.3);
        }

        public void loadFromCsv(String csvPath) {
            var path = Path.of(Filesystem.getDeployDirectory().getPath(), csvPath);
            try(var reader = java.nio.file.Files.newBufferedReader(path)) {
                String line;
                reader.readLine(); // skip header
                while((line = reader.readLine()) != null) {
                    String[] parts = line.split(",");
                    if(parts.length < 4) continue;
                    // distance,velocity,hood,tof
                    double distance = Double.parseDouble(parts[0]);
                    flywheelSpeedMap.put(distance, getFlywheelVelocityRPM(Double.parseDouble(parts[1])));
                    hoodAngleMap.put(distance, Double.parseDouble(parts[2]));
                    timeOfFlightMap.put(distance, Double.parseDouble(parts[3]));

                    maxDistance = Math.max(maxDistance, distance);
                    minDistance = Math.min(minDistance, distance);
                }
            } catch (Exception e) {
                DriverStation.reportError("Failed to load shot map!", false);
            }
        }
    }

    private static ShotMapData hubShots = new ShotMapData();
    private static ShotMapData passShots = new ShotMapData();
    
    private static LoggedTunableNumber phaseDelay = new LoggedTunableNumber("ShotCalculator/PhaseDelay", 0.02);
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

        hubShots.loadFromCsv("hub_shots.csv");
        passShots.loadFromCsv("lob_shots.csv");
    }

    public enum ShotType {
        NONE(null, 0., false),
        HUB(hubShots, 1., false),
        PASS_LEFT(passShots, 1.5, false),
        PASS_RIGHT(passShots, 1.5, false),
        HUB_TRENCH_STOW(hubShots, 0., true),
        PASS_LEFT_TRENCH_STOW(passShots, 0., true),
        PASS_RIGHT_TRENCH_STOW(passShots, 0., true);

        private ShotMapData shotMapData;
        // If 0, no shots can be made
        public double setpointToleranceScalar;
        public boolean stowHood;
        ShotType(ShotMapData shotMapData, double setpointToleranceScalar, boolean stowHood) {
            this.shotMapData = shotMapData;
            this.setpointToleranceScalar = setpointToleranceScalar;
            this.stowHood = stowHood;
        }
    }

    public record ShotParameters(
        ShotType shotType,
        /** The target or null if the turret should maintain its current target. */
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

    private Translation2d getTargetPosition(ShotType type, Translation2d turretPosition) {
        switch(type) {
            case PASS_LEFT, PASS_LEFT_TRENCH_STOW:
                return AllianceFlipUtil.apply(new Translation2d(3.2, FieldConstants.fieldWidthY - 2.47));
            case PASS_RIGHT, PASS_RIGHT_TRENCH_STOW:
                return AllianceFlipUtil.apply(new Translation2d(3.2, 2.47));
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
        double firstOrder = vel * phaseDelayDt;
        double seconndOrder = vel * phaseDelayDt + 0.5 * accel * phaseDelayDt * phaseDelayDt;
        return MathUtil.interpolate(firstOrder, seconndOrder, secondOrderCompensation.get());
    }

    private double getEffectiveTOF(double tof) {
        // Calculate the effective time of flight, including induced linear drag. See
        // https://frc-docs--3242.org.readthedocs.build/en/3242/docs/software/advanced-controls/fire-control/linear-drag.html
        // (Described in https://www.chiefdelphi.com/t/recursive-time-of-flight-fire-control-simulator-for-frc-docs-preview/513819/10)
        // (1 - e^(-kx) / k
        return (1 - Math.exp(-dragConstant.get() * tof)) / dragConstant.get();
    }
    private double getEffectiveTOFDerivative(double tof) {
        // Analytical derivative of effective tof wrt the input tof:
        // d/dt [ (1 - e^(-kt)) / k ] = e^(-kt)
        return Math.exp(-dragConstant.get() * tof);
    }

    public ShotType getShotType(Translation2d pos) {
        ShotType type = ShotType.HUB;
        if(
            FieldConstants.leftTrenchLowerZone.contains(pos) ||
            FieldConstants.rightTrenchLowerZone.contains(pos)
        ) {
            type = ShotType.HUB_TRENCH_STOW;
        } else if(FieldConstants.Tower.bounds.contains(pos)) {
            type = ShotType.HUB_TRENCH_STOW;
        } else if(FieldConstants.oppLeftTrenchLowerZone.contains(pos)) {
            type = ShotType.PASS_RIGHT_TRENCH_STOW;
        } else if(FieldConstants.oppRightTrenchLowerZone.contains(pos)) {
            type = ShotType.PASS_LEFT_TRENCH_STOW;
        } else if(
            FieldConstants.noPassZone.contains(pos) ||
            FieldConstants.Tower.oppBounds.contains(pos) ||
            FieldConstants.zoneSeparatorBounds.contains(pos) ||
            FieldConstants.oppZoneSeparatorBounds.contains(pos) ||
            FieldConstants.oppHubNoShotZone.contains(pos)
        ) {
            type = ShotType.NONE;
        } else if(leftPassBounds.contains(pos)) {
            type = ShotType.PASS_LEFT;
        } else if(rightPassBounds.contains(pos)) {
            type = ShotType.PASS_RIGHT;
        }

        return type;
    }

    public void storeShotTypeMapTelemetry() {
        // For telemetry purposes, sample the field and record which shot type is active at each position, then save to a CSV file
        int xSamples = (int)(FieldConstants.fieldLengthX / 0.1), ySamples = (int)(FieldConstants.fieldWidthY / 0.1);
        StringBuilder sb = new StringBuilder();
        sb.append("X,Y,ShotType\n");
        for(int i = 0; i <= xSamples; i++) {
            for(int j = 0; j <= ySamples; j++) {
                double x = i * 0.1, y = j * 0.1;
                ShotType type = getShotType(new Translation2d(x, y));
                sb.append(String.format("%f,%f,%s\n", x, y, type.name()));
            }
        }
        var path = Path.of("logs/shot_type_map.csv");
        try {
            java.nio.file.Files.writeString(path, sb.toString());
        } catch (Exception e) {
            DriverStation.reportError("Failed to save shot type map!", false);
        }
    }
    
    public ShotParameters calculate() {
        Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

        // Distance from turret to target
        Pose2d turretPose = estimatedPose.transformBy(
            new Transform2d(TurretConstants.robotToTurret.toTranslation2d(), Rotation2d.kZero)
        );

        Pose2d zoneCheckPosition = AllianceFlipUtil.apply(turretPose);
        ShotType type = getShotType(zoneCheckPosition.getTranslation());

        Logger.recordOutput("LaunchCalculator/ShotType", type);
        if(type == ShotType.NONE || type.shotMapData == null) {
            return new ShotParameters(type, null);
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
        
        Translation2d target = getTargetPosition(type, turretPose.getTranslation());
        double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

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
        
        // Use Newton's method to converge faster and more often
        // We could warm start this with the previous state but that probably
        // wouldn't help much since the target changes discontinuously when we switch shot types
        int iterations = 0;
        for(int i = 0; i < 5; i++) {
            // Evaluate current timeOfFlight to get distance and mapped target ToF
            effectiveTimeOfFlight = getEffectiveTOF(timeOfFlight);
            Translation2d offset = new Translation2d(turretVelocityX * effectiveTimeOfFlight, turretVelocityY * effectiveTimeOfFlight);
            virtualTarget = target.minus(offset);
            
            double dx = virtualTarget.getX() - turretPose.getX();
            double dy = virtualTarget.getY() - turretPose.getY();
            lookaheadTurretToTargetDistance = Math.hypot(dx, dy);
            
            double mappedToF = type.shotMapData.getTimeOfFlight(lookaheadTurretToTargetDistance);
            double deltaToF = timeOfFlight - mappedToF; // f(t) = t - mapped(t)
            
            if(Math.abs(deltaToF) < 1e-4) break; // Converged
            
            // Analytical derivative of distance wrt effective ToF
            double distanceDerivative = lookaheadTurretToTargetDistance > 1e-6 ? 
                (dx * -turretVelocityX + dy * -turretVelocityY) / lookaheadTurretToTargetDistance : 0.0;
                
            // Numerical derivative of the lookup map (since it lacks an analytical expression)
            double h = 1e-3;
            double tofMapDerivative = (type.shotMapData.getTimeOfFlight(lookaheadTurretToTargetDistance + h) - mappedToF) / h;
            
            // Chain rule f'(t) = 1 - (d_map / d_dist) * (d_dist / d_teff) * (d_teff / d_t)
            double derivative = 1.0 - (tofMapDerivative * distanceDerivative * getEffectiveTOFDerivative(timeOfFlight));
            
            // Prevent division by zero
            if(Math.abs(derivative) < 1e-6) break;
            
            timeOfFlight -= deltaToF / derivative;
            iterations += 1;
        }
        
        Logger.recordOutput("LaunchCalculator/NewtonIterations", iterations);
        Logger.recordOutput("LaunchCalculator/RealTarget", target);
        Logger.recordOutput("LaunchCalculator/VirtualTarget", virtualTarget);
        Logger.recordOutput("LaunchCalculator/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("LaunchCalculator/EffectiveTimeOfFlight", effectiveTimeOfFlight);
        Logger.recordOutput("LaunchCalculator/TurretPosition", turretPose);
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        if(lookaheadTurretToTargetDistance > type.shotMapData.maxDistance || lookaheadTurretToTargetDistance < type.shotMapData.minDistance) {
            // Target is out of range of our shot map, so we won't be accurate. Don't even try to shoot.
            return new ShotParameters(ShotType.NONE, null);
        }

        var shotDirection = virtualTarget.minus(turretPose.getTranslation());
        Rotation2d turretAngleAbsolute = shotDirection.getAngle().plus(
            Rotation2d.fromDegrees(fudgeAzimuthOffsetDegCCW.get())
        );
        Rotation2d turretAngleRobotRelative = turretAngleAbsolute.minus(RobotState.getInstance().getEstimatedPose().getRotation());
        double hoodAngleRad = MathUtil.clamp(
            type.shotMapData.getHood(lookaheadTurretToTargetDistance),
            TurretConstants.hoodMinAngle,
            TurretConstants.hoodMaxAngle
        );

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

        if(type == ShotType.HUB_TRENCH_STOW) {
            hoodAngleRad = Math.min(hoodAngleRad, TurretConstants.maxTrenchHoodAngle);
        }

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