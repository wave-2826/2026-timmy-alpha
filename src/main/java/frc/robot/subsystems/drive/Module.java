package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.tunables.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Module {
    private static final LoggedTunableNumber driveP = new LoggedTunableNumber("Drive/DriveP");
    private static final LoggedTunableNumber driveD = new LoggedTunableNumber("Drive/DriveD");

    private static final LoggedTunableNumber driveA = new LoggedTunableNumber("Drive/DriveA");

    private static final LoggedTunableNumber turnP = new LoggedTunableNumber("Drive/TurnP");
    private static final LoggedTunableNumber turnD = new LoggedTunableNumber("Drive/TurnD");

    private static final LoggedTunableNumber speedScalar = new LoggedTunableNumber("Drive/SpeedScalar", 1.0);

    static {
        driveP.initDefault(DriveConstants.driveGains.kP);
        driveD.initDefault(DriveConstants.driveGains.kD);
        driveA.initDefault(DriveConstants.driveGains.kA);

        turnP.initDefault(DriveConstants.steerGains.kP);
        turnD.initDefault(DriveConstants.steerGains.kD);
    }

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final String name;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /** The angle at which this module will turn the robot clockwise. */
    public final Rotation2d spinAngle;
    public final Rotation2d angleToCenter;

    public Module(ModuleIO io, String name, Translation2d translation) {
        this.name = name;
        this.io = io;
        driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + name + ".", AlertType.kError);
        turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + name + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + name + ".", AlertType.kError);

        this.angleToCenter = new Rotation2d(translation.getY(), translation.getX());
        this.spinAngle = angleToCenter.plus(Rotation2d.kCW_90deg);

        // Reset tunables' hasChanged since we'll configure anyway
        driveP.hasChanged(hashCode());
        driveD.hasChanged(hashCode());
        driveA.hasChanged(hashCode());
        turnP.hasChanged(hashCode());
        turnD.hasChanged(hashCode());
        
        Drive.tuningResults.onChange(() -> setDrivePID());
    }

    private void setDrivePID() {
        io.setDrivePID(
            driveP.get(), 0, driveD.get(),
            Drive.tuningResults.feedforwardResults.kS(), Drive.tuningResults.feedforwardResults.kV(),
            driveA.get()
        );
    }

    public void periodic() {
        if(driveP.hasChanged(hashCode()) || driveD.hasChanged(hashCode()) || driveA.hasChanged(hashCode())) {
            setDrivePID();
        }
        if(turnP.hasChanged(hashCode()) || turnD.hasChanged(hashCode()) ) {
            io.setTurnPID(turnP.get(), 0, turnD.get());
        }

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * Drive.tuningResults.wheelRadiusResults.radiusMeters();
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    private record OptimizePair(SwerveModuleState state, double acceleration) {}
    /**
     * Optimize the module state and 
     * @param currentAngle
     * @param accelerationMps2
     * @return
     */
    private OptimizePair optimizeState(SwerveModuleState state, Rotation2d currentAngle, double accelerationMps2) {
        var delta = state.angle.minus(currentAngle);
        if(Math.abs(delta.getDegrees()) > 90.0) {
            state.speedMetersPerSecond *= -1;
            state.angle = state.angle.rotateBy(Rotation2d.kPi);
            accelerationMps2 *= -1;
        }
        return new OptimizePair(state, accelerationMps2);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state, double accelerationMps2) {
        // Optimize velocity setpoint
        var pair = optimizeState(state, getAngle(), accelerationMps2);
        state = pair.state;
        accelerationMps2 = pair.acceleration;

        state.cosineScale(inputs.turnAbsolutePosition);

        // Apply setpoints
        io.setDriveVelocity(
            state.speedMetersPerSecond / Drive.tuningResults.wheelRadiusResults.radiusMeters() * speedScalar.get(),
            accelerationMps2 / Drive.tuningResults.wheelRadiusResults.radiusMeters()
        );
        io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoopCurrent(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Characterize robot angular motion. */
    public void runAngularCharacterization(double output) {
        io.setDriveOpenLoopCurrent(output);
        io.setTurnPosition(spinAngle);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoopCurrent(0.0);
        io.setTurnOpenLoopCurrent(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * Drive.tuningResults.wheelRadiusResults.radiusMeters();
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * Drive.tuningResults.wheelRadiusResults.radiusMeters();
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getModuleCharacterizationPosiiton() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    /** Gets the zero offset of the module. This is the amount to be added to the encoder. */
    public Rotation2d getZeroOffset() {
        return inputs.uncorrectedTurnAbsolute.unaryMinus();
    }

    /** Sets the current limit on the drive motor temporarily for slip current measurement. Pass null to reset. */
    public void setSlipMeasurementCurrentLimit(Current limit) {
        io.setSlipMeasurementCurrentLimit(limit);
    }
    /** Returns the drive motor current draw in amps. */
    public double getCharacterizationCurrent() {
        return inputs.driveCurrentAmps;
    }
}
