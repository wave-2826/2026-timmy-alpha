// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.tunables.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

public class Module {
    private static final LoggedTunableNumber driveP = new LoggedTunableNumber("Drive/DriveP");
    private static final LoggedTunableNumber driveD = new LoggedTunableNumber("Drive/DriveD");

    private static final LoggedTunableNumber driveS = new LoggedTunableNumber("Drive/DriveS");
    private static final LoggedTunableNumber driveV = new LoggedTunableNumber("Drive/DriveV");
    private static final LoggedTunableNumber driveA = new LoggedTunableNumber("Drive/DriveA");

    private static final LoggedTunableNumber turnP = new LoggedTunableNumber("Drive/TurnP");
    private static final LoggedTunableNumber turnD = new LoggedTunableNumber("Drive/TurnD");

    static {
        driveP.initDefault(DriveConstants.driveGains.kP);
        driveD.initDefault(DriveConstants.driveGains.kD);

        driveS.initDefault(DriveConstants.driveGains.kS);
        driveV.initDefault(DriveConstants.driveGains.kV);
        driveA.initDefault(DriveConstants.driveGains.kA);

        turnP.initDefault(DriveConstants.steerGains.kP);
        turnD.initDefault(DriveConstants.steerGains.kD);
    }

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String name;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(ModuleIO io, String name) {
        this.name = name;
        this.io = io;
        driveDisconnectedAlert = new Alert("Disconnected drive motor on module " + name + ".", AlertType.kError);
        turnDisconnectedAlert = new Alert("Disconnected turn motor on module " + name + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert("Disconnected turn encoder on module " + name + ".", AlertType.kError);
    }

    public void periodic() {
        if(driveP.hasChanged(hashCode()) || driveD.hasChanged(hashCode()) ||
            driveS.hasChanged(hashCode()) || driveV.hasChanged(hashCode()) || driveA.hasChanged(hashCode())) {
            io.setDrivePID(driveP.get(), 0, driveD.get(), driveS.get(), driveV.get(), driveA.get());
        }
        if(turnP.hasChanged(hashCode()) || turnD.hasChanged(hashCode())) {
            io.setTurnPID(turnP.get(), 0, turnD.get());
        }

        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius.in(Meters);
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.turnAbsolutePosition);

        // Apply setpoints
        io.setDriveVelocity(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters));
        io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Characterize robot angular motion. */
    public void runAngularCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.fromDegrees(switch(name) {
            case "FrontLeft" -> 135.0;
            case "FrontRight" -> 45.0;
            case "BackLeft" -> -135.0;
            case "BackRight" -> -45.0;
            default -> 0.0;
        }));
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drivePositionRad * DriveConstants.wheelRadius.in(Meters);
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius.in(Meters);
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
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    /** Sets the current limit on the drive motor temporarily for slip current measurement. */
    public void setSlipMeasurementCurrentLimit(Current limit) {
        io.setSlipMeasurementCurrentLimit(limit);
    }
    /** Returns the drive motor current draw in amps. */
    public double getSlipMeasurementCurrent() {
        return inputs.driveCurrentAmps;
    }
}
