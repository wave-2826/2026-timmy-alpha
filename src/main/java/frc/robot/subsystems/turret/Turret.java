package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;

/**
 * Our robot has a triple-coaxial turret - all motors are static relative to the robot frame.  
 * The power transmission stack is as follows:
 * - Flywheel: 2x NEO Vortex, on the "innermost" coaxial stage; this will be affected by both azimuth and hood rotation, but runs
 *   at a high velocity anyway so we don't care to compensate. The motors spin opposite, and the top needs to spin counterclockwise
 *   to shoot.
 * - Hood: 1x NEO Vortex, on the "middle" coaxial stage. Must run with the azimuth rotation to maintain a consistent hood angle.
 * - Azimuth: 1x NEO Vortex, on the "outermost" coaxial stage. Isn't affected by the other two stages and runs closed-loop over an
 *   attached absolute encoder.
 */
public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public class TurretTarget {
        public double flywheelSpeedRadPerSec;
        public double azimuthAngleRad;
        public double hoodRingAngleDiffRad;

        public TurretTarget(double flywheelSpeedRadPerSec, double azimuthAngleRad, double hoodRingAngleDiffRad) {
            this.flywheelSpeedRadPerSec = flywheelSpeedRadPerSec;
            this.azimuthAngleRad = azimuthAngleRad;
            this.hoodRingAngleDiffRad = hoodRingAngleDiffRad;
        }
    }

    public TurretTarget target = null;

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        if(target == null) {
            io.stop();
        } else {
            io.setOutputs(new TurretIOOutputs(target.flywheelSpeedRadPerSec, target.azimuthAngleRad, target.hoodRingAngleDiffRad));
        }
    }

    public Command runManual(
        DoubleSupplier flywheelSpeedSupplier,
        DoubleSupplier azimuthSpeedSupplier,
        DoubleSupplier hoodSpeedSupplier
    ) {
        return Commands.runEnd(() -> {
            if(target == null) {
                target = new TurretTarget(0.0, inputs.azimuth.azimuthAngleRad(), inputs.hood.hoodRingAngleRad());
            }
            target.flywheelSpeedRadPerSec = flywheelSpeedSupplier.getAsDouble() * TurretConstants.maxFlywheelSpeedRadPerSec;
            target.azimuthAngleRad += azimuthSpeedSupplier.getAsDouble() * TurretConstants.maxAzimuthSpeedRadPerSec * 0.5;
            target.hoodRingAngleDiffRad += hoodSpeedSupplier.getAsDouble() * TurretConstants.maxHoodRingSpeedRadPerSec * 0.05;
        }, () -> {
            target = null;
        }, this);
    }
}