package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Controls;
import frc.robot.commands.tuning.TurretTuning;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.subsystems.turret.TurretIO.TurretIOPIDOutputs;

/**
 * Our robot has a triple-coaxial turret - all motors are static relative to the robot frame.  
 * The power transmission stack is as follows:
 * - Flywheel: 2x NEO Vortex, on the "innermost" coaxial stage; this will be affected by the azimuth rotation, but runs
 *   at a high velocity anyway so we don't care to compensate. The motors spin opposite, and the top needs to spin counterclockwise
 *   to shoot.
 * - Hood: 1x NEO Vortex, on the "middle" coaxial stage. Must run with the azimuth rotation to maintain a consistent hood angle.
 * - Azimuth: 1x NEO Vortex, on the "outermost" coaxial stage. Isn't affected by the other two stages and runs closed-loop over an
 *   attached absolute encoder.
 */
public class Turret extends SubsystemBase {
    public static class TurretTarget {
        public double flywheelSpeedRadPerSec;
        public double azimuthAngleRad;
        public double hoodAngleRad;

        public TurretTarget(double flywheelSpeedRadPerSec, double azimuthAngleRad, double hoodAngleRad) {
            this.flywheelSpeedRadPerSec = flywheelSpeedRadPerSec;
            this.azimuthAngleRad = azimuthAngleRad;
            this.hoodAngleRad = hoodAngleRad;
        }
    }
    private enum ControlMode {
        NONE,
        PID,
        LQR
    }

    private final TurretIO io;

    private final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

    private LoggedDashboardChooser<ControlMode> controlModeChooser = new LoggedDashboardChooser<>("Turret/ControlMode");

    public TurretTarget target = null;

    private TurretController controller = new TurretController();
    /** True once the LQR observer has been seeded with an initial measurement. */
    private boolean controllerInitialised = false;

    public Turret(TurretIO io) {
        this.io = io;

        controlModeChooser.addOption("PID", ControlMode.PID);
        controlModeChooser.addDefaultOption("LQR", ControlMode.LQR);

        TurretTuning.init();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", (TurretIOInputsAutoLogged)inputs);

        if(DriverStation.isTest()) return;

        if(target == null) {
            controllerInitialised = false;
            io.stop();

            Logger.recordOutput("Turret/Target/Azimuth", 0.0, Radians);
            Logger.recordOutput("Turret/Target/Hood", 0.0, Radians);
            Logger.recordOutput("Turret/Target/Flywheel", 0.0, RadiansPerSecond);

            TurretVisualizer.getInstance().update(
                0.0, inputs.getAzimuthAngleRad(),
                0.0, inputs.getHoodAngleRad()
            );
        } else {
            switch(controlModeChooser.get()) {
                case NONE:
                    return;
                case PID: {
                    TurretIOPIDOutputs outputs = new TurretIOPIDOutputs(
                        target.flywheelSpeedRadPerSec,
                        target.azimuthAngleRad % (Math.PI * 2),
                        (
                            MathUtil.clamp(
                                target.hoodAngleRad,
                                TurretConstants.hoodMinAngle,
                                TurretConstants.hoodMaxAngle
                            ) - TurretConstants.hoodMinAngle
                        ) / TurretConstants.hoodRingToHoodReduction
                    );
                    io.setPIDOutputs(outputs);
                    break;
                }
                case LQR: {
                    // Seed the observer the first time we enter LQR mode so it
                    // starts from the real measured state rather than zero.
                    if(!controllerInitialised) {
                        controller.reset(inputs);
                        controllerInitialised = true;
                    }
                    io.setMPCOutputs(controller.calculate(inputs, target));
                    break;
                }
            }
            
            Logger.recordOutput("Turret/Target/Azimuth", target.azimuthAngleRad, Radians);
            Logger.recordOutput("Turret/Target/Hood", target.hoodAngleRad, Radians);
            Logger.recordOutput("Turret/Target/Flywheel", target.flywheelSpeedRadPerSec, RadiansPerSecond);

            TurretVisualizer.getInstance().update(
                target.azimuthAngleRad, inputs.getAzimuthAngleRad(),
                target.hoodAngleRad, inputs.getHoodAngleRad()
            );
        }

        Logger.recordOutput("Turret/Measured/FlywheelVelocity", inputs.getFlywheelVelocityRadPerSecond(), RadiansPerSecond);
        Logger.recordOutput("Turret/Measured/Hood", inputs.getHoodAngleRad(), Radians);
        Logger.recordOutput("Turret/Measured/HoodVelocity", inputs.getHoodVelocityRadPerSec(), RadiansPerSecond);
        Logger.recordOutput("Turret/Measured/Azimuth", MathUtil.angleModulus(inputs.getAzimuthAngleRad()), Radians);
        Logger.recordOutput("Turret/Measured/AzimuthVelocity", inputs.getAzimuthVelocityRadPerSec(), RadiansPerSecond);
    }

    public Command runManual(
        DoubleSupplier flywheelSpeedSupplier,
        DoubleSupplier azimuthSpeedSupplier,
        DoubleSupplier hoodSpeedSupplier
    ) {
        return Commands.runEnd(() -> {
            if(target == null) {
                target = new TurretTarget(0.0, inputs.getAzimuthAngleRad(), TurretConstants.hoodMinAngle);
            }

            // target.flywheelSpeedRadPerSec = MathUtil.applyDeadband(-flywheelSpeedSupplier.getAsDouble(), 0.2) * TurretConstants.maxFlywheelSpeedRadPerSec;
            target.flywheelSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(3000);

            // target.azimuthAngleRad -= MathUtil.applyDeadband(azimuthSpeedSupplier.getAsDouble(), 0.2) * Math.PI * 0.02;
            // target.azimuthAngleRad %= Math.PI * 2;
            target.azimuthAngleRad = Math.sin(Timer.getFPGATimestamp() * 0.5) * Math.PI;
            // target.azimuthAngleRad = Math.PI / 2;

            // target.hoodAngleRad -= hoodSpeedSupplier.getAsDouble() * Math.PI * 0.008;
            // target.hoodAngleRad = MathUtil.clamp(target.hoodAngleRad, TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);

            target.hoodAngleRad = MathUtil.interpolate(
                TurretConstants.hoodMinAngle + 0.1,
                TurretConstants.hoodMaxAngle - 0.1,
                Math.sin(Timer.getFPGATimestamp() * 2) * 0.5 + 0.5
            );
        }, () -> {
            target = null;
        }, this);
    }

    public Command runTuning() {
        if(!(io instanceof TurretIOReal)) {
            // TODO: This is really temporary sob
            return Commands.none();
        }
        
        TurretTuning tuning = new TurretTuning((TurretIOReal)io, Controls.getInstance().coDriver);
        return Commands.runOnce(tuning::start).andThen(Commands.runEnd(tuning::run, tuning::stop, this));
    }
}