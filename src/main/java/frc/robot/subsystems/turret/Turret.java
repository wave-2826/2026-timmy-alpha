package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.Controls;
import frc.robot.commands.tuning.TurretTuning;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.subsystems.turret.TurretIO.TurretIOPIDOutputs;
import frc.robot.util.Container;
import frc.robot.util.tunables.LoggedTunableNumber;

/**
 * Our robot has a triple-coaxial turret - all motors are static relative to the robot frame.  
 * The power transmission stack is as follows:
 * - Flywheel: 2x Kraken X60, on the "innermost" coaxial stage; this will be affected by the azimuth rotation, but runs
 *   at a high velocity anyway so we don't care to compensate. The motors spin opposite, and the top needs to spin counterclockwise
 *   to shoot.
 * - Hood: 1x Kraken X60, on the "middle" coaxial stage. Must run with the azimuth rotation to maintain a consistent hood angle.
 * - Azimuth: 1x Kraken X60, on the "outermost" coaxial stage. Isn't affected by the other two stages and runs closed-loop over an
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

    public static enum ControlMode {
        NONE,
        PID,
        LQR
    }

    private final TurretIO io;

    private final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

    private ControlMode lastControlMode = null;
    private LoggedDashboardChooser<ControlMode> controlModeChooser = new LoggedDashboardChooser<>("Turret/ControlMode");

    public TurretTarget target = null;

    public Turret(TurretIO io) {
        this.io = io;

        controlModeChooser.addDefaultOption("PID", ControlMode.PID);
        controlModeChooser.addOption("LQR", ControlMode.LQR);

        TurretTuning.init();
    }

    @Override
    public void periodic() {
        var controlMode = controlModeChooser.get();
        if(controlMode != lastControlMode) {
            lastControlMode = controlMode;
            io.setControlMode(controlMode);
        }

        io.updateInputs(inputs);
        Logger.processInputs("Turret", (TurretIOInputsAutoLogged)inputs);

        if(target == null) {
            io.stop();

            Logger.recordOutput("Turret/Target/Azimuth", 0.0, Radians);
            Logger.recordOutput("Turret/Target/Hood", 0.0, Radians);
            Logger.recordOutput("Turret/Target/Flywheel", 0.0, RadiansPerSecond);
        } else {
            switch(controlMode) {
                case NONE:
                    return;
                case PID: {
                    TurretIOPIDOutputs outputs = new TurretIOPIDOutputs(
                        target.flywheelSpeedRadPerSec,
                        target.azimuthAngleRad % (Math.PI * 2),
                        MathUtil.clamp(
                            target.hoodAngleRad,
                            TurretConstants.hoodMinAngle,
                            TurretConstants.hoodMaxAngle
                        )
                    );
                    io.setPIDOutputs(outputs);
                    break;
                }
                case LQR: {
                    Logger.recordOutput("Turret/LQRKalman/azimuthPosition", inputs.LQRKalmanState[0]);
                    Logger.recordOutput("Turret/LQRKalman/azimuthVelocity", inputs.LQRKalmanState[1]);
                    Logger.recordOutput("Turret/LQRKalman/hoodPosition", inputs.LQRKalmanState[2]);
                    Logger.recordOutput("Turret/LQRKalman/hoodVelocity", inputs.LQRKalmanState[3]);
                    Logger.recordOutput("Turret/LQRKalman/flywheelVelocity", inputs.LQRKalmanState[4]);

                    // Managed in the IO layer
                    io.setTarget(target);            
                }
            }
            
            Logger.recordOutput("Turret/Target/Azimuth", target.azimuthAngleRad, Radians);
            Logger.recordOutput("Turret/Target/Hood", target.hoodAngleRad, Radians);
            Logger.recordOutput("Turret/Target/Flywheel", target.flywheelSpeedRadPerSec, RadiansPerSecond);
        }
        TurretVisualizer.getInstance().update(
            target != null ? target.azimuthAngleRad : 0.0, inputs.getAzimuthAngleRad(),
            target != null ? target.hoodAngleRad : 0.0, inputs.getHoodAngleRad()
        );

        Logger.recordOutput("Turret/Measured/FlywheelVelocity", inputs.getFlywheelVelocityRadPerSecond(), RadiansPerSecond);
        Logger.recordOutput("Turret/Measured/Hood", inputs.getHoodAngleRad(), Radians);
        Logger.recordOutput("Turret/Measured/HoodVelocity", inputs.getHoodVelocityRadPerSec(), RadiansPerSecond);
        Logger.recordOutput("Turret/Measured/Azimuth", inputs.getAzimuthAngleRad(), Radians);
        Logger.recordOutput("Turret/Measured/AzimuthVelocity", inputs.getAzimuthVelocityRadPerSec(), RadiansPerSecond);
    }

    public Command runManualVelocity(
        DoubleSupplier flywheelSpeedSupplier,
        DoubleSupplier azimuthSpeedSupplier,
        DoubleSupplier hoodSpeedSupplier
    ) {
        return Commands.runEnd(() -> {
            if(target == null) {
                target = new TurretTarget(0.0, inputs.getAzimuthAngleRad(), TurretConstants.hoodMinAngle);
            }

            target.flywheelSpeedRadPerSec = MathUtil.applyDeadband(flywheelSpeedSupplier.getAsDouble(), 0.2) * TurretConstants.maxFlywheelSpeedRadPerSec;
            
            target.azimuthAngleRad -= MathUtil.applyDeadband(azimuthSpeedSupplier.getAsDouble(), 0.2) * Math.PI * 0.02;
            target.azimuthAngleRad %= Math.PI * 2;

            target.hoodAngleRad -= hoodSpeedSupplier.getAsDouble() * Math.PI * 0.008;
            target.hoodAngleRad = MathUtil.clamp(target.hoodAngleRad, TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);
        }, () -> {
            target = null;
        }, this);
    }
    
    public static LoggedTunableNumber manualFlywheelSpeed = new LoggedTunableNumber("Turret/ManualFlywheelSpeed", 0.0);
    public static LoggedTunableNumber manualHoodOffset = new LoggedTunableNumber("Turret/ManualHoodAngleOffset", 0.0);
    public Command adjustManualVelocity(double change) {
        return Commands.runOnce(() -> manualFlywheelSpeed.set(manualFlywheelSpeed.get() + change));
    }
    public Command adjustManualAngle(double changeDegrees) {
        return Commands.runOnce(() -> manualHoodOffset.set(manualHoodOffset.get() + Units.degreesToRadians(changeDegrees)));
    }

    double manualControlAzimuthOffset = 0.0;

    public Command runManual(
        DoubleSupplier flywheelScalar,
        DoubleSupplier azimuthSpeed
    ) {
        SlewRateLimiter flyLimiter = new SlewRateLimiter(4000);
        return Commands.runEnd(() -> {
            if(target == null) {
                target = new TurretTarget(0.0, inputs.getAzimuthAngleRad(), TurretConstants.hoodMinAngle);
            }

            var parameters = ShotCalculator.getInstance().calculate();
            double calcRPM = Units.radiansPerSecondToRotationsPerMinute(parameters.target().flywheelSpeedRadPerSec);

            target.flywheelSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flyLimiter.calculate(
                flywheelScalar.getAsDouble() * (calcRPM + manualFlywheelSpeed.get())
            ));
            
            manualControlAzimuthOffset += MathUtil.applyDeadband(azimuthSpeed.getAsDouble(), 0.2) * Math.PI * 0.02;
            target.azimuthAngleRad = MathUtil.angleModulus(
                manualControlAzimuthOffset + parameters.target().azimuthAngleRad
            );

            // target.hoodAngleRad = TurretConstants.hoodMinAngle + manualHoodOffset.get();
            target.hoodAngleRad = parameters.target().hoodAngleRad + manualHoodOffset.get();
        }, () -> {
            target = null;
        }, this);
    }

    public Command reset() {
        return Commands.runOnce(() -> {
            io.resetAzimuth(TurretConstants.azimuthResetAngle);
            io.resetHoodToBottom();
            manualControlAzimuthOffset = 0.0;
        });
    }

    public Command runOscillationTest() {
        return Commands.runEnd(() -> {
            if(target == null) {
                target = new TurretTarget(0.0, inputs.getAzimuthAngleRad(), TurretConstants.hoodMinAngle);
            }

            target.flywheelSpeedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(2000);
            target.azimuthAngleRad = Math.sin(Timer.getFPGATimestamp() * 0.5) * Math.PI;
            target.hoodAngleRad = MathUtil.interpolate(
                TurretConstants.hoodMinAngle + Units.degreesToRadians(5),
                TurretConstants.hoodMaxAngle - Units.degreesToRadians(5),
                Math.sin(Timer.getFPGATimestamp() * 2) * 0.5 + 0.5
            );
        }, () -> {
            target = null;
        }, this);
    }

    @AutoLogOutput(key = "Turret/AtSetpoint")
    public boolean atSetpoint() {
        if(target == null) return true;

        double flywheelError = Math.abs(inputs.getFlywheelVelocityRadPerSecond() - target.flywheelSpeedRadPerSec);
        double azimuthError = Math.abs(MathUtil.angleModulus(inputs.getAzimuthAngleRad() - target.azimuthAngleRad));
        double hoodError = Math.abs(inputs.getHoodAngleRad() - target.hoodAngleRad);

        Logger.recordOutput("Turret/Errors/Flywheel", flywheelError, RadiansPerSecond);
        Logger.recordOutput("Turret/Errors/Azimuth", azimuthError, Radians);
        Logger.recordOutput("Turret/Errors/Hood", hoodError, Radians);

        return flywheelError < TurretConstants.flywheelToleranceRadPerSec
            && azimuthError < TurretConstants.azimuthToleranceRad
            && hoodError < TurretConstants.hoodToleranceRad;
    }

    public LinearVelocity getShotVelocity() {
        return MetersPerSecond.of(
            inputs.getFlywheelVelocityRadPerSecond() * TurretConstants.flywheelRadius *
                0.5 * // one fixed side
                0.1 // 10% of tangential velocity imparted
        );
    }
    public Angle getShotAngle() {
        return Radians.of(inputs.getHoodAngleRad() + Math.PI);
    }
    public Angle getRobotRelativeYaw() {
        return Radians.of(inputs.getAzimuthAngleRad());
    }

    public Command zeroRoutine() {
        Container<Boolean> startZeroValue = new Container<>(false);
        Container<Double> hoodStartPos = new Container<>(0.);
        double hoodRunVelocity = 3000;
        double hoodRangeRad = TurretConstants.hoodMaxAngle - TurretConstants.hoodMinAngle;
        return Commands.sequence(
            runOnce(() -> {
                startZeroValue.value = inputs.azimuthZeroTriggered;
                hoodStartPos.value = inputs.getHoodAngleRad();
            }),
            // Clockwise until the sensor value is the opposite of what it started
            run(() -> {
                target = null;
                io.setVelocityOutputs(0, Units.rotationsPerMinuteToRadiansPerSecond(50), hoodRunVelocity);
            }).until(() -> inputs.azimuthZeroTriggered != startZeroValue.value).withTimeout(2),
            // Counterclockwise until no longer triggered
            run(() -> {
                target = null;
                io.setVelocityOutputs(0, Units.rotationsPerMinuteToRadiansPerSecond(25), hoodRunVelocity);
            }).until(() -> !inputs.azimuthZeroTriggered).withTimeout(1),
            runOnce(() -> io.resetAzimuth(TurretConstants.azimuthResetAngle)),
            // Run until hood has changed by at least its full range
            run(() -> {
                io.setVelocityOutputs(0, 0, hoodRunVelocity);
            }).until(() -> Math.abs(inputs.getHoodAngleRad() - hoodStartPos.value) > hoodRangeRad).withTimeout(1),
            runOnce(() -> io.resetHoodToBottom())
        ).withName("TurretZero");
    }

    public Command runTuning() {
        TurretTuning tuning = new TurretTuning(io, () -> inputs, Controls.getInstance().coDriver);
        return runOnce(tuning::start).andThen(runEnd(tuning::run, tuning::stop));
    }
}