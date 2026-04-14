package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

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
        /** The target velocity of the flywheel itself in radians/second */
        public double flywheelSpeedRadPerSec;
        /** The target angle of the azimuth relative to the robot base in radians. */
        public double azimuthAngleRad;
        /** The current feedforward of the azimuth in rad/sec^2. */
        public double azimuthFeedforwardRadPerSec;
        /**
         * The target hood angle relative to the turret surface; 0 rad would be shooting
         * straight up and pi/2 rad would theoretically be shots directly outward.
         */
        public double hoodAngleRad;

        public TurretTarget(double flywheelSpeedRadPerSec, double azimuthAngleRad, double hoodAngleRad) {
            this.flywheelSpeedRadPerSec = flywheelSpeedRadPerSec;
            this.azimuthAngleRad = azimuthAngleRad;
            this.azimuthFeedforwardRadPerSec = 0;
            this.hoodAngleRad = hoodAngleRad;
        }

        public TurretTarget(double flywheelSpeedRadPerSec, double azimuthAngleRad, double azimuthFeedforwardRadPerSec, double hoodAngleRad) {
            this.flywheelSpeedRadPerSec = flywheelSpeedRadPerSec;
            this.azimuthAngleRad = azimuthAngleRad;
            this.azimuthFeedforwardRadPerSec = azimuthFeedforwardRadPerSec;
            this.hoodAngleRad = hoodAngleRad;
        }
    }

    public static enum ControlMode {
        NONE,
        PID,
        LQR
    }

    public static interface ControlTarget {
        public static ControlTarget NONE = new ControlTarget.None();
        public static ControlTarget SHOT_CALCULATOR_DEFAULT = new ControlTarget.ShotCalculator();
        public static ControlTarget SHOT_CALCULATOR = new ControlTarget.ShotCalculator();
        
        public static class None implements ControlTarget {};
        public static class Manual implements ControlTarget {
            public TurretTarget target;
            public Manual(TurretTarget target) { this.target = target; }
        };
        public static class ShotCalculator implements ControlTarget {
            public double maxFlyVelocityRadPerSec = 6000;
            public double azimuthOffsetRad = 0;
            public double hoodOffsetRad = 0;
            public double flyOffsetRadPerSec = 0;
        }
    }

    private final TurretIO io;

    private final TurretIOInputs inputs = new TurretIOInputsAutoLogged();

    private ControlMode lastControlMode = null;
    private LoggedDashboardChooser<ControlMode> controlModeChooser = new LoggedDashboardChooser<>("Turret/ControlMode");

    public ControlTarget target = ControlTarget.NONE;

    private final Alert flywheel1DisconnectedAlert = new Alert("Flywheel motor 1 disconnected! Using only 2 as a fallback. Recovery performance will be reduced.", AlertType.kError);
    private final Alert flywheel2DisconnectedAlert = new Alert("Flywheel motor 2 disconnected! Recovery performance will be reduced.", AlertType.kError);
    private final Alert azimuthDisconnectedAlert = new Alert("Turret azimuth motor disconnected!", AlertType.kError);
    private final Alert hoodDisconnectedAlert = new Alert("Turret hood motor disconnected!", AlertType.kError);

    public Turret(TurretIO io) {
        this.io = io;

        controlModeChooser.addDefaultOption("PID", ControlMode.PID);
        controlModeChooser.addOption("LQR", ControlMode.LQR);

        TurretTuning.init();

        RobotModeTriggers.autonomous().onTrue(reset());
    }

    private boolean zeroing = false;

    private boolean atSetpoint = false;

    @Override
    public void periodic() {
        var controlMode = controlModeChooser.get();
        if(controlMode != lastControlMode) {
            lastControlMode = controlMode;
            io.setControlMode(controlMode);
        }

        io.updateInputs(inputs);
        Logger.processInputs("Turret", (TurretIOInputsAutoLogged)inputs);

        // Clamp hood if in an unreasonable position
        if(!zeroing && inputs.getHoodAngleRad() > TurretConstants.hoodMaxAngle + Units.degreesToRadians(0.5)) {
            // Hood will be mechanically limited in range but the motor can keep spinning; clamp so our
            // understanding of the hood position is at least close
            io.resetHoodTo(TurretConstants.hoodMaxAngle);
        }

        flywheel1DisconnectedAlert.set(!inputs.flywheel1.connected());
        flywheel2DisconnectedAlert.set(!inputs.flywheel2.connected());
        azimuthDisconnectedAlert.set(!inputs.azimuth.connected());
        hoodDisconnectedAlert.set(!inputs.hood.connected());

        if(target == null || target instanceof ControlTarget.None) {
            atSetpoint = true;

            io.stop();

            Logger.recordOutput("Turret/Target/Azimuth", 0.0, Radians);
            Logger.recordOutput("Turret/Target/AzimuthVel", 0.0, RadiansPerSecond);
            Logger.recordOutput("Turret/Target/Hood", 0.0, Radians);
            Logger.recordOutput("Turret/Target/Flywheel", 0.0, RadiansPerSecond);
        } else {
            TurretTarget calculatedTarget = null;
            if(target instanceof ControlTarget.Manual) {
                calculatedTarget = ((ControlTarget.Manual)target).target;
            } else if(target instanceof ControlTarget.ShotCalculator) {
                ControlTarget.ShotCalculator shotTarget = (ControlTarget.ShotCalculator)target;
                calculatedTarget = ShotCalculator.getInstance().calculate().target();
                calculatedTarget.flywheelSpeedRadPerSec += shotTarget.flyOffsetRadPerSec;
                calculatedTarget.azimuthAngleRad += shotTarget.azimuthOffsetRad;
                calculatedTarget.hoodAngleRad += shotTarget.hoodOffsetRad;
                calculatedTarget.flywheelSpeedRadPerSec = Math.min(shotTarget.maxFlyVelocityRadPerSec, calculatedTarget.flywheelSpeedRadPerSec);
            }

            if(calculatedTarget == null) return;

            // Setpoints
            double flywheelError = Math.abs(inputs.getFlywheelVelocityRadPerSecond() - calculatedTarget.flywheelSpeedRadPerSec);
            double azimuthError = Math.abs(MathUtil.angleModulus(inputs.getAzimuthAngleRad() - calculatedTarget.azimuthAngleRad));
            double hoodError = Math.abs(inputs.getHoodAngleRad() - calculatedTarget.hoodAngleRad);
    
            Logger.recordOutput("Turret/Errors/Flywheel", flywheelError, RadiansPerSecond);
            Logger.recordOutput("Turret/Errors/Azimuth", azimuthError, Radians);
            Logger.recordOutput("Turret/Errors/Hood", hoodError, Radians);
    
            boolean hoodAzimuthAtSetpoint = azimuthError < TurretConstants.azimuthToleranceRad && hoodError < TurretConstants.hoodToleranceRad;
            boolean withinEnterSetpoint = flywheelError < TurretConstants.flywheelToleranceRadPerSecEnter && hoodAzimuthAtSetpoint;
            boolean withinExitSetpoint = flywheelError < TurretConstants.flywheelToleranceRadPerSecExit && hoodAzimuthAtSetpoint;
    
            if(atSetpoint && !withinExitSetpoint) {
                atSetpoint = false;
            } else if(!atSetpoint && withinEnterSetpoint) {
                atSetpoint = true;
            }

            switch(controlMode) {
                case NONE:
                    return;
                case PID: {
                    // TODO: limit fly target slew rate right here, mayhaps?
                    TurretIOPIDOutputs outputs = new TurretIOPIDOutputs(
                        MathUtil.clamp(
                            calculatedTarget.flywheelSpeedRadPerSec,
                            DriverStation.isFMSAttached() ? Units.rotationsPerMinuteToRadiansPerSecond(2000) : 0.,
                            Units.rotationsPerMinuteToRadiansPerSecond(5500)
                        ),
                        calculatedTarget.azimuthAngleRad % (Math.PI * 2),
                        calculatedTarget.azimuthFeedforwardRadPerSec,
                        MathUtil.clamp(
                            calculatedTarget.hoodAngleRad,
                            TurretConstants.hoodMinAngle,
                            TurretConstants.hoodMaxAngle - Units.degreesToRadians(0.5)
                        ) - TurretConstants.hoodMinAngle
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
                    io.setTarget(calculatedTarget);            
                }
            }
            
            Logger.recordOutput("Turret/Target/Azimuth", calculatedTarget.azimuthAngleRad, Radians);
            Logger.recordOutput("Turret/Target/AzimuthVel", calculatedTarget.azimuthFeedforwardRadPerSec, RadiansPerSecond);
            Logger.recordOutput("Turret/Target/Hood", calculatedTarget.hoodAngleRad, Radians);
            Logger.recordOutput("Turret/Target/Flywheel", calculatedTarget.flywheelSpeedRadPerSec, RadiansPerSecond);
            
            TurretVisualizer.getInstance().update(
                calculatedTarget.azimuthAngleRad, inputs.getAzimuthAngleRad(),
                calculatedTarget.hoodAngleRad, inputs.getHoodAngleRad()
            );
        }
        Logger.recordOutput("Turret/AtSetpoint", atSetpoint);

        Logger.recordOutput("Turret/Measured/FlywheelVelocity", inputs.getFlywheelVelocityRadPerSecond(), RadiansPerSecond);
        Logger.recordOutput("Turret/Measured/Hood", inputs.getHoodAngleRad(), Radians);
        Logger.recordOutput("Turret/Measured/HoodVelocity", inputs.getHoodVelocityRadPerSec(), RadiansPerSecond);
        Logger.recordOutput("Turret/Measured/Azimuth", inputs.getAzimuthAngleRad(), Radians);
        Logger.recordOutput("Turret/Measured/AzimuthVelocity", inputs.getAzimuthVelocityRadPerSec(), RadiansPerSecond);
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
        SlewRateLimiter flyLimiter = new SlewRateLimiter(5000);
        return Commands.runEnd(() -> {
            target = ControlTarget.SHOT_CALCULATOR;
            
            manualControlAzimuthOffset -= MathUtil.applyDeadband(azimuthSpeed.getAsDouble(), 0.2) * Math.PI * 0.005;
            Logger.recordOutput("Turret/ManualControlAzimuthOffsetDeg", Units.radiansToDegrees(manualControlAzimuthOffset));

            ControlTarget.ShotCalculator shotTarget = (ControlTarget.ShotCalculator)target;
            // TODO: better limiting logic
            shotTarget.maxFlyVelocityRadPerSec = Units.radiansPerSecondToRotationsPerMinute(flyLimiter.calculate(
                flywheelScalar.getAsDouble() * 6500
            ));
            shotTarget.azimuthOffsetRad = manualControlAzimuthOffset;
            shotTarget.flyOffsetRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(manualFlywheelSpeed.get());
            shotTarget.hoodOffsetRad = manualHoodOffset.get();
        }, () -> {
            target = ControlTarget.NONE;
        }, this);
    }

    public Command reset() {
        return Commands.runOnce(() -> {
            io.resetAzimuth(TurretConstants.azimuthResetAngle);
            io.resetHoodTo(TurretConstants.hoodMaxAngle);
            manualControlAzimuthOffset = 0.0;
            manualHoodOffset.set(0.0);
        });
    }

    public Command runOscillationTest() {
        return Commands.runEnd(() -> {
            target = new ControlTarget.Manual(new TurretTarget(
                Units.rotationsPerMinuteToRadiansPerSecond(2000),
                Math.sin(Timer.getFPGATimestamp() * 0.5) * Math.PI,
                MathUtil.interpolate(
                    TurretConstants.hoodMinAngle + Units.degreesToRadians(5),
                    TurretConstants.hoodMaxAngle - Units.degreesToRadians(5),
                    Math.sin(Timer.getFPGATimestamp() * 2) * 0.5 + 0.5
                )
            ));
        }, () -> {
            target = null;
        }, this);
    }

    public boolean atSetpoint() {
        return atSetpoint;
    }

    public LinearVelocity getShotVelocity() {
        return MetersPerSecond.of(
            inputs.getFlywheelVelocityRadPerSecond() * TurretConstants.flywheelRadius
                * 0.5 // one fixed side
                / 1.2
        );
    }
    public Angle getShotAngle() {
        return Radians.of(Math.PI / 2 - inputs.getHoodAngleRad() + Units.degreesToRadians(1));
    }
    public Angle getRobotRelativeYaw() {
        return Radians.of(inputs.getAzimuthAngleRad());
    }

    private boolean hasZeroed = false;

    public Command zeroRoutine() {        
        Container<Double> azimuthVelocity = new Container<>(0.);
        Container<Double> hoodVelocity = new Container<>(0.);

        return Commands.sequence(
            Commands.runOnce(() -> {
                zeroing = true;
            }),

            Commands.parallel(
                zeroAzimuth((v) -> azimuthVelocity.value = v),
                zeroHood((v) -> hoodVelocity.value = v)
            ).raceWith(run(() -> {
                target = null;
                Logger.recordOutput("Turret/Reset/Azimuth", azimuthVelocity.value);
                Logger.recordOutput("Turret/Reset/Hood", hoodVelocity.value);
                io.setVelocityOutputs(0, azimuthVelocity.value, hoodVelocity.value);
            })),

            runOnce(() -> {
                io.resetHoodTo(TurretConstants.hoodMaxAngle);
                manualControlAzimuthOffset = 0;
                manualHoodOffset.set(0.0);
                hasZeroed = true;
            })
        ).finallyDo(() -> {
            zeroing = false;
        }).withName("TurretZero");
    }

    private Command zeroHood(DoubleConsumer setHoodVelocity) {
        LinearFilter hoodCurrentFilter = LinearFilter.movingAverage((int)(0.3 / 0.02));
        double hoodRunVelocity = Units.rotationsPerMinuteToRadiansPerSecond(-400);

        Container<Double> hoodStartPos = new Container<>(0.);
        double zeroRangeRad = TurretConstants.hoodMaxAngle - TurretConstants.hoodMinAngle;
        
        // Run velocity until EITHER the hood has gone its full range or it hits the zero current
        return Commands.sequence(
            Commands.runOnce(() -> {
                hoodStartPos.value = inputs.getHoodAngleRad();
                hoodCurrentFilter.reset();
            }),
            Commands.run(() -> {
                setHoodVelocity.accept(hoodRunVelocity);
            }).until(() -> {
                return Math.abs(inputs.getHoodAngleRad() - hoodStartPos.value) > zeroRangeRad ||
                    hoodCurrentFilter.calculate(inputs.hood.currentAmps()) > TurretConstants.hoodResetCurrent;
            }),
            Commands.runOnce(() -> {
                setHoodVelocity.accept(0);
            }),
            // Wait for the hood motor velocity to be zero so we don't zero before we stop moving up
            Commands.waitUntil(() -> Math.abs(inputs.hood.velocityRadPerSec() / TurretConstants.hoodMotorToRingReduction) < Units.degreesToRadians(10))
        );
    }

    private Command zeroAzimuth(DoubleConsumer setAzimuthVelocity) {
        Container<Boolean> previousZeroTriggered = new Container<>(false);
        double triggerSpeed = Units.rotationsPerMinuteToRadiansPerSecond(15);
        double detriggerSpeed = Units.rotationsPerMinuteToRadiansPerSecond(-4);
        Debouncer negativeVelocityDebouncer = new Debouncer(0.3, DebounceType.kFalling);
        return Commands.sequence(
            Commands.runOnce(() -> {
                previousZeroTriggered.value = inputs.azimuthZeroTriggered;
            }),

            // Clockwise until the sensor value is the opposite of what it started
            Commands.run(() -> {
                target = null;
                setAzimuthVelocity.accept(triggerSpeed);
                previousZeroTriggered.value = inputs.azimuthZeroTriggered;
            }).until(() -> inputs.azimuthZeroTriggered)
                .withTimeout(6)
                .onlyIf(() -> !inputs.azimuthZeroTriggered),
            
            // Counterclockwise until falling edge while also traveling in correct direction
            Commands.run(() -> {
                target = null;
                setAzimuthVelocity.accept(detriggerSpeed);
            }).until(() -> {
                boolean fallingEdge = previousZeroTriggered.value && !inputs.azimuthZeroTriggered;
                boolean negativeVelocity = negativeVelocityDebouncer.calculate(inputs.azimuth.internalEncoderVelocity() < detriggerSpeed / 3.);
                previousZeroTriggered.value = inputs.azimuthZeroTriggered;
                return fallingEdge && negativeVelocity;
            })
                .withTimeout(1),
            
            Commands.runOnce(() -> {
                io.resetAzimuth(TurretConstants.azimuthResetAngle);
                setAzimuthVelocity.accept(0.);
            })
        );
    }

    public boolean zeroed() {
        return hasZeroed;
    }

    public Command runTuning() {
        TurretTuning tuning = new TurretTuning(io, () -> inputs, Controls.getInstance().coDriver);
        return runOnce(tuning::start).andThen(runEnd(tuning::run, tuning::stop));
    }
}