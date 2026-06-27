package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopperVision.HopperVision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.LEDState;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.ShotCalculator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.ShotCalculator.ShotType;
import frc.robot.subsystems.turret.Turret.ControlTarget;
import frc.robot.util.Container;

public class ScoringCommands {

    public static Command prep(Turret turret) {
        return turret.runOnce(() -> {
            turret.target = ControlTarget.SHOT_CALCULATOR_DEFAULT;
        });
    }

    public static Command autoScoreHopper(Turret turret, Spindexer spindexer, Intake intake, HopperVision hopperVision, LEDs leds) {
        return autoScoreHopper(turret, spindexer, intake, hopperVision, leds, true);
    }
    public static Command autoScoreHopper(
        Turret turret, Spindexer spindexer, Intake intake, HopperVision hopperVision, LEDs leds,
        boolean stopWhenHopperEmpty
    ) {
        return Commands.sequence(
            Commands.waitUntil(turret::atSetpoint).withTimeout(3.0),
            
            // Run spin until no pieces remain
            spindexer.run(() -> {
                spindexer.setPower(1.0, turret.atSetpoint() ? 1.0 : 0.0);
            }).alongWith(
                intake.setIntakePositionNormalized(() -> Math.sin(Timer.getFPGATimestamp() * 1.5) * 0.5 + 0.5)
            ).alongWith(leds.runStateCommand(LEDState.Scoring))
            .raceWith(stopWhenHopperEmpty ? hopperVision.waitForFewerThanNPieces(2, 1.0, 8.0, 10.0) : Commands.idle())
        ).raceWith(
            turret.run(() -> { turret.target = ControlTarget.SHOT_CALCULATOR_DEFAULT; })
        ).andThen(
            // Reset turret/spin
            spindexer.run(() -> {
                turret.target = ControlTarget.NONE;
                spindexer.setPower(0.0, 0.0);
            }).until(() -> {
                return true;
            })
        ).withName("AutoScoreHopper");
    }

    // /**
    //  * @param turret
    //  * @param spindexer
    //  * @param driverShoot The trigger on the driver controller which indicates the driver's intent to shoot
    //  * @param codriverOverrideAxis The trigger on the co-driver controller which runs the spindexer
    //  * backward to "unstuck" balls in case something goes wrong.
    //  * @return
    //  */
    // public static Command teleopScoring(
    //     Turret turret,
    //     Spindexer spindexer,
    //     DoubleSupplier driverShoot,
    //     DoubleSupplier codriverOverrideAxis,
    //     BooleanSupplier codriverStop
    // ) {
    //     return Commands.runEnd(() -> {
    //         var parameters = ShotCalculator.getInstance().getLatestResult();
    //         double spinPower;
    //         if(parameters.shotType() != ShotType.NONE && driverShoot.getAsDouble() > 0.25) {
    //             turret.target = ControlTarget.SHOT_CALCULATOR;
    //             spinPower = turret.atSetpoint() ? 1.0 : 0.0;
    //         } else {
    //             spinPower = 0.0;
    //         }

    //         if(codriverStop.getAsBoolean()) {
    //             turret.target = ControlTarget.NONE;
    //         }

    //         double codriverOverride = codriverOverrideAxis.getAsDouble();
    //         spindexer.setPower(Math.abs(codriverOverride) > 0.1 ? codriverOverride : spinPower, 1.0);
    //     }, () -> {
    //         turret.target = ControlTarget.NONE;
    //         spindexer.setPower(0.0, 0.0);
    //     }, turret, spindexer).withName("TeleopScoring");
    // }

    public static Command teleopOverrideScoring(
        DoubleSupplier flywheelScalar,
        DoubleSupplier azimuthSpeed,
        BooleanSupplier hoodDown,
        Turret turret,
        LEDs leds
    ) {
        SlewRateLimiter flyLimiter = new SlewRateLimiter(5000);
        Container<Boolean> flyRampEdgeActive = new Container<>(false);
        return Commands.runEnd(() -> {
            turret.target = ControlTarget.SHOT_CALCULATOR;
            
            Turret.manualAzimuthOffset.set(Turret.manualAzimuthOffset.get() - MathUtil.applyDeadband(azimuthSpeed.getAsDouble(), 0.2) * Math.PI * 0.003);

            ControlTarget.ShotCalculator shotTarget = (ControlTarget.ShotCalculator)turret.target;

            double flySpeed = MathUtil.applyDeadband(flywheelScalar.getAsDouble(), 0.15);
            boolean runFly = flySpeed > 0.;
            if(flyRampEdgeActive.value != runFly) flyLimiter.reset(
                Units.radiansPerSecondToRotationsPerMinute(turret.getFlywheelVelocityRadPerSecond())
            );
            flyRampEdgeActive.value = runFly;
            shotTarget.maxFlyVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(flyLimiter.calculate(runFly ? 6000 : 0.));
            shotTarget.flyOffsetRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                Turret.manualFlywheelSpeed.get() - (1.0 - flySpeed) * 200
            );

            leds.setStateActive(LEDState.ScoringRecovering, runFly && !turret.atSetpoint());
            leds.setStateActive(LEDState.Scoring, runFly);

            shotTarget.azimuthOffsetRad = Turret.manualAzimuthOffset.get();
            shotTarget.hoodOffsetRad = Turret.manualHoodOffset.get();

            shotTarget.maxHoodAngleRad = hoodDown.getAsBoolean() ? TurretConstants.maxTrenchHoodAngle : 0;
        }, () -> {
            turret.target = ControlTarget.NONE;
            
            leds.setStateActive(LEDState.ScoringRecovering, false);
            leds.setStateActive(LEDState.Scoring, false);
        }, turret).withName("TeleopOverrideScoring");
    }
}
