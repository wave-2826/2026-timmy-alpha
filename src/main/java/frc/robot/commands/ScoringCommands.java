package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.turret.ShotCalculator.ShotType;
import frc.robot.subsystems.turret.Turret.ControlTarget;

public class ScoringCommands {
    /**
     * @param turret
     * @param spindexer
     * @param driverShoot The trigger on the driver controller which indicates the driver's intent to shoot
     * @param codriverOverrideAxis The trigger on the co-driver controller which runs the spindexer
     * backward to "unstuck" balls in case something goes wrong.
     * @return
     */
    public static Command teleopScoring(
        Turret turret,
        Spindexer spindexer,
        DoubleSupplier driverShoot,
        DoubleSupplier codriverOverrideAxis,
        BooleanSupplier codriverStop
    ) {
        return Commands.runEnd(() -> {
            var parameters = ShotCalculator.getInstance().getLatestResult();
            double spinPower;
            if(parameters.shotType() != ShotType.NONE && driverShoot.getAsDouble() > 0.25) {
                turret.target = ControlTarget.SHOT_CALCULATOR;
                spinPower = turret.atSetpoint() ? 1.0 : 0.0;
            } else {
                spinPower = 0.0;
            }

            if(codriverStop.getAsBoolean()) {
                turret.target = ControlTarget.NONE;
            }

            double codriverOverride = codriverOverrideAxis.getAsDouble();
            spindexer.setPower(Math.abs(codriverOverride) > 0.1 ? codriverOverride : spinPower, 1.0);
        }, () -> {
            turret.target = ControlTarget.NONE;
            spindexer.setPower(0.0, 0.0);
        }, turret, spindexer).withName("TeleopScoring");
    }

    public static Command prep(Turret turret) {
        return turret.runOnce(() -> {
            turret.target = ControlTarget.SHOT_CALCULATOR_DEFAULT;
        });
    }

    public static Command autoScoreHopper(Turret turret, Spindexer spindexer, Intake intake, HopperVision hopperVision, LEDs leds) {
        return Commands.sequence(
            Commands.waitUntil(turret::atSetpoint).withTimeout(3.0),
            
            // Run spin until no pieces remain
            spindexer.run(() -> {
                spindexer.setPower(1.0, turret.atSetpoint() ? 1.0 : 0.0);
            }).alongWith(
                intake.setIntakePositionNormalized(() -> Math.sin(Timer.getFPGATimestamp() * 0.3) * 0.4 + 0.5)
            ).raceWith(hopperVision.waitForNoPieces(1.0, 8.0, 10.0)).alongWith(leds.runStateCommand(LEDState.Scoring))
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
}
