package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.ShotCalculator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.ShotCalculator.ShotType;

public class ScoringCommands {
    /**
     * @param turret
     * @param spindexer
     * @param driverShoot The trigger on the driver controller which indicates the driver's intent to shoot
     * @param codriverOverrideAxis The trigger on the co-driver controller which runs the spindexer
     * backward to "unstuck" balls in case something goes wrong.
     * @return
     */
    public static Command autoShoot(
        Turret turret,
        Spindexer spindexer,
        DoubleSupplier driverShoot,
        DoubleSupplier codriverOverrideAxis,
        BooleanSupplier codriverStop
    ) {
        return Commands.runEnd(() -> {
            var parameters = ShotCalculator.getInstance().calculate();
            double spinPower;
            if(parameters.shotType() != ShotType.NONE && driverShoot.getAsDouble() > 0.25) {
                turret.target = parameters.target();
                spinPower = turret.atSetpoint() ? 1.0 : 0.0;
            } else {
                spinPower = 0.0;
            }

            if(codriverStop.getAsBoolean()) {
                turret.target = null;
            }

            double codriverOverride = codriverOverrideAxis.getAsDouble();
            spindexer.setPower(Math.abs(codriverOverride) > 0.1 ? codriverOverride : spinPower, 1.0);
        }, () -> {
            turret.target = null;
            spindexer.setPower(0.0, 0.0);
        }, turret, spindexer);
    }
}
