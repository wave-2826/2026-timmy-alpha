package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tunables.LoggedTunableNumber;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private static final LoggedTunableNumber intakeRollerPercent = new LoggedTunableNumber("Intake/RollerPercent", 0.5);
    
    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        
    }

    public Command enable() {
        return runRollerPercent(intakeRollerPercent.get());
    }

    public Command disable() {
        return runRollerPercent(0);
    }
    
    private Command runRollerPercent(double percent) {
        return runOnce(() -> io.setRollerVoltage(percent/100 * 12.0));
    }
    
    public Command runRollerTeleop(DoubleSupplier forward, DoubleSupplier reverse) {
        return runEnd(
            () -> io.setRollerVoltage((forward.getAsDouble() - reverse.getAsDouble()) * 12.0),
            () -> io.setRollerVoltage(0.0)
        );
    }

    private LinearFilter deployLCurrentFilter = LinearFilter.movingAverage(5);
    private LinearFilter deployRCurrentFilter = LinearFilter.movingAverage(5);

    public boolean isDeployed() {
        return Math.abs((inputs.deployL.motorPosition() + inputs.deployR.motorPosition()) / 2) < 0.1;
    }
    
    public Command deployIntake() {
        final double deployVoltage = 5+.0;
        return Commands.parallel(
            Commands.runEnd(() -> io.setDeployVoltageL(-deployVoltage), () -> io.setDeployVoltageL(0.0))
                .until(() -> deployLCurrentFilter.calculate(inputs.deployL.currentAmps()) > IntakeConstants.deployStallCurrent),
            Commands.runEnd(() -> io.setDeployVoltageR(-deployVoltage), () -> io.setDeployVoltageR(0.0))
                .until(() -> deployRCurrentFilter.calculate(inputs.deployR.currentAmps()) > IntakeConstants.deployStallCurrent)
        ).withTimeout(1.0).andThen(() -> {
            io.resetDeployEncoders();
        });
    }
    
    public Command setIntakePosition(DoubleSupplier position) {
        return run(() -> {
            io.setDeployPosition(-position.getAsDouble());
        });
    }
    
    public Command setIntakePositionNormalized(DoubleSupplier triggerPosition) {
        return setIntakePosition(() -> {
            return IntakeConstants.trackLengthMeters * triggerPosition.getAsDouble();
        });
    }
}