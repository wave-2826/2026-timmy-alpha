package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tunables.LoggedTunableNumber;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private static final LoggedTunableNumber intakeRollerSpeed = new LoggedTunableNumber("Intake/RollerSpeed", 4000);
    
    private final Alert leftDeployDisconnectedAlert = new Alert("Left intake deploy motor disconnected!", AlertType.kError);
    private final Alert rightDeployDisconnectedAlert = new Alert("Right intake deploy motor disconnected!", AlertType.kError);
    private final Alert leftRollerDisconnectedAlert = new Alert("Left intake roller motor disconnected!", AlertType.kError);
    private final Alert rightRollerDisconnectedAlert = new Alert("Right intake roller motor disconnected!", AlertType.kError);

    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        
        leftDeployDisconnectedAlert.set(!inputs.deployL.connected());
        rightDeployDisconnectedAlert.set(!inputs.deployR.connected());
        leftRollerDisconnectedAlert.set(!inputs.rollerL.connected());
        rightRollerDisconnectedAlert.set(!inputs.rollerR.connected());
    }

    public Command enable() {
        return runRollerScaledOnce(1);
    }

    public Command enableOutward() {
        return runRollerScaledOnce(-1);
    }

    public Command disable() {
        return runRollerScaledOnce(0);
    }
    
    public Command runRollerScaledOnce(double percent) {
        return runOnce(() -> io.setRollerSpeed(percent * intakeRollerSpeed.get()));
    }
    
    public Command runRollerScaled(DoubleSupplier percent) {
        return Commands.run(() -> io.setRollerSpeed(percent.getAsDouble() * intakeRollerSpeed.get()));
    }

    private LinearFilter deployLCurrentFilter = LinearFilter.movingAverage(5);
    private LinearFilter deployRCurrentFilter = LinearFilter.movingAverage(5);

    public boolean isDeployed() {
        return Math.abs((inputs.deployL.motorPosition() + inputs.deployR.motorPosition()) / 2) < 0.1;
    }
    
    public Command deployIntake() {
        final double deployPower = 0.6;
        return Commands.parallel(
            Commands.runEnd(() -> io.setDeployPowerL(deployPower), () -> io.setDeployPowerL(0.0))
                .until(() -> deployLCurrentFilter.calculate(inputs.deployL.currentAmps()) > IntakeConstants.deployStallCurrent),
            Commands.runEnd(() -> io.setDeployPowerR(deployPower), () -> io.setDeployPowerR(0.0))
                .until(() -> deployRCurrentFilter.calculate(inputs.deployR.currentAmps()) > IntakeConstants.deployStallCurrent)
        ).withTimeout(1.0).andThen(() -> {
            io.resetDeployEncoders();
        });
    }
    
    /** Set the intake position. Positive numbers are inward. */
    public Command setIntakePosition(DoubleSupplier position) {
        return run(() -> {
            io.setDeployPosition(position.getAsDouble());
        });
    }
    
    public Command setIntakePositionNormalized(DoubleSupplier triggerPosition) {
        return setIntakePosition(() -> {
            return IntakeConstants.trackLengthMeters * triggerPosition.getAsDouble();
        });
    }
}