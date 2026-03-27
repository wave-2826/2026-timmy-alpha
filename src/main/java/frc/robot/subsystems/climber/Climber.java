package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    private Command runRightPercent(double percent) {
        return Commands.runEnd(() -> io.setRightPower(percent), () -> io.setRightPower(0.0));
    }

    private Command runLeftPercent(double percent) {
        return Commands.runEnd(() -> io.setLeftPower(percent), () -> io.setLeftPower(0.0));
    }

    public Command extendBoth() {
        return Commands.parallel(
             runLeftPercent(50).until(() ->  inputs.left.position() >= Units.inchesToMeters(25)),
            runRightPercent(50).until(() -> inputs.right.position() >= Units.inchesToMeters(25))
        );
    }

    public Command retractBoth() {
        return Commands.parallel(
             runLeftPercent(-50).until(() ->  inputs.left.position() <= Units.inchesToMeters(15)),
            runRightPercent(-50).until(() -> inputs.right.position() <= Units.inchesToMeters(15))
        );
    }

    public Command manualControls(DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        return Commands.runOnce(() -> {
            io.setLeftServoPosition(2.5 / 4);
            io.setRightServoPosition(2.5 / 4);
        }).andThen(Commands.runEnd(() -> {
            // Flipped since it should be relative to driver
            double leftPercent = MathUtil.applyDeadband(rightSupplier.getAsDouble(), 0.3);
            double rightPercent = MathUtil.applyDeadband(leftSupplier.getAsDouble() + rightSupplier.getAsDouble(), 0.3);
            io.setLeftPower(leftPercent * 0.6);
            io.setRightPower(rightPercent * 0.6);
        }, () -> {
            io.setLeftPower(0.0);
            io.setRightPower(0.0);
            io.setLeftServoPosition(0.0);
            io.setRightServoPosition(0.0);
        }));
    }

    public Command extendLeftServo(double position) {
        return Commands.run(() -> {
            io.setLeftServoPosition(position);
        }).until(() -> MathUtil.isNear(position, inputs.leftServeoPosition, 0.05));
    }
    public Command extendRightServo(double position) {
        return Commands.run(() -> {
            io.setRightServoPosition(position);
        }).until(() -> MathUtil.isNear(position, inputs.rightServeoPosition, 0.05));
    }
    public Command extendServos() {
        return extendLeftServo(0.5).alongWith(extendRightServo(0.5));
    }
}