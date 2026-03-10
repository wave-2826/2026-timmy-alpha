package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Command runRightPercent(double percent) {
        return runEnd(() -> io.setRightPower(percent * 12.0), () -> io.setRightPower(0.0));
    }

    public Command runLeftPercent(double percent) {
        return runEnd(() -> io.setLeftPower(percent * 12.0), () -> io.setLeftPower(0.0));
    }

    // TODO: run separately? not sure why we would average here
    
    public Command extendBoth() {
        return runEnd(() -> {
            runLeftPercent(50);
            runRightPercent(50);
        }, () -> {
            runLeftPercent(0);
            runRightPercent(0);
        }).until(() -> ((inputs.left.position() + inputs.right.position()) / 2 >= Units.inchesToMeters(25)));
    }

    public Command retractBoth() {
        return runEnd(() -> {
            runLeftPercent(-50);
            runRightPercent(-50);
        }, () -> {
            runLeftPercent(0);
            runRightPercent(0);
        }).until(() -> ((inputs.left.position() + inputs.right.position()) / 2 <= Units.inchesToMeters(15)));
    }

    public Command extendLeftServo(double position) {
        return run(() -> {
            io.setLeftServoPosition(position);
        }).until(() -> MathUtil.isNear(position, inputs.leftServeoPosition, 0.05));
    }
    public Command extendRightServo(double position) {
        return run(() -> {
            io.setRightServoPosition(position);
        }).until(() -> MathUtil.isNear(position, inputs.rightServeoPosition, 0.05));
    }
    public Command extendServos() {
        return extendLeftServo(0.5).alongWith(extendRightServo(0.5));
    }
}