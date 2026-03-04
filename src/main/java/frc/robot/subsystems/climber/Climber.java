package frc.robot.subsystems.climber;

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

    public Command extendBoth() {
        return runEnd(() -> {
            runLeftPercent(50);
            runRightPercent(50);
        }, () -> {
            runLeftPercent(0);
            runRightPercent(0);
        }).until(() -> ((inputs.left.motorPosition() + inputs.right.motorPosition()) / 2 >= Units.inchesToMeters(25)));
    }

    public Command retractBoth() {
        return runEnd(() -> {
            runLeftPercent(-50);
            runRightPercent(-50);
        }, () -> {
            runLeftPercent(0);
            runRightPercent(0);
        }).until(() -> ((inputs.left.motorPosition() + inputs.right.motorPosition()) / 2 <= Units.inchesToMeters(15)));
    }

    /** Extend to a ceratin length in millimeters */
    public Command extendLeftServo(int length) {
        return runEnd(() -> {
            if (!(length > ClimberConstants.servoLengthmm)) {
                io.setLeftServoPosition(length/ClimberConstants.servoLengthmm);
            }
        }, null);
    }
    /** Extend to a ceratin length in millimeters */
    public Command extendRightServo(int length) {
        return runEnd(() -> {
            if (!(length > ClimberConstants.servoLengthmm)) {
                io.setRightServoPosition(length/ClimberConstants.servoLengthmm);
            }
        }, null);
    }
    /** Extend to a ceratin length in millimeters */
    public Command extendBothServos(int length) {
        return runEnd(() -> {
            extendLeftServo(length);
            extendRightServo(length);
        }, null);
    }

}