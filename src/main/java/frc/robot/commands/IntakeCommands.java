package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    double rollerSpeed = 0;

    public Command overrideIn() {
        return Commands.runOnce(() -> {
            rollerSpeed = 1;
        });
    }
    
    public Command overrideOut() {
        return Commands.runOnce(() -> {
            rollerSpeed = -1;
        });
    }
    
    public Command overrideOff() {
        return Commands.runOnce(() -> {
            rollerSpeed = 0;
        });
    }

    public Command run(Intake intake, DoubleSupplier pullIn, BooleanSupplier runRoller) {
        return Commands.parallel(
            intake.runRollerScaled(() -> rollerSpeed + (runRoller.getAsBoolean() ? 1. : 0.)),
            intake.setIntakePosition(() -> {
                return pullIn.getAsDouble() * 0.7 + (runRoller.getAsBoolean() ? 0.0 : 0.1);
            })
        );
    }
}
