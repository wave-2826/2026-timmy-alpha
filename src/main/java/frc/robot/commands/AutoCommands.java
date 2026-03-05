package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoCommands {
    public static enum AutoPaths {
        // SWIPE = Go out to center grab and come back
        // SWEEP = Go out and go across and come back
        RIGHT_SWIPE_OUTPOST,
        RIGHT_SWIPE_CLIMB_RIGHT,
        RIGHT_SWIPE_CLIMB_LEFT,
        RIGHT_SWEEP_CLIMB_RIGHT,
        RIGHT_SWEEP_CLIMB_LEFT,
        LEFT_DOUBLE_SWIPE,
        LEFT_SWIPE_CLIMB_RIGHT,
        LEFT_SWIPE_CLIMB_LEFT,
        LEFT_SWEEP_CLIMB_RIGHT,
        LEFT_SWEEP_CLIMB_LEFT,
        LEFT_SWEEP_OUTPOST,
    }

    public static Command runCodeCommand(AutoPaths path) {
        switch (path) {
            case RIGHT_SWIPE_OUTPOST:
            case RIGHT_SWIPE_CLIMB_RIGHT:
            case RIGHT_SWIPE_CLIMB_LEFT:
            case RIGHT_SWEEP_CLIMB_RIGHT:
            case RIGHT_SWEEP_CLIMB_LEFT:
            case LEFT_DOUBLE_SWIPE:
            case LEFT_SWIPE_CLIMB_RIGHT:
            case LEFT_SWIPE_CLIMB_LEFT:
            case LEFT_SWEEP_CLIMB_RIGHT:
            case LEFT_SWEEP_CLIMB_LEFT:
            case LEFT_SWEEP_OUTPOST:
        }
        return null;
    }
}
