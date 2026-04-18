package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class ShiftHelpers {
    private static ShiftHelpers instance;
    public static ShiftHelpers getInstance() {
        if(instance == null) {
            instance = new ShiftHelpers();
        }
        return instance;
    }
    
    private static LoggedNetworkString overrideFMS = new LoggedNetworkString("/Overrides/AutoWin");
    public static boolean blueWonAuto() {
        String matchInfo = DriverStation.getGameSpecificMessage();
        String override = overrideFMS.toString().trim().toUpperCase();
        if(override.equals("B") || override.equals("R")) {
            matchInfo = override;
        }
        if(matchInfo != null && matchInfo.length() > 0) {
            return matchInfo.charAt(0) != 'B';
        }

        // Default if data isn't ready yet
        return false;
    }
    public static boolean currentAllianceWonAuto() {
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue);
        return blueWonAuto() ? isBlueAlliance : !isBlueAlliance;
    }
        
    public enum Shift {
        // Order matters here!
        DISABLED(false, false, 0., false, "Disabled"),
        AUTO(true, true, 20., false, "Autonomous"),
        TRANSITION(true, true, 10., "Transition - WON AUTO", "Transition - LOST AUTO"),
        SHIFT1(true, false, 25., "Shift 1"),
        SHIFT2(false, true, 25., "Shift 2"),
        SHIFT3(true, false, 25., "Shift 3"),
        SHIFT4(false, true, 25., "Shift 4"),
        ENDGAME(true, true, 30., false, "Endgame");

        public boolean winnerCanScore;
        public boolean loserCanScore;
        /** The duration of this shift. If zero, the shift won't advance automatically. */
        public double duration;
        public String winText;
        public String loseText;
        public boolean advanceAfterDuration = true;

        private Shift(boolean winnerCanScore, boolean loserCanScore, double duration, String winName, String loseName) {
            this.winnerCanScore = winnerCanScore;
            this.loserCanScore = loserCanScore;
            this.duration = duration;
            this.winText = winName;
            this.loseText = loseName;
        }
        
        private Shift(boolean winnerCanScore, boolean loserCanScore, double duration, String name) {
            this(winnerCanScore, loserCanScore, duration, name, name);
        }

        private Shift(boolean winnerCanScore, boolean loserCanScore, double duration, boolean advanceAfterDuration, String name) {
            this(winnerCanScore, loserCanScore, duration, name, name);
            this.advanceAfterDuration = advanceAfterDuration;
        }

        public boolean canScore() {
            if(this == DISABLED) return false;
            boolean isWinningAlliance = currentAllianceWonAuto();
            return (isWinningAlliance && winnerCanScore) || (!isWinningAlliance && loserCanScore);
        }
        public String getText() {
            return currentAllianceWonAuto() ? winText : loseText;
        }
    }
    
    private Shift currentShift = Shift.DISABLED;
    
    /**
     * The timer for the current shift, restarted when a new shift begins.
     * We track independently of the FMS because its time reports aren't reliable.
     */
    private Timer shiftTimer = new Timer();

    public void advanceShiftIfNeeded() {
        if(
            currentShift.duration > 0. &&
            currentShift.advanceAfterDuration &&
            shiftTimer.advanceIfElapsed(currentShift.duration)
        ) {
            // Next shift ordinal
            int nextOrdinal = (currentShift.ordinal() + 1) % Shift.values().length;
            currentShift = Shift.values()[nextOrdinal];
        }
    }

    private LoggedNetworkString currentShiftEntry = new LoggedNetworkString("/ShiftHelpers/CurrentShift");
    private LoggedNetworkNumber shiftTimeRemainingEntry = new LoggedNetworkNumber("/ShiftHelpers/ShiftTimeRemaining");
    private LoggedNetworkBoolean canScoreEntry = new LoggedNetworkBoolean("/ShiftHelpers/CanScore");
    public ShiftHelpers() {
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
            currentShift = Shift.AUTO;
            shiftTimer.restart();
        }).ignoringDisable(true));
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            currentShift = Shift.TRANSITION;
            shiftTimer.restart();
        }).ignoringDisable(true));
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            currentShift = Shift.DISABLED;
            shiftTimer.stop();
        }).ignoringDisable(true));

        // TODO: Shift LEDs
    }

    /** Updates the shift helpers' logging. */
    public void periodic() {
        advanceShiftIfNeeded();

        currentShiftEntry.set(currentShift.getText());
        shiftTimeRemainingEntry.set(Math.max(0., currentShift.duration - shiftTimer.get()));
        canScoreEntry.set(currentShift.canScore());

        Logger.recordOutput("ShiftHelpers/BlueWonAuto", blueWonAuto());
    }

    public Shift getActiveShift() {
        return currentShift;
    }

    public Shift getShiftIn(double seconds) {
        if(currentShift.duration > 0.) {
            double timeUntilNextShift = currentShift.duration - shiftTimer.get();
            if(seconds >= timeUntilNextShift) {
                int shiftsToAdvance = 1 + (int)((seconds - timeUntilNextShift) / Shift.values()[0].duration);
                int nextOrdinal = (currentShift.ordinal() + shiftsToAdvance) % Shift.values().length;
                return Shift.values()[nextOrdinal];
            }
        }
        return currentShift;
    }
}