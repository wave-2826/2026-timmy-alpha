package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeSet;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.RioAlerts;
import frc.robot.util.VirtualSubsystem;

public class LEDs extends VirtualSubsystem {
    /**
     * We skip updating LEDs until this many full periodic loop cycles have passed. This prevents some weird behavior.
     */
    private static final int SKIP_FIRST_LOOP_CYCLES = 10;
    /**
     * The duration of the autonomous start animation, in seconds.
     */
    private static final double AUTONOMOUS_START_ANIMATION_DURATION = 1.0;

    /** The WAVE Blue color. */
    private static final Color WAVE_BLUE = new Color("#00A0C3");
    /** The WAVE Green color. */
    private static final Color WAVE_GREEN = new Color("#3bff36");

    /** The maximum speed multiplier for when the robot is going its maximum speed. */
    private static final double maximumSpeedMultiplier = 2.5;

    private int[] buffer;

    /**
     * The active compositing mode. Set while rendering the LEDs.
     */
    private LEDCompositingMode activeCompositingMode = LEDCompositingMode.Exclusive;

    private LEDIO io;
    /**
     * The active LED states. We use a TreeSet of LEDState enums because it will automatically sort the states by their
     * priority order. Then, we display the highest priority state that is active.
     */
    private TreeSet<LEDState> activeStates = new TreeSet<LEDState>();
    /**
     * The time that each state was last started. Used to fade temporary states. NOTE: This isn't in the same time base
     * as the LED time; it uses Timer.getTimestamp() so it's not affected by the robot speed multiplier.
     */
    private HashMap<LEDState, Double> stateStartTimes = new HashMap<LEDState, Double>();

    /**
     * The number of loop cycles that have passed.
     */
    private int loopCycles = 0;

    /**
     * A notifier that runs the animation until the robot code is fully started. This allows us to display the LEDs as
     * soon as possible.
     */
    private final Notifier initialUpdateNotifier;
    /**
     * If initial updates are currently running.
     */
    private boolean initialUpdatesRunning = true;

    /**
     * The time of the last update.
     */
    private double lastTime = Timer.getTimestamp();
    /**
     * The time used for the LEDs, in seconds (but scaled by the drivetrain speed). This isn't necessarily monotonically
     * increasing.
     */
    private double time = 0;

    /**
     * A compositing mode for the LED state. Used to allow us to layer multiple LED states on top of each other.
     */
    public static enum LEDCompositingMode {
        /** The LED state is added on top of the previous state. */
        Additive,
        /** The LED state is multiplied with the previous state. */
        Multiplicative,
        /** The value of the new state is used to blend. */
        Value,
        /** This state is layered on top of all previous states. The below states aren't even rendered. */
        Exclusive
    }

    /**
     * The set of possible LED states. Each state has a lambda function that accepts the LED subsystem.
     *
     * The order of the states is their priority order. The first state in the enum is the highest priority, and the
     * last state is the lowest priority.
     */
    public static enum LEDState {
        EStopped((leds) -> leds.pulse(Color.kBlack, Color.kRed, 2.0)), // Active when the robot is E-stopped

        BatteryLow((leds) -> leds.flash(Color.kOrangeRed, Color.kBlack, 0.3, 0.05, 0.3, 5.0), LEDCompositingMode.Value), // Active when the robot battery is low

        AutonomousStart(LEDs::autonomousStart, LEDCompositingMode.Additive), // Active at the start of autonomous

        ScoringRecovering((leds) -> leds.rainbow(0.6, 1.0)),
        Scoring((leds) -> leds.rainbow()), //
        
        Compacting((leds) -> leds.pulse(Color.kPurple, Color.kPink, 0.3)), //

        // CoralSeen((leds) -> leds.rainbow(0.5, 0.5)),

        Disabled((leds) -> leds.gradient(leds.allianceDark(), leds.allianceLight(), 5.0)), // Active when the robot is disabled

        Teleop((leds) -> leds.gradient(leds.allianceDark(), leds.allianceLight(), 2.0)), // Active when the robot is in teleop
        Autonomous((leds) -> leds.gradient(WAVE_BLUE, WAVE_GREEN, 1.5, 4.0)), // Active when the robot is in autonomous
        Test((leds) -> leds.gradient(Color.kYellow, Color.kOrange, 5.0)), // Active when the robot is in test mode

        Default((leds) -> leds.gradient(WAVE_BLUE, Color.kWhite, 2.0)); // Active when no other state is active

        private final Consumer<LEDs> function;
        private final LEDCompositingMode compositingMode;

        private LEDState(Consumer<LEDs> function) {
            this(function, LEDCompositingMode.Exclusive);
        }

        private LEDState(Consumer<LEDs> function, LEDCompositingMode compositingMode) {
            this.function = function;
            this.compositingMode = compositingMode;
        }
    }

    public LEDs(LEDIO io) {
        buffer = new int[LEDConstants.ledCount * 3];

        this.io = io;

        enableState(LEDState.Default);

        initialUpdateNotifier = new Notifier(() -> {
            synchronized(this) {
                this.time += 1. / 50.;
                this.gradient(Color.kWhite, Color.kDimGray, 1.0);
                this.io.pushLEDs(this.buffer);
            }
        });
        initialUpdateNotifier.setName("LED Initial Updates");
        initialUpdateNotifier.startPeriodic(1. / 50.);

        registerLEDTriggers();
    }

    /**
     * Binds a state to a trigger.
     */
    private void bindStateToTrigger(LEDState state, Trigger trigger) {
        trigger.onFalse(disableStateCommand(state)).onTrue(enableStateCommand(state));
    }

    /**
     * Registers the LED triggers.
     */
    private void registerLEDTriggers() {
        Trigger lowBatteryTrigger = new Trigger(RioAlerts.getInstance()::batteryLow);
        Trigger eStoppedTrigger = new Trigger(DriverStation::isEStopped);

        // RobotModeTriggers exists, but some of them don't respect disabled mode? This seems cleaner anyway.
        Trigger autonomousTrigger = new Trigger(DriverStation::isAutonomousEnabled);
        Trigger disabledTrigger = new Trigger(DriverStation::isDisabled);
        Trigger teleopTrigger = new Trigger(DriverStation::isTeleopEnabled);
        Trigger testTrigger = new Trigger(DriverStation::isTestEnabled);

        bindStateToTrigger(LEDState.EStopped, eStoppedTrigger);
        bindStateToTrigger(LEDState.BatteryLow, lowBatteryTrigger);

        bindStateToTrigger(LEDState.Autonomous, autonomousTrigger);
        bindStateToTrigger(LEDState.Disabled, disabledTrigger);
        bindStateToTrigger(LEDState.Teleop, teleopTrigger);
        bindStateToTrigger(LEDState.Test, testTrigger);

        autonomousTrigger.onTrue(Commands.sequence(
            enableStateCommand(LEDState.AutonomousStart),
            Commands.waitSeconds(AUTONOMOUS_START_ANIMATION_DURATION),
            disableStateCommand(LEDState.AutonomousStart)
        ));

        // Start the disabled state
        enableState(LEDState.Disabled);
    }

    /**
     * Gets the time since the LED state was last changed.
     */
    private double timeSinceStart(LEDState state) {
        return Timer.getTimestamp() - stateStartTimes.getOrDefault(state, 0.);
    }

    /**
     * Enables the given LED state. States are automatically prioritized.
     */
    public void enableState(LEDState state) {
        if(!activeStates.contains(state)) {
            activeStates.add(state);
            stateStartTimes.put(state, Timer.getTimestamp());
        }
    }

    /**
     * Disables the given LED state. States are automatically prioritized.
     */
    public void disableState(LEDState state) {
        activeStates.remove(state);
    }

    /**
     * Creates a command that enables the given state.
     * @return
     */
    public Command enableStateCommand(LEDState state) {
        return Commands.runOnce(() -> enableState(state)).ignoringDisable(true);
    }

    /**
     * Creates a command that disables the given state.
     */
    public Command disableStateCommand(LEDState state) {
        return Commands.runOnce(() -> disableState(state)).ignoringDisable(true);
    }

    /**
     * Creates a command that runs the given state while the command runs.
     */
    public Command runStateCommand(LEDState state) {
        return Commands.startEnd(() -> enableState(state), () -> disableState(state)).ignoringDisable(true);
    }

    /**
     * Gets the dark alliance color.
     */
    private Color allianceDark() {
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return new Color(0.0f, 0.0f, 1.0f);
        } else {
            return new Color(1.0f, 0.0f, 0.0f);
        }
    }

    /**
     * Gets the light alliance color.
     */
    private Color allianceLight() {
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return new Color(0.0f, 0.0f, 0.3f);
        } else {
            return new Color(0.3f, 0.0f, 0.0f);
        }
    }

    /**
     * Sets if the given LED state is active. States are automatically prioritized.
     */
    public void setStateActive(LEDState state, boolean active) {
        if(active) {
            enableState(state);
        } else {
            disableState(state);
        }
    }

    /**
     * A solid color effect.
     */
    public void solid(Color color) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            setLEDColor(i, color);
        }
    }

    /**
     * A moving rainbow effect.
     */
    private void rainbow() {
        rainbow(1, 1);
    }

    /**
     * A moving rainbow effect.
     */
    public void rainbow(double saturationScalar, double speedScalar) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double t = (time * speedScalar + i / (double) LEDConstants.ledCount) % 1.;
            setLEDColor(i, Color.fromHSV((int) (t * 255), (int) (255 * saturationScalar), 255));
        }
    }

    /**
     * A flashing effect. Periods must be an even number of values, where every pair of values is a period of on and off
     * times in seconds.
     * @param color
     * @param periods
     */
    public void flash(Color colorOn, Color colorOff, double... periods) {
        double totalLength = 0;
        for(double period : periods) {
            totalLength += period;
        }

        double time = this.time % totalLength;
        double factor = 0;

        double accumulatedTime = 0;
        for(int i = 0; i < periods.length; i += 2) {
            double periodOn = periods[i];
            double periodOff = periods[i + 1];

            if(time < accumulatedTime + periodOn) {
                factor = (time - accumulatedTime) / periodOn;
                break;
            } else if(time < accumulatedTime + periodOn + periodOff) {
                factor = 1.;
                break;
            }

            accumulatedTime += periodOn + periodOff;
        }

        solid(Color.lerpRGB(colorOn, colorOff, factor));
    }

    /**
     * A pulsing color effect.
     */
    public void pulse(Color color1, Color color2, double period) {
        // We offset the period slightly based on how far the pixels are
        // from the center of the strip to create a wave effect
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double offset = (i - LEDConstants.ledCount / 2) / (double) LEDConstants.ledCount;
            double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * time / period + offset);
            setLEDColor(i, Color.lerpRGB(color1, color2, brightness));
        }
    }

    /**
     * A rotating circular gradient effect.
     */
    public void gradient(Color color1, Color color2, double period) {
        gradient(color1, color2, period, 1.);
    }

    /**
     * A rotating circular gradient effect.
     */
    public void gradient(Color color1, Color color2, double period, double repetitions) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            double t = 0.5 + 0.5 * Math.sin(2 * Math.PI * (time / period + i / (double) LEDConstants.ledCount * repetitions));
            setLEDColor(i, Color.lerpRGB(color1, color2, t));
        }
    }

    /**
     * The fade effect we use at the start of autonomous.
     */
    public static void autonomousStart(LEDs leds) {
        double fade = Math.min(1., leds.timeSinceStart(LEDState.AutonomousStart) / AUTONOMOUS_START_ANIMATION_DURATION);
        leds.solid(Color.lerpRGB(Color.kLimeGreen, Color.kBlack, fade));
    }

    /**
     * Sets the color of an LED, adjusting for the compositing mode.
     * @param index The index of the LED.
     * @param color The color to set.
     */
    private void setLEDColor(int index, Color color) {
        switch(activeCompositingMode) {
            case Additive: {
                buffer[index * 3] = Math.min(255, (int) (buffer[index * 3] * 255 + color.red * 255));
                buffer[index * 3 + 1] = Math.min(255, (int) (buffer[index * 3 + 1] * 255 + color.green * 255));
                buffer[index * 3 + 2] = Math.min(255, (int) (buffer[index * 3 + 2] * 255 + color.blue * 255));
                break;
            }
            case Multiplicative: {
                buffer[index * 3] = (int) (buffer[index * 3] * color.red);
                buffer[index * 3 + 1] = (int) (buffer[index * 3 + 1] * color.green);
                buffer[index * 3 + 2] = (int) (buffer[index * 3 + 2] * color.blue);
                break;
            }
            case Value: {
                // use the brightness of the new color to blend between the old color and the new color
                double brightness = (color.red + color.green + color.blue) / 3.;
                buffer[index * 3] = (int) (buffer[index * 3] * (1 - brightness) + color.red * 255 * brightness);
                buffer[index * 3 + 1] = (int) (buffer[index * 3 + 1] * (1 - brightness) + color.green * 255 * brightness);
                buffer[index * 3 + 2] = (int) (buffer[index * 3 + 2] * (1 - brightness) + color.blue * 255 * brightness);
                break;
            }
            case Exclusive: {
                buffer[index * 3] = (int) (color.red * 255);
                buffer[index * 3 + 1] = (int) (color.green * 255);
                buffer[index * 3 + 2] = (int) (color.blue * 255);
                break;
            }
        }
    }

    @Override
    public void periodic() {}

    /**
     * Updates the LEDs. Runs regardless of the robot mode.
     */
    @Override
    public void periodicAfterScheduler() {
        loopCycles++;
        if(loopCycles < SKIP_FIRST_LOOP_CYCLES) { return; }

        if(initialUpdatesRunning) {
            initialUpdateNotifier.stop();
            initialUpdatesRunning = false;
        }

        // Update the time based on the delta and robot speed
        double robotSpeedMultiplier = 1. + RobotState.getInstance().getRobotLinearVelocity()
            / DriveConstants.maxSpeedMetersPerSec * (maximumSpeedMultiplier - 1.);

        double currentTime = Timer.getTimestamp();
        time += (currentTime - lastTime) * robotSpeedMultiplier;
        lastTime = currentTime;

        ArrayList<String> activeStateNames = new ArrayList<>();

        if(!activeStates.isEmpty()) {
            // To render the LEDs with compositing, we need to find the highest-priority exclusive
            // state and render upward from that. That way, we avoid unnecessary work that will
            // be overwritten by the exclusive state.
            LEDState[] activeStatesArray = activeStates.toArray(new LEDState[0]);

            int exclusiveStateIndex = activeStatesArray.length;
            for(int i = 0; i < activeStatesArray.length - 1; i++) {
                if(activeStatesArray[i].compositingMode == LEDCompositingMode.Exclusive) {
                    exclusiveStateIndex = i;
                    break;
                }
            }

            for(int i = exclusiveStateIndex; i >= 0; i--) {
                activeCompositingMode = activeStatesArray[i].compositingMode;
                activeStatesArray[i].function.accept(this);
                activeStateNames.add(activeStatesArray[i].name());
            }
        }

        Logger.recordOutput("LEDs/ActiveStates", activeStateNames.toString());
        Logger.recordOutput("LEDs/Time", time);
        Logger.recordOutput("LEDs/RobotSpeedMultiplier", robotSpeedMultiplier);

        io.pushLEDs(buffer);
    }
}
