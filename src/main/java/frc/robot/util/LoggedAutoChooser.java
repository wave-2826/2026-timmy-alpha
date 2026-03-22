package frc.robot.util;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAlert;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
* An Choreo specific {@code SendableChooser} that allows for the selection of {@link AutoRoutine}s at runtime via a
* <a href="https://docs.wpilib.org/en/stable/docs/software/dashboards/index.html#dashboards">Dashboard</a>.
*
* <p>
* This chooser takes a <a href="https://en.wikipedia.org/wiki/Lazy_loading">lazy loading</a> approach to
* {@link AutoRoutine}s, only generating the {@link AutoRoutine} when it is selected. This approach has the benefit of
* not loading all autos on startup, but also not loading the auto during auto start causing a delay.
*
* <p>
* Once the {@link LoggedAutoChooser} is made you can add {@link AutoRoutine}s to it using {@link #addRoutine} or add
* {@link Command}s to it using {@link #addCmd}. Similar to {@code
* SendableChooser} this chooser can be added to the {@link edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} using
* {@code
* SmartDashboard.putData(Sendable)}.
*
* <p>
* You can set the Robot's autonomous command to the chooser's chosen auto routine via <code>
* RobotModeTriggers.autonomous.whileTrue(chooser.autoSchedulingCmd());</code>
*/
public class LoggedAutoChooser {
    static final String NONE_NAME = "Nothing";
    private static final Alert selectedNonexistentAuto = ChoreoAlert.alert("Selected an auto that doesn't exist!", kError);
    
    private final HashMap<String, Supplier<Command>> getters = new HashMap<>(
        Map.of(NONE_NAME, Commands::none)
    );
    
    private String lastSelectedValue = NONE_NAME;
    private Command lastCommand = Commands.none();

    private LoggedDashboardChooser<String> dashboardChooser;

    public LoggedAutoChooser(String key) {
        dashboardChooser = new LoggedDashboardChooser<>(key);

        var defaultOptions = getters.keySet().toArray(new String[0]);
        for(String option : defaultOptions) {
            if(option.equals(NONE_NAME)) {
                dashboardChooser.addDefaultOption(option, option);
            } else {
                dashboardChooser.addOption(option, option);
            }
        }

        dashboardChooser.onChange(this::selected);
    }

    private void selected(String value) {
        if(value == null) return;
        if(value.equals(lastSelectedValue)) return;
        
        if(!getters.containsKey(value) && !value.equals(NONE_NAME)) {
            selectedNonexistentAuto.set(true);
        } else {
            selectedNonexistentAuto.set(false);
        }

        lastSelectedValue = value;
        lastCommand = getters.get(lastSelectedValue).get();
    }
    
    
    /**
    * Add an AutoRoutine to the chooser.
    *
    * This is done to load AutoRoutines when and only when they are selected, in order to save
    * memory and file loading time for unused AutoRoutines.
    */
    public void addRoutine(String name, Supplier<AutoRoutine> generator) {
        getters.put(name, () -> generator.get().cmd());
        dashboardChooser.addOption(name, name);
    }
    
    /**
    * Adds a Command to the auto chooser.
    *
    * This is done to load autonomous commands when and only when they are selected, in order to
    * save memory and file loading time for unused autonomous commands.
    */
    public void addCmd(String name, Supplier<Command> generator) {
        getters.put(name, generator);
        dashboardChooser.addOption(name, name);
    }
    
    /**
    * Gets a Command that schedules the selected auto routine. This Command finishes immediately as
    * it simply schedules another Command. This Command can directly be bound to a trigger.
    */
    public Command selectedCommandScheduler() {
        return Commands.defer(() -> lastCommand.asProxy(), Set.of());
    }
    
    /**
    * Returns the currently selected command.
    *
    * If you plan on using this command in a trigger it is recommended to use
    * {@link #selectedCommandScheduler()} instead.
    */
    public Command selectedCommand() {
        return lastCommand.withName(lastSelectedValue);
    }

    public String getSelectedName() {
        return lastSelectedValue;
    }
}