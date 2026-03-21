package frc.robot.util;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAlert;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

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
public class LoggedAutoChooser extends LoggedNetworkInput {
    final String key;
    static final String NONE_NAME = "__Nothing__";
    private static final Alert selectedNonexistentAuto = ChoreoAlert.alert("Selected an auto that doesn't exist!", kError);
    
    private final HashMap<String, Supplier<Command>> autoRoutines = new HashMap<>(Map.of(NONE_NAME, Commands::none));
    
    private final StringEntry selected, active;
    private final StringArrayEntry options;
    
    private String lastCommandName = NONE_NAME;
    private Command lastCommand = Commands.none();
    
    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, lastCommandName);
        }
        
        public void fromLog(LogTable table) {
            lastCommandName = table.get(key, lastCommandName);
        }
    };
    
    public LoggedAutoChooser(String tableName) {
        this(tableName, NetworkTableInstance.getDefault());
    }
    
    LoggedAutoChooser(String tableName, NetworkTableInstance ntInstance) {
        Logger.registerDashboardInput(this);
        if(tableName == null) tableName = "";
        
        key = tableName;
        String path = tableName.isEmpty() ? "" : NetworkTable.normalizeKey(tableName, true);
        NetworkTable table = ntInstance.getTable(path + "/AutoChooser");
        
        selected = table.getStringTopic("selected").getEntry("");
        selected.set(NONE_NAME);
        
        table.getStringTopic(".type").publish().set("String Chooser");
        table.getStringTopic("default").publish().set(NONE_NAME);
        
        active = table.getStringTopic("active").getEntry(NONE_NAME);
        active.set(NONE_NAME);
        
        var defaultOptions = autoRoutines.keySet().toArray(new String[0]);
        options = table.getStringArrayTopic("options").getEntry(defaultOptions);
        options.set(defaultOptions);
        periodic();

        // "This function should not be called by the user"... heh. try me.
        Logger.registerDashboardInput(this);
    }
    
    /**
    * Update the auto chooser.
    *
    * This shouldn't be called by the user! These dashboard inputs are automatically registered.
    *
    * <p>The AutoRoutine can only be updated when the robot is disabled and connected to
    * driver station. If the .chooser in your dashboard says {@code BAD}, the {@link LoggedAutoChooser}
    * has not responded to the selection yet and you need to disable the robot to update it.
    */
    public void periodic() {
        if(!Logger.hasReplaySource()) {
            if(DriverStation.isDisabled()
            && DriverStation.isDSAttached()
            && DriverStation.getAlliance().isPresent()) {
                String selectStr = selected.get();
                if(selectStr.equals(lastCommandName)) return;
                if(!autoRoutines.containsKey(selectStr) && !selectStr.equals(NONE_NAME)) {
                    selected.set(NONE_NAME);
                    selectStr = NONE_NAME;
                    selectedNonexistentAuto.set(true);
                } else {
                    selectedNonexistentAuto.set(false);
                }
                lastCommandName = selectStr;
                lastCommand = autoRoutines.get(lastCommandName).get();
                active.set(lastCommandName);
            }
        }
        Logger.processInputs(prefix + " " + key, inputs);
    }
    
    /**
    * Add an AutoRoutine to the chooser.
    *
    * <p>The options of the chooser are actually a function that takes an {@link AutoFactory} and
    * returns a {@link AutoRoutine}. These functions can be static, a lambda or belong to a local
    * variable.
    *
    * This is done to load AutoRoutines when and only when they are selected, in order to save
    * memory and file loading time for unused AutoRoutines.
    */
    public void addRoutine(String name, Supplier<AutoRoutine> generator) {
        autoRoutines.put(name, () -> generator.get().cmd());
        options.set(autoRoutines.keySet().toArray(new String[0]));
    }
    
    /**
    * Adds a Command to the auto chooser.
    *
    * <p>This is done to load autonomous commands when and only when they are selected, in order to
    * save memory and file loading time for unused autonomous commands.
    *
    * <h3>Example:</h3>
    *
    * <pre><code>
    * LoggedAutoChooser chooser;
    * Autos autos = new Autos(swerve, shooter, intake, feeder);
    * public Robot() {
    *   chooser = new LoggedAutoChooser("/Choosers");
    *   addPeriodic(chooser::update, 0.02); // chooser must be updated every loop
    *   // fourPieceLeft is a method that accepts an AutoFactory and returns a command.
    *   chooser.addCmd("4 Piece left", autos::fourPieceLeft);
    *   chooser.addCmd("Just Shoot", shooter::shoot);
    * }
    * </code></pre>
    */
    public void addCmd(String name, Supplier<Command> generator) {
        autoRoutines.put(name, generator);
        options.set(autoRoutines.keySet().toArray(new String[0]));
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
        return lastCommand.withName(lastCommandName);
    }
}