package frc.robot.util.tunables;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import frc.robot.Robot;

public class LoggedTunableVector<N extends Num> implements Supplier<Vector<N>> {
    private final String key;
    private Vector<N> defaultValue;
    private Vector<N> value;
    private LoggedNetworkNumber[] dashboardNumbers;
    private Map<Integer, Double[]> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableVector(String dashboardKey, Vector<N> defaultValue) {
        this.key = LoggedTunableNumber.tableKey + "/" + dashboardKey;
        
        this.defaultValue = defaultValue;
        this.value = new Vector<>(this.defaultValue);

        if(Robot.tuningMode()) {
            dashboardNumbers = new LoggedNetworkNumber[defaultValue.getNumRows()];
            for(int i = 0; i < defaultValue.getNumRows(); i++) {
                dashboardNumbers[i] = new LoggedNetworkNumber(key + "/" + i, defaultValue.get(i));
            }
        }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public Vector<N> get() {
        if(Robot.tuningMode()) {
            for(int i = 0; i < value.getNumRows(); i++) {
                value.set(i, 0, dashboardNumbers[i].get());
            }
        }

        return value;
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple objects. Recommended
     *            approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false otherwise.
     */
    public boolean hasChanged(int id) {
        if(Robot.tuningMode()) {
            Double[] lastValues = lastHasChangedValues.get(id);
            if(lastValues == null) {
                lastValues = new Double[value.getNumRows()];
                Arrays.fill(lastValues, Double.NaN);
            }

            boolean changed = false;
            for(int i = 0; i < value.getNumRows(); i++) {
                double currentValue = dashboardNumbers[i].get();
                if(Double.isNaN(lastValues[i]) || currentValue != lastValues[i]) {
                    changed = true;
                    lastValues[i] = currentValue;
                }
            }

            lastHasChangedValues.put(id, lastValues);
            return changed;
        }

        return false;
    }
}
