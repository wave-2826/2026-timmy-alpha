package frc.robot.util.simUtils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Simulates the main battery of the robot. Electrical appliances can be added to the battery to draw
 * current, and the battery voltage is affected by the current drawn from various appliances.
 */
public class SimulatedBattery {
    // Nominal voltage for a fully charged battery (13.5 volts).
    private static final double BATTERY_NOMINAL_VOLTAGE = 13.5;

    // Filter to smooth the current readings.
    private static final LinearFilter currentFilter = LinearFilter.movingAverage(50);

    private static final List<Supplier<Current>> electricalAppliances = new ArrayList<>();

    // The current battery voltage in volts.
    private static double batteryVoltageVolts = BATTERY_NOMINAL_VOLTAGE;

    public static void addElectricalAppliances(Supplier<Current> customElectricalAppliances) {
        electricalAppliances.add(customElectricalAppliances);
    }

    public static void simulationSubTick() {
        double totalCurrentAmps = getTotalCurrentDrawn().in(Amps);
        totalCurrentAmps = currentFilter.calculate(totalCurrentAmps);

        batteryVoltageVolts = BatterySim.calculateLoadedBatteryVoltage(BATTERY_NOMINAL_VOLTAGE, 0.02, totalCurrentAmps);

        if(Double.isNaN(batteryVoltageVolts)) {
            batteryVoltageVolts = 12.0;
            DriverStation.reportError(
                "[MapleSim (kind of)] Internal Library Error: Calculated battery voltage is invalid, reverting to normal operation voltage...",
                false);
        }
        if(batteryVoltageVolts < RoboRioSim.getBrownoutVoltage()) {
            batteryVoltageVolts = RoboRioSim.getBrownoutVoltage();
            DriverStation.reportError("[MapleSim (kind of)] BrownOut Detected, protecting battery voltage...", false);
        }

        RoboRioSim.setVInVoltage(batteryVoltageVolts);
    }

    public static Voltage getBatteryVoltage() {
        return Volts.of(batteryVoltageVolts);
    }

    public static Current getTotalCurrentDrawn() {
        double totalCurrentAmps = electricalAppliances.stream()
            .mapToDouble(currentSupplier -> currentSupplier.get().in(Amps))
            .sum();
        return Amps.of(totalCurrentAmps);
    }

    public static Voltage clamp(Voltage voltage) {
        return Volts.of(MathUtil.clamp(voltage.in(Volts), -batteryVoltageVolts, batteryVoltageVolts));
    }
}