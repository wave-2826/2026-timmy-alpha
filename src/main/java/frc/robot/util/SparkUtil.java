package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SignalsConfig;

import edu.wpi.first.wpilibj.Alert;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SparkUtil {
    private static final boolean ENABLE_ALERT_TRACKING = true;
    
    /** Stores whether any error was has been detected by other utility methods. */
    public static boolean sparkStickyFault = false;

    /** Returns if there has been a fault and resets if so. */
    public static boolean checkFault() {
        if(sparkStickyFault) {
            sparkStickyFault = false;
            return true;
        }
        return false;
    }
    
    public static SignalsConfig defaultSignals = new SignalsConfig()
        .faultsAlwaysOn(true)
        .warningsAlwaysOn(true)
        .iAccumulationAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .primaryEncoderPositionAlwaysOn(false)
        .primaryEncoderVelocityAlwaysOn(false)
        .absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        
        .faultsPeriodMs(250)
        .limitsPeriodMs(20)
        .warningsPeriodMs(250)
        .busVoltagePeriodMs(20)
        .analogVoltagePeriodMs(250)
        .appliedOutputPeriodMs(20)
        .iAccumulationPeriodMs(250)
        .outputCurrentPeriodMs(20)
        .analogPositionPeriodMs(250)
        .analogVelocityPeriodMs(250)
        .motorTemperaturePeriodMs(250)
        .primaryEncoderPositionPeriodMs(100)
        .primaryEncoderVelocityPeriodMs(100)
        .absoluteEncoderPositionPeriodMs(250)
        .absoluteEncoderVelocityPeriodMs(250)
        .externalOrAltEncoderPosition(250)
        .externalOrAltEncoderVelocity(250);
    
    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if(spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            sparkStickyFault = true;
        }
    }
    
    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for(int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if(spark.getLastError() != REVLibError.kOk) {
                sparkStickyFault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Returns a spark's value only if it's valid; otherwise, returns a default. */
    public static double getIfOk(SparkBase spark, DoubleSupplier supplier, double defaultValue) {
        double value = supplier.getAsDouble();
        if(spark.getLastError() == REVLibError.kOk) {
            return value;
        } else {
            sparkStickyFault = true;
            return defaultValue;
        }
    }
    
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
        for(int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if(error == REVLibError.kOk) {
                break;
            } else {
                sparkStickyFault = true;
            }
        }
    }

    private static class SparkFaultAlerts {
        private final SparkBase spark;
        private final Alert[] faultAlerts;
        private final Alert[] warningAlerts;

        public SparkFaultAlerts(SparkBase spark, String name) {
            this.spark = spark;
            this.faultAlerts = new Alert[] {
                new Alert(name + " other fault", Alert.AlertType.kError),
                new Alert(name + " motor type fault", Alert.AlertType.kError),
                new Alert(name + " sensor fault", Alert.AlertType.kError),
                new Alert(name + " can fault", Alert.AlertType.kError),
                new Alert(name + " temperature fault", Alert.AlertType.kError),
                new Alert(name + " gate driver fault", Alert.AlertType.kError),
                new Alert(name + " esc eeprom fault", Alert.AlertType.kError),
                new Alert(name + " firmware fault", Alert.AlertType.kError)
            };
            this.warningAlerts = new Alert[] {
                new Alert(name + " brownout warning", Alert.AlertType.kWarning),
                new Alert(name + " overcurrent warning", Alert.AlertType.kWarning),
                new Alert(name + " esc eeprom warning", Alert.AlertType.kWarning),
                new Alert(name + " ext eeprom warning", Alert.AlertType.kWarning),
                new Alert(name + " sensor warning", Alert.AlertType.kWarning),
                new Alert(name + " stall warning", Alert.AlertType.kWarning), //
                null, // new Alert(name + " has reset warning", Alert.AlertType.kWarning),
                new Alert(name + " other warning", Alert.AlertType.kWarning)
            };
        }

        public void updateAlerts() {
            var faults = spark.getFaults();
            int faultBits = faults.rawBits;
            int warningBits = spark.getWarnings().rawBits;

            // if there's a CAN fault, disable others since they're meaningless 
            if(faults.can) {
                faultBits = 1 >> 3;
                warningBits = 0;
            }

            for(int i = 0; i < faultAlerts.length; i++) {
                if(faultAlerts[i] != null) faultAlerts[i].set(((faultBits >> i) & 1) == 1);
            }
            for(int i = 0; i < warningAlerts.length; i++) {
                if(warningAlerts[i] != null) warningAlerts[i].set(((warningBits >> i) & 1) == 1);
            }
        }
    }

    private static final List<SparkFaultAlerts> sparkFaultAlerts = new ArrayList<>();

    /** Registers a spark to show alerts if there are any active motor controller faults or warnings. */
    public static void registerFaultAlerts(SparkBase spark, String name) {
        sparkFaultAlerts.add(new SparkFaultAlerts(spark, name));
    }

    private static int faultUpdateIndex = 0;

    /** Updates all registered spark fault alerts. */
    public static void updateFaultAlerts() {
        if(!ENABLE_ALERT_TRACKING) return;
        if(sparkFaultAlerts.size() == 0) return;

        // Spread out work by only updating one spark's faults per loop
        // Probably not necessary but meh
        sparkFaultAlerts.get(faultUpdateIndex).updateAlerts();
        faultUpdateIndex = (faultUpdateIndex + 1) % sparkFaultAlerts.size();
    }
}