package frc.robot.util;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/** Handles alerts related to the RoboRIO (e.g. shorted rails, CAN issues, etc.) */
public class RioAlerts {
    private final Alert faulted6VRailAlert = new Alert("RoboRIO: Faulted 6V rail!", AlertType.kWarning);
    private final Alert faulted5VRailAlert = new Alert("RoboRIO: Faulted 5V rail!", AlertType.kWarning);
    private final Alert faulted3V3RailAlert = new Alert("RoboRIO: Faulted 3.3V rail!", AlertType.kWarning);

    private final Alert disabled6VRailAlert = new Alert("RoboRIO: Disabled 6V rail!", AlertType.kError);
    private final Alert disabled5VRailAlert = new Alert("RoboRIO: Disabled 5V rail!", AlertType.kError);
    private final Alert disabled3V3RailAlert = new Alert("RoboRIO: Disabled 3.3V rail!", AlertType.kError);

    private final Alert canBusFault = new Alert("RoboRIO: CAN bus fault!", AlertType.kError);

    /**
     * The minimum voltage for the battery to be considered low when the robot is disabled.
     */
    private static final double LOW_BATTERY_VOLTAGE = 11.7;
    private final Alert lowBatteryAlert = new Alert("RoboRIO: Battery voltage low! Please replace the battery soon.",
        AlertType.kWarning);

    private static final RioAlerts instance = new RioAlerts();

    public static RioAlerts getInstance() {
        return instance;
    }

    private RioAlerts() {
        // This is a singleton class.
    }

    public boolean batteryLow() {
        return DriverStation.isDisabled() && RobotController.getBatteryVoltage() < RioAlerts.LOW_BATTERY_VOLTAGE;
    }

    /** Updates the RoboRIO alerts. */
    public void update() {
        faulted6VRailAlert.set(RobotController.getFaultCount6V() >= 1);
        faulted5VRailAlert.set(RobotController.getFaultCount5V() >= 1);
        faulted3V3RailAlert.set(RobotController.getFaultCount3V3() >= 1);

        disabled6VRailAlert.set(!RobotController.getEnabled6V());
        disabled5VRailAlert.set(!RobotController.getEnabled5V());
        disabled3V3RailAlert.set(!RobotController.getEnabled3V3());

        CANStatus status = RobotController.getCANStatus();
        canBusFault.set(status.busOffCount > 0 || status.receiveErrorCount > 0 || status.transmitErrorCount > 0);

        lowBatteryAlert.set(batteryLow());
    }
}