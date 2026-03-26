package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
    private static List<VirtualSubsystem> instances = new ArrayList<>();

    public VirtualSubsystem() {
        instances.add(this);
    }

    /** This method is called periodically before the command scheduler. */
    public abstract void periodic();

    /** This method is called periodically after the command scheduler. */
    public void periodicAfterScheduler() {};

    /** Run the periodic methods for all subsystems. */
    public static void beforeScheduler() {
        for (VirtualSubsystem instance : instances) {
            instance.periodic();
        }
    }

    /** Run the "after periodic" methods for all subsystems. */
    public static void afterScheduler() {
        for(VirtualSubsystem instance : instances) {
            instance.periodicAfterScheduler();
        }
    }
}