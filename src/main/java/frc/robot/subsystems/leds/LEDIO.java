package frc.robot.subsystems.leds;

public interface LEDIO {
    /**
     * @param colors Colors in RGB order
     */
    public default void pushLEDs(int[] colors) {
    }
}
