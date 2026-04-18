package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;

/**
 * LED IO that sends pulse-train WS2812B data out of a RoboRIO DIO port. Note that this also applies to a CANdle
 * connected using pulse-train mode, which is how we currently connect our LEDs. Otherwise, we would need to send one
 * CAN message per block of same-colored LEDs to the CANdle, which is extremely inefficient.
 */
public class LEDIORio implements LEDIO {
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    public LEDIORio() {
        leds = new AddressableLED(LEDConstants.ledDIOPort);
        buffer = new AddressableLEDBuffer(LEDConstants.ledCount);

        leds.setLength(LEDConstants.ledCount);

        // Required for the CANdle
        leds.setBitTiming(350, 900, 900, 350);
        leds.setSyncTime(100);

        leds.setColorOrder(ColorOrder.kRGB);

        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void pushLEDs(int[] colors) {
        for(int i = 0; i < LEDConstants.ledCount; i++) {
            buffer.setRGB(i, colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]);
        }

        leds.setData(buffer);
    }
}