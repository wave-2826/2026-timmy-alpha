#!/usr/bin/env python3
"""WebSocket client for receiving LED color commands from a remote server."""

import asyncio
import json
import logging
import os

import websockets

from config import Configuration
from mock_led import Color, MockPixelStrip as PixelStrip

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

config = Configuration()

# Check if we're running on a Raspberry Pi
IS_RASPBERRY_PI = os.path.exists("/sys/firmware/devicetree/base/model")
MOCK_HARDWARE = os.environ.get("MOCK_HARDWARE", "0") == "1"

if IS_RASPBERRY_PI and not MOCK_HARDWARE:
    try:
        from rpi_ws281x import Color, PixelStrip  # type: ignore
        LED_AVAILABLE = True
    except ImportError:
        logger.warning("rpi_ws281x not available, using mock LED")
        LED_AVAILABLE = False
else:
    LED_AVAILABLE = False

if not LED_AVAILABLE:
    from mock_led import Color, MockPixelStrip as PixelStrip


class LEDController:
    """Controller for LED strip."""

    def __init__(self, num_leds: int = 60, pin: int = 18):
        """Initialize the LED controller."""
        self.num_leds = num_leds
        self.pin = pin
        self.strip = PixelStrip(
            num=num_leds,
            pin=pin,
            freq_hz=800000,
            dma=10,
            invert=False,
            brightness=255,
            channel=0,
        )
        self.strip.begin()
        logger.info(f"LED controller initialized with {num_leds} LEDs on pin {pin}")

    def set_color(self, r: int, g: int, b: int) -> None:
        """Set all LEDs to a specific RGB color.

        Args:
            r: Red component (0-255)
            g: Green component (0-255)
            b: Blue component (0-255)
        """
        # Clamp values to valid range
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))

        color = Color(r, g, b)

        for i in range(self.num_leds):
            self.strip.setPixelColor(i, color)

        self.strip.show()
        logger.info(f"LED color set to RGB({r}, {g}, {b})")

        # if not LED_AVAILABLE:
            # logger.info(self.strip.getPixels())

    def turn_off(self) -> None:
        """Turn off all LEDs."""
        self.set_color(0, 0, 0)


async def led_client(
    server_uri: str,
    alliance: str = "Blue",
) -> None:
    """Connect to WebSocket server and receive LED color commands.

    Args:
        server_uri: WebSocket server URI (e.g., "ws://localhost:8080")
        alliance: Team alliance - "Red" or "Blue"
    """
    controller = LEDController(num_leds=config.led_count, pin=config.data_pin)

    logger.info(f"Connecting to {server_uri}")
    logger.info(f"Listening for {alliance} team colors")

    try:
        async with websockets.connect(server_uri) as websocket:
            logger.info("Connected to server")

            async for message in websocket:
                try:
                    data = json.loads(message)

                    # Check for hubLed message type
                    if data.get("type") == "hubLed":
                        hub_data = data.get("data", {})

                        # Get the color for our alliance
                        if alliance in hub_data:
                            color_obj = hub_data[alliance]
                            r = color_obj.get("R", 0)
                            g = color_obj.get("G", 0)
                            b = color_obj.get("B", 0)

                            controller.set_color(r, g, b)
                        else:
                            logger.warning(
                                f"Alliance '{alliance}' not found in message. "
                                f"Available: {list(hub_data.keys())}"
                            )

                except json.JSONDecodeError as e:
                    logger.error(f"Failed to parse JSON: {e}")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")

    except websockets.exceptions.ConnectionClosed:
        logger.info("Connection closed by server")
    except Exception as e:
        logger.error(f"Connection error: {e}")
    finally:
        controller.turn_off()
        logger.info("LEDs turned off")


async def main() -> None:
    """Main entry point."""
    server_uri = config.fmsAddress
    alliance = config.HubAlliance

    await led_client(server_uri, alliance)


if __name__ == "__main__":
    while True:
        try:
            asyncio.run(main())
        except Exception as e:
            logger.error(f"Unexpected error: {e}")
            logger.info("Reconnecting in 5 seconds...")
            asyncio.run(asyncio.sleep(5))
