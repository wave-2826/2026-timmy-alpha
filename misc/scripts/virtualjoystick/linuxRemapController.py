from evdev import InputDevice, categorize, ecodes, UInput, AbsInfo
import time

# Run `python3 -m evdev.evtest` to find - look for something like "AT Translated Set 2 keyboard"
# Note that the first one you see may not be the right device!
dev = InputDevice("/dev/input/event19")

guid = "030000005e0400001907000000010000"
guid_bytes = bytes.fromhex(guid)
# Parse out bus type, vendor, product, version from guid
bus_type = int.from_bytes(guid_bytes[0:4], "little")
vendor = int.from_bytes(guid_bytes[4:8], "little")
product = int.from_bytes(guid_bytes[8:12], "little")
version = int.from_bytes(guid_bytes[12:16], "little")

print(f"Creating virtual controller with bus_type={bus_type}, vendor={vendor:#06x}, product={product:#06x}, version={version:#06x}")

CONTROLLERS = 2


# virtual controller capabilities
capabilities = {
    # See https://github.com/torvalds/linux/blob/master/drivers/input/joystick/xpad.c
    # for how xbox controllers are handled
    # note that this is different(!) than what wpilib expects - the controller mappings
    # are wrong by default on linux! we change them to be like wpilib.
    # https://github.com/wpilibsuite/allwpilib/blob/7ca35e5678cf32caec6a1a866ca51d0136c4c398/simulation/halsim_gui/src/main/native/cpp/DriverStationGui.cpp#L421

    ecodes.EV_KEY: [
        ecodes.BTN_A,       # A
        ecodes.BTN_B,       # B
        ecodes.BTN_X,       # X
        ecodes.BTN_Y,       # Y
        ecodes.BTN_TL,      # L1
        ecodes.BTN_TR,      # R1
        ecodes.BTN_THUMBL,  # Left Stick Press
        ecodes.BTN_THUMBR,  # Right Stick Press
        ecodes.BTN_SELECT,  # Select
        ecodes.BTN_START,   # Start
        # D-pad
        ecodes.BTN_DPAD_UP,
        ecodes.BTN_DPAD_DOWN,
        ecodes.BTN_DPAD_LEFT,
        ecodes.BTN_DPAD_RIGHT,

        # Back buttons - we don't use these but they're required for glfw to recognize the
        # controller properly
        ecodes.BTN_MODE    # Xbox button
    ],
    ecodes.EV_ABS: [
        # Left stick
        (ecodes.ABS_X, AbsInfo(0, -32768, 32767, 0, 0, 0)),
        (ecodes.ABS_Y, AbsInfo(0, -32768, 32767, 0, 0, 0)),
        # Triggers
        (ecodes.ABS_Z, AbsInfo(0, 0, 255, 0, 0, 0)),
        (ecodes.ABS_RZ, AbsInfo(0, 0, 255, 0, 0, 0)),
        # Right stick
        (ecodes.ABS_RX, AbsInfo(0, -32768, 32767, 0, 0, 0)),
        (ecodes.ABS_RY, AbsInfo(0, -32768, 32767, 0, 0, 0)),

        (ecodes.ABS_HAT0X, AbsInfo(0, -1, 1, 0, 0, 0)),  # D-pad X
        (ecodes.ABS_HAT0Y, AbsInfo(0, -1, 1, 0, 0, 0))   # D-pad Y
    ]
}

controller = UInput(capabilities, name=f"Remapped Xbox controller pad", vendor=vendor, product=product, version=version, bustype=bus_type)

while True:
    ui = controller

    try:
        for event in dev.read():
            print(event)
            # Forward events
            if event.type == ecodes.EV_KEY:
                ui.write(ecodes.EV_KEY, event.code, event.value)
            elif event.type == ecodes.EV_ABS:
                ui.write(ecodes.EV_ABS, event.code, event.value)
            ui.syn()
    except Exception:
        pass

    time.sleep(0.01)