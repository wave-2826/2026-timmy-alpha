from evdev import InputDevice, categorize, ecodes, UInput, AbsInfo
import time

# Run `python3 -m evdev.evtest` to find - look for something like "AT Translated Set 2 keyboard"
# Note that the first one you see may not be the right device!
KEYBOARD_PATH = "/dev/input/event17"

dev = InputDevice(KEYBOARD_PATH)

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

guid = "030000005e0400001907000000010000"
guid_bytes = bytes.fromhex(guid)
# Parse out bus type, vendor, product, version from guid
bus_type = int.from_bytes(guid_bytes[0:4], "little")
vendor = int.from_bytes(guid_bytes[4:8], "little")
product = int.from_bytes(guid_bytes[8:12], "little")
version = int.from_bytes(guid_bytes[12:16], "little")

print(f"Creating virtual controller with bus_type={bus_type}, vendor={vendor:#06x}, product={product:#06x}, version={version:#06x}")

CONTROLLERS = 2

# has to contain "pad" or else wpilib maps it differentlly SOB

controllers = [
    UInput(capabilities, name=f"Sim Xbox pad {i}", vendor=vendor, product=product, version=version, bustype=bus_type)
    for i in range(CONTROLLERS)
]

# 030000005e0400000a0b000005040000
# 030000005e0400000a0b000005040000

key_map = {
    ecodes.KEY_W: ("axis_left_y", -32768),
    ecodes.KEY_S: ("axis_left_y", 32767),
    ecodes.KEY_A: ("axis_left_x", -32768),
    ecodes.KEY_D: ("axis_left_x", 32767),

    ecodes.KEY_I: ("axis_right_y", -32768),
    ecodes.KEY_K: ("axis_right_y", 32767),
    ecodes.KEY_J: ("axis_right_x", -32768),
    ecodes.KEY_L: ("axis_right_x", 32767),

    ecodes.KEY_Q: ("axis_left_trigger", 255),
    ecodes.KEY_E: ("axis_right_trigger", 255),

    ecodes.KEY_T: [("button", ecodes.BTN_DPAD_UP), ("axis_dpad_y", -1)],
    ecodes.KEY_G: [("button", ecodes.BTN_DPAD_DOWN), ("axis_dpad_y", 1)],
    ecodes.KEY_F: [("button", ecodes.BTN_DPAD_LEFT), ("axis_dpad_x", -1)],
    ecodes.KEY_H: [("button", ecodes.BTN_DPAD_RIGHT), ("axis_dpad_x", 1)],

    ecodes.KEY_Z: ("button", ecodes.BTN_A),
    ecodes.KEY_B: ("button", ecodes.BTN_B),
    ecodes.KEY_X: ("button", ecodes.BTN_X),
    ecodes.KEY_Y: ("button", ecodes.BTN_Y),

    ecodes.KEY_U: ("button", ecodes.BTN_TL),
    ecodes.KEY_O: ("button", ecodes.BTN_TR),

    ecodes.KEY_1: ("button", ecodes.BTN_SELECT),
    ecodes.KEY_2: ("button", ecodes.BTN_START),

    ecodes.KEY_F1: ("toggle", None),
    ecodes.KEY_F2: ("toggle", 0),
    ecodes.KEY_F3: ("toggle", 1)
}

class Interpolator:
    def __init__(self):
        self.value = 0
    def set_target(self, target):
        self.value = target
    def update(self):
        pass
    def __int__(self):
        return int(self.value)
    def __str__(self):
        return f"{int(self.value):6d}"

class SmoothInterpolator(Interpolator):
    def __init__(self):
        self.value = 0
        self.target = 0
        self.start_time = time.time()
        self.duration = 0.1  # seconds

    def set_target(self, target):
        self.target = target
        self.start_time = time.time()

    def update(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.value = self.target
        else:
            t = elapsed / self.duration
            self.value += (self.target - self.value) * t
    
    def get_eased(self):
        # Simple ease-out cubic
        t = min(max((time.time() - self.start_time) / self.duration, 0), 1)
        eased_t = 1 - (1 - t) ** 3
        return int(self.value + (self.target - self.value) * eased_t)

    def __int__(self):
        return self.get_eased()

axis_state = {
    "axis_left_x": SmoothInterpolator(),
    "axis_left_y": SmoothInterpolator(),
    "axis_left_trigger": SmoothInterpolator(),
    "axis_right_trigger": SmoothInterpolator(),
    "axis_right_x": SmoothInterpolator(),
    "axis_right_y": SmoothInterpolator(),
    "axis_dpad_x": SmoothInterpolator(),
    "axis_dpad_y": SmoothInterpolator()
}
button_state = {}

print("Running... press Ctrl+C to stop")

def map_name(key, action):
    # String like "key(Z) -> btn(A): "
    button_names = ecodes.BTN[action[1]]
    if isinstance(button_names, tuple):
        button_names = [name.replace("BTN_", "") for name in button_names]
    else:
        button_names = [button_names.replace("BTN_", "")]
    # Sort to find the shortest
    button_name = min(button_names, key=len)
    return f"key({ecodes.KEY[key].replace("KEY_", "")})->btn({button_name})".ljust(20)
def highlight_bool(value):
    return f"\033[1m\033[32m{value}\033[0m" if value else f"\033[2m\033[31m{value}\033[0m"
def highlight_int(value, max = 32768, min = -32768):
    value = int(value)
    # Fancy progress bar background color oooh
    bar_length = 8
    filled_length = round(bar_length * (value - min) / (max - min))
    value_str = f" {value:6d} "
    return f"\033[42m{value_str[:filled_length]}\033[0m\033[2m{value_str[filled_length:]}\033[0m"

def columnize(lines, col_width=40, start_indent=""):
    # Simple columnizer for button states
    lines = list(lines)
    cols = (len(lines) + 1) // 2
    left = lines[:cols]
    right = lines[cols:]
    result = ""
    for l, r in zip(left, right + [""] * (len(left) - len(right))):
        result += f"{start_indent}{l.ljust(col_width)}{r}\n"
    return result

active_controller = None
grabbed = False

while True:
    ui = active_controller

    try:
        for event in dev.read():
            if event.type != ecodes.EV_KEY:
                continue

            key_event = categorize(event)
            key = key_event.scancode

            if key not in key_map:
                continue
            
            actions = key_map[key]

            for action in (actions if isinstance(actions, list) else [actions]):
                if action[0] == "toggle":
                    if key_event.keystate == key_event.key_down:
                        active_controller = controllers[action[1]] if action[1] is not None else None

                        # Eat events if enabled
                        should_grab = active_controller != None

                        if should_grab != grabbed:
                            if should_grab:
                                dev.grab()
                            else:
                                dev.ungrab()
                            grabbed = should_grab
                elif active_controller is not None and action[0].startswith("axis"):
                    if key_event.keystate == key_event.key_down:
                        axis_state[action[0]].set_target(action[1])
                    elif key_event.keystate == key_event.key_up:
                        axis_state[action[0]].set_target(0)
                elif active_controller is not None and action[0] == "button":
                    value = 1 if key_event.keystate == key_event.key_down or\
                        key_event.keystate == key_event.key_hold else 0
                    ui.write(ecodes.EV_KEY, action[1], value)
                    button_state[action[1]] = value
                    ui.syn()
    except BlockingIOError:
        pass
    
    ui = active_controller

    # Update interpolators and send axis values
    for axis, interp in axis_state.items():
        interp.update()
    if active_controller is not None:
        ui.write(ecodes.EV_ABS, ecodes.ABS_X, int(axis_state["axis_left_x"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_Y, int(axis_state["axis_left_y"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_Z, int(axis_state["axis_left_trigger"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_RX, int(axis_state["axis_right_x"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_RY, int(axis_state["axis_right_y"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_RZ, int(axis_state["axis_right_trigger"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_HAT0X, int(axis_state["axis_dpad_x"]))
        ui.write(ecodes.EV_ABS, ecodes.ABS_HAT0Y, int(axis_state["axis_dpad_y"]))
        ui.syn()

    # Print the controller state to the bottom line
    state_str = (
        f"Active controller: {active_controller.name if active_controller is not None else "None"} {"(press F1 to disable and ungrab!!)" if active_controller is not None else "(when enabling a controller, keyboard input will be eaten!)"}\n"
        f"Left Stick: ({highlight_int(axis_state['axis_left_x'])}, {highlight_int(axis_state['axis_left_y'])})  |  "
        f"Right Stick: ({highlight_int(axis_state['axis_right_x'])}, {highlight_int(axis_state['axis_right_y'])}) \n"
        f"Triggers: ({highlight_int(axis_state['axis_left_trigger'], 255, 0)}, {highlight_int(axis_state['axis_right_trigger'], 255, 0)})  |  "
        f"D-pad: ({highlight_int(axis_state['axis_dpad_x'], 1, -1)}, {highlight_int(axis_state['axis_dpad_y'], 1, -1)})\n"
        "Buttons:\n"
        + columnize(
            (f"{map_name(key, action)}: {highlight_bool(button_state.get(action[1], 0))}"
            for key, action in key_map.items()
            if action[0] == "button"),
            50,
            "  "
        )
    )

    ANSI_CLEAR_SCREEN = "\033[2J"
    ANSI_CURSOR_HOME = "\033[H"
    print(ANSI_CLEAR_SCREEN + ANSI_CURSOR_HOME + state_str + "      ", end="", flush=True)

    time.sleep(0.01)

PATH="/dev/input/event2"