import os

from .subsystems import SubsystemMap
from .analyze import LogResult, analyze_log
from .plot import plot

LOGS_BASE = os.path.join(os.path.dirname(__file__), "../../../dlogs/")

def extract_name(log_file: str):
    # e.g. "xxx_wicmp_e1.wpilog" -> "Wicmp Elims 1"
    name = log_file[:-7].split('_')[-1]
    if name.startswith("q"):
        name = "Quals " + name[1:]
    elif name.startswith("p"):
        name = "Practice " + name[1:]
    elif name.startswith("e"):
        name = "Elims " + name[1:]
    event = log_file[:-7].split('_')[-2]
    return event.capitalize() + " " + name

def enumerate_logs(base_path: str = LOGS_BASE):
    logs: list[tuple[str, str]] = []

    for entry in os.listdir(base_path):
        if entry.endswith(".wpilog"):
            path = os.path.join(base_path, entry)
            logs.append((path, extract_name(entry), get_subsystem_map_for_time(entry.split('_')[1])))
    return logs

def find_log(match: str):
    for entry in os.listdir(LOGS_BASE):
        if match in entry:
            path = os.path.join(LOGS_BASE, entry)
            return (path, extract_name(entry), get_subsystem_map_for_time(entry.split('_')[1]))

# (before time, PDH map)
def get_subsystem_map_for_time(timestamp: str) -> SubsystemMap:
    for time, subsystem_map in subsystem_maps:
        if time is not None and timestamp <= time:
            return subsystem_map
    return subsystem_maps[-1][1]
subsystem_maps = [
    ("26-04-18_23-23-17", SubsystemMap({
        "drivetrain": [0, 1, 2, 3, 16, 17, 18, 19],
        "turret": [4, 5, 6, 7],
        "coprocessors/leds": [9],
        "intake": [10, 11, 14, 15],
        "spindexer": [12, 13],
        "rio/radio": [20, 22]
    })),
    (None, SubsystemMap({
        "drivetrain": [0, 1, 5, 6, 16, 17, 18, 19],
        "turret": [4, 2, 3, 7],
        "coprocessors/leds": [9],
        "intake": [10, 11, 14, 15],
        "spindexer": [12, 13],
        "rio/radio": [20, 22]
    }))
]

# logs = [
#     # find_log("q53"),
#     find_log("q85"),
#     find_log("q97")
# ]
logs = enumerate_logs()

def main():
    log_results: list[LogResult] = []

    import concurrent.futures
    with concurrent.futures.ProcessPoolExecutor() as executor:
        log_results.extend(executor.map(analyze_log, logs))

    plot(log_results)

if __name__ == '__main__': 
    main()