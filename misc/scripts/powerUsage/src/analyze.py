
from dataclasses import dataclass
import os
import pickle
from .subsystems import SubsystemMap
import vlogger
from .timed_data import TimedBooleanData, TimedNumericData

CACHE_VER = 2

@dataclass
class SubsystemResult:
    power_sum: float
    amperage_sum: float
    average_current_while_enabled: float
    percentage_total_power: float
    percentage_total_current: float

@dataclass
class LogResult:
    name: str
    file_last_modified: float
    version: float

    power_sum_per_channel: list[float]
    amperage_sum_per_channel: list[float]
    start_offset: float
    power_integral: TimedNumericData
    amperage_integral: TimedNumericData
    brownout_timestamps: list[float]
    average_voltage_while_enabled: float
    average_current_while_enabled: float

    subsystem_results: dict[str, SubsystemResult]

    def integral(self, ty: str) -> TimedNumericData:
        if ty == "power":
            return self.power_integral
        return self.amperage_integral

def joulesToWattHours(j: float):
    return j / 3600

########### Cache

CACHE_DIR = "cache/"
def find_cache_result(original_log_path: str) -> LogResult | None:
    log_modified_time = os.path.getmtime(original_log_path)

    cache_path = CACHE_DIR + original_log_path.split('/')[-1] + ".cache"
    if os.path.exists(cache_path):
        with open(cache_path, "rb") as f:
            result = pickle.load(f)
            if result.file_last_modified == log_modified_time and result.version == CACHE_VER:
                print(f"Cache hit for {original_log_path}")
                return result
    return None

def save_cache_result(log_path: str, result: LogResult):
    os.makedirs(CACHE_DIR, exist_ok=True)
    cache_path = CACHE_DIR + log_path.split('/')[-1] + ".cache"
    with open(cache_path, "wb") as f:
        pickle.dump(result, f)

########### Analysis

def analyze_log(log: tuple[str, str, SubsystemMap]):
    print(f"Analyzing log {log[0]}")

    if cached_result := find_cache_result(log[0]):
        return cached_result
    
    log_modified_time = os.path.getmtime(log[0])
    source = vlogger.get_source(f"wpilog://{log[0]}", "/PowerDistribution|/DriverStation/Enabled|/DriverStation/Autonomous|/SystemStats/BrownedOut")

    voltages = TimedNumericData()
    currents = [TimedNumericData() for _ in range(24)]
    total_current = TimedNumericData()

    enabled = TimedBooleanData()

    start_offset = 0

    brownout_timestamps = []
    prev_browned_out = False
    autonomous = False

    for field in source:
        ts = field["timestamp"] / 1e6
        if field["name"].endswith("Voltage"):
            voltages.add(ts, field["data"])
        if field["name"].endswith("TotalCurrent") and enabled:
            total_current.add(ts, field["data"])
        if field["name"].endswith("ChannelCurrent"):
            for i, current in enumerate(field["data"]):
                currents[i].add(ts, current)
        if field["name"].endswith("Enabled"):
            enabled.add(ts, field["data"])
        if field["name"].endswith("Autonomous"):
            autonomous = field["data"]
        if enabled.last_or(False) and autonomous and start_offset == 0:
            start_offset = ts
        
        if field["name"].endswith("BrownedOut"):
            browned_out = field["data"]
            if browned_out and not prev_browned_out:
                brownout_timestamps.append(ts)
            prev_browned_out = browned_out

    total_power = total_current * voltages
    channel_power_sums = [0 for _ in range(24)]
    channel_amperage_sums = [0 for _ in range(24)]

    for i, current in enumerate(currents):
        channel_power_sums[i] = joulesToWattHours((current * voltages).integrate())
        channel_amperage_sums[i] = current.integrate()

        print(f"[{log[1]}] Channel {i}: Total Energy = {channel_power_sums[i]:.2f} Wh")
    
    power_integral = total_power.integral().map(joulesToWattHours)
    amperage_integral = total_current.integral()

    subsystem_results: dict[str, SubsystemResult] = {}
    for subsystem, ports in log[2].subsystems.items():
        power_sum = sum(channel_power_sums[port] for port in ports)
        amperage_sum = sum(channel_amperage_sums[port] for port in ports)
        average_current_while_enabled = sum(currents[port].average_filtered(enabled) for port in ports)
        subsystem_results[subsystem] = SubsystemResult(
            power_sum=power_sum,
            amperage_sum=amperage_sum,
            average_current_while_enabled=average_current_while_enabled,
            percentage_total_current=power_sum / sum(channel_power_sums) if sum(channel_power_sums) > 0 else 0,
            percentage_total_power=amperage_sum / sum(channel_amperage_sums) if sum(channel_amperage_sums) > 0 else 0
        )
    
    result = LogResult(
        name=log[1],
        version=CACHE_VER,
        file_last_modified=log_modified_time,
        power_sum_per_channel=channel_power_sums,
        amperage_sum_per_channel=channel_amperage_sums,
        start_offset=start_offset,
        power_integral=power_integral,
        amperage_integral=amperage_integral,
        brownout_timestamps=brownout_timestamps,
        average_voltage_while_enabled=voltages.average_filtered(enabled),
        average_current_while_enabled=total_current.average_filtered(enabled),
        subsystem_results=subsystem_results
    )
    save_cache_result(log[0], result)
    return result