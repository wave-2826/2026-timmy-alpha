
from dataclasses import dataclass
from .subsystems import SubsystemMap
import vlogger
from .timed_data import TimedBooleanData, TimedNumericData

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

########### Analysis

def analyze_log(log: tuple[str, str, SubsystemMap]):
    print(f"Analyzing log {log[0]}")
    
    source = vlogger.get_source(f"wpilog://../{log[0]}", "/PowerDistribution|/DriverStation/Enabled|/DriverStation/Autonomous|/SystemStats/BrownedOut")

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

    channel_power_sums = [0 for _ in range(24)]
    channel_amperage_sums = [0 for _ in range(24)]

    for i, current in enumerate(currents):
        last_ts = current.timestamps[0] - 0.02 if current.timestamps else 0
        for ts, val in zip(current.timestamps, current.values):
            delta_ts = ts - last_ts
            last_ts = ts

            voltage = voltages.get_nearest(ts)
            if voltage is not None:
                power = joulesToWattHours(voltage * val * delta_ts)
                channel_power_sums[i] += power
                channel_amperage_sums[i] += val * delta_ts
        
        print(f"[{log[1]}] Channel {i}: Total Energy = {channel_power_sums[i]:.2f} Wh")
    
    power_integral = TimedNumericData()
    amperage_integral = TimedNumericData()

    last_ts = currents[0].timestamps[0] - 0.02 if current.timestamps else 0
    for ts in currents[0].timestamps:
        delta_ts = ts - last_ts
        last_ts = ts
        
        total_power = 0
        total_amperage = 0
        for i, current in enumerate(currents):
            voltage = voltages.get_nearest(ts)
            if voltage is not None:
                total_power += joulesToWattHours(voltage * current.get_nearest(ts) * delta_ts)
                total_amperage += current.get_nearest(ts) * delta_ts

        next_power = power_integral.last_or(0) + total_power
        power_integral.add(ts, next_power)
        
        next_amperage = amperage_integral.last_or(0) + total_amperage
        amperage_integral.add(ts, next_amperage)
    
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
    
    return LogResult(
        name=log[1],
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