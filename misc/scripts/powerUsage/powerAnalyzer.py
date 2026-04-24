import vlogger
import os
from dataclasses import dataclass

class TimedData:
    timestamps: list[float]
    values: list[float]

    def __init__(self):
        self.timestamps = []
        self.values = []

    def add(self, ts: float, val: float):
        self.timestamps.append(ts)
        self.values.append(val)
    
    def get_nearest(self, ts: float):
        # Binary search for nearest timestamp
        left, right = 0, len(self.timestamps) - 1
        while left <= right:
            mid = (left + right) // 2
            if self.timestamps[mid] < ts:
                left = mid + 1
            else:
                right = mid - 1
        
        # Check neighbors to find closest timestamp
        candidates = []
        if left < len(self.timestamps):
            candidates.append((abs(self.timestamps[left] - ts), self.values[left]))
        if right >= 0:
            candidates.append((abs(self.timestamps[right] - ts), self.values[right]))
        
        if not candidates:
            return None
        
        return min(candidates, key=lambda x: x[0])[1]

    def get_last_value(self, default: float):
        if self.values:
            return self.values[-1]
        return default

LOGS_BASE = "../../dlogs/"

def extract_name(log_file: str):
    name = log_file[:-7].split('_')[-1]
    if name.startswith("q"):
        name = "Quals " + name[1:]
    elif name.startswith("p"):
        name = "Practice " + name[1:]
    elif name.startswith("e"):
        name = "Elims " + name[1:]
    return name

def enumerate_logs(base_path: str = LOGS_BASE):
    logs: list[tuple[str, str]] = []

    for entry in os.listdir(base_path):
        if entry.endswith(".wpilog"):
            path = os.path.join(base_path, entry)
            logs.append((path, extract_name(entry)))
    return logs

def find_log(match: str):
    for entry in os.listdir(LOGS_BASE):
        if match in entry:
            path = os.path.join(LOGS_BASE, entry)
            return (path, extract_name(entry))

# logs = [
# #     # ("../../dlogs/akit_26-04-18_15-56-45_wicmp_q2.wpilog", "Quals 2"),
# #     ("../../dlogs/akit_26-04-18_14-59-19_wicmp_p12.wpilog", "Practice 12"),
# #     # ("../../dlogs/akit_26-04-18_22-32-06_wicmp_q37.wpilog", "Quals 37"),
#     find_log("q53"),
#     find_log("q57")
# ]
logs = enumerate_logs()

@dataclass
class LogResult:
    power_sum_per_channel: list[float]
    amperage_sum_per_channel: list[float]
    start_offset: float
    power_integral: TimedData
    amperage_integral: TimedData
    brownout_timestamps: list[float]
    average_voltage_while_enabled: float
    average_current_while_enabled: float

    def integral(self, ty: str) -> TimedData:
        if ty == "power":
            return self.power_integral
        return self.amperage_integral

def joulesToWattHours(j: float):
    return j / 3600

########### Analysis

def analyze_log(log: tuple[str, str]):
    print(f"Analyzing log {log[0]}")
    
    source = vlogger.get_source(f"wpilog://../{log[0]}", "/PowerDistribution|/DriverStation/Enabled|/SystemStats/BrownedOut")

    voltages = TimedData()
    currents = [TimedData() for _ in range(24)]

    start_offset = 0

    brownout_timestamps = []
    prev_browned_out = False
    enabled = False

    enabled_voltage_sum = 0
    enabled_voltage_count = 0

    enabled_current_sum = 0
    enabled_current_count = 0

    for field in source:
        ts = field["timestamp"] / 1e6
        if field["name"].endswith("Voltage"):
            voltages.add(ts, field["data"])
            if enabled:
                enabled_voltage_sum += field["data"]
                enabled_voltage_count += 1
        if field["name"].endswith("TotalCurrent") and enabled:
            enabled_current_sum += field["data"]
            enabled_current_count += 1
        if field["name"].endswith("ChannelCurrent"):
            for i, current in enumerate(field["data"]):
                currents[i].add(ts, current)
        if field["name"].endswith("Enabled"):
            enabled = field["data"]
            if field["data"] == True and start_offset == 0:
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
    
    power_integral = TimedData()
    amperage_integral = TimedData()

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

        next_power = power_integral.get_last_value(0) + total_power
        power_integral.add(ts, next_power)
        
        next_amperage = amperage_integral.get_last_value(0) + total_amperage
        amperage_integral.add(ts, next_amperage)
        
    average_voltage = enabled_voltage_sum / enabled_voltage_count if enabled_voltage_count > 0 else 0
    average_current = enabled_current_sum / enabled_current_count if enabled_current_count > 0 else 0
    return LogResult(
        channel_power_sums, channel_amperage_sums, start_offset, power_integral, amperage_integral, brownout_timestamps,
        average_voltage, average_current)

def plot_integrals(ty, units):
    # Power integral plot
    plt.figure(figsize=(12, 6))

    # Draw a light red box in the background for auto and a light green box for teleop
    max_energy = max(result.integral(ty).get_last_value(0) for result in log_results) * 1.1
    plt.gca().add_patch(plt.Rectangle((0, 0), 20, max_energy, facecolor="red", alpha=0.1))
    plt.gca().add_patch(plt.Rectangle((23, 0), 140, max_energy, facecolor="green", alpha=0.1))

    for log_idx, (log_path, log_name) in enumerate(logs):
        result = log_results[log_idx]
        timestamps = result.integral(ty).timestamps
        energies = result.integral(ty).values
        plt.plot([ts - result.start_offset for ts in timestamps], energies, label=f"{log_name} (Total: {energies[-1]:.2f} {units}, {len(result.brownout_timestamps)} brownouts)")
        # Plot brownouts as red dots
        for brownout_ts in result.brownout_timestamps:
            plt.plot(brownout_ts - result.start_offset, [
                result.integral(ty).get_nearest(brownout_ts) or 0
            ], 'ro', "", markersize=2)

    plt.legend()
    plt.grid(axis="both", linestyle="--", alpha=0.7)

if __name__ == '__main__':
    log_results: list[LogResult] = []

    import concurrent.futures
    with concurrent.futures.ProcessPoolExecutor() as executor:
        log_results.extend(executor.map(analyze_log, logs))

    ########### Plotting

    print("Plotting results...")

    # Channel plot
    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 6))
    bar_width = 0.8 / len(logs)
    for log_idx, (log_path, log_name) in enumerate(logs):
        result = log_results[log_idx]
        total_power = sum(result.power_sum_per_channel)
        plt.bar([x + log_idx * bar_width for x in range(24)], result.power_sum_per_channel, width=bar_width, label=f"{log_name} (Total: {total_power:.2f} Wh, {len(result.brownout_timestamps)} brownouts)")
    plt.xlabel("Channel")
    plt.ylabel("Total Energy (Wh)")
    plt.title("Total Energy per Channel")
    plt.legend()
    plt.xticks(range(24))
    plt.grid(axis="y", linestyle="--", alpha=0.7)

    # Per-log plot
    plt.figure(figsize=(12, 6))
    max_brownouts = max(len(result.brownout_timestamps) for result in log_results) + 1
    ax1 = plt.gca()
    ax2 = ax1.twinx()
    x = range(len(logs))
    width = 0.4
    max_current = max(result.average_current_while_enabled for result in log_results)
    min_current = min(result.average_current_while_enabled for result in log_results)

    for log_idx, (log_path, log_name) in enumerate(logs):
        result = log_results[log_idx]
        total_power = sum(result.power_sum_per_channel)
        ax1.bar(
            x[log_idx] - width/2, total_power, width=width,
            label=f"{log_name} (Total: {total_power:.2f} Wh, {len(result.brownout_timestamps)} brownouts)",
            color=plt.cm.RdYlGn_r(min(len(result.brownout_timestamps) / max_brownouts, 1.0))
        )
        ax2.bar(
            x[log_idx] + width/2, result.average_current_while_enabled, width=width*0.5,
            color="#ffff55"
        )

    plt.xlabel("Log")
    plt.title("Total Energy and Average Current per Log")
    plt.legend()
    ax1.set_xticks(x)
    ax1.set_xticklabels([log_name for _, log_name in logs])
    ax1.set_ylabel("Total Energy (Wh)")
    ax2.set_ylabel("Average Current While Enabled (A)")
    # ax2.set_ylim(0, 14)

    plot_integrals("power", "Wh")
    plt.xlabel("Time (s)")
    plt.ylabel("Cumulative Energy (Wh)")
    plt.yscale("linear")
    plt.title("Cumulative Energy over Time")
    
    # plot_integrals("amperage", "As")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Cumulative Amperage (As)")
    # plt.yscale("linear")
    # plt.title("Cumulative Amperage over Time")
    
    plt.show()