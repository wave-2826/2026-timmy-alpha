import vlogger
import urllib
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

logs = [
    # ("../../../dlogs/akit_26-04-18_15-56-45_wicmp_q2.wpilog", "Quals 2"),
    ("../../../dlogs/akit_26-04-18_14-59-19_wicmp_p12.wpilog", "Practice 12"),
    ("../../../dlogs/akit_26-04-18_20-27-23_wicmp_q25.wpilog", "Quals 25"),
]

@dataclass
class LogResult:
    power_sum_per_channel: list[float]
    start_offset: float
    power_integral: list[float]
    brownout_timestamps: list[float]

log_results: list[LogResult] = []

# log_power_sums = [
#     [0 for _ in range(24)] for _ in logs
# ]
# start_offsets = [0 for _ in logs]
# log_power_integrals = []

def joulesToWattHours(j: float):
    return j / 3600

for log_idx, log in enumerate(logs):
    source = vlogger.get_source(f"wpilog://{log[0]}", "/PowerDistribution|/DriverStation/Enabled|/SystemStats/BrownedOut")

    voltages = TimedData()
    currents = [TimedData() for i in range(24)]

    start_offset = 0

    brownout_timestamps = []
    prev_browned_out = False

    for field in source:
        if field["name"].endswith("Voltage"):
            voltages.add(field["timestamp"], field["data"])
        if field["name"].endswith("ChannelCurrent"):
            for i, current in enumerate(field["data"]):
                currents[i].add(field["timestamp"], current)
        if field["name"].endswith("Enabled") and field["data"] == True and start_offset == 0:
            start_offset = field["timestamp"]
        
        if field["name"].endswith("BrownedOut"):
            browned_out = field["data"]
            if browned_out and not prev_browned_out:
                brownout_timestamps.append(field["timestamp"])
            prev_browned_out = browned_out
    
    channel_power_sums = [0 for _ in range(24)]

    for i, current in enumerate(currents):
        for ts, val in zip(current.timestamps, current.values):
            voltage = voltages.get_nearest(ts)
            if voltage is not None:
                power = joulesToWattHours(voltage * val * 0.02)
                channel_power_sums[i] += power

        print(f"Channel {i}: Total Energy = {channel_power_sums[i]:.2f} Wh")
    
    power_integral = []

    for ts in currents[0].timestamps:
        total_power = 0
        for i, current in enumerate(currents):
            voltage = voltages.get_nearest(ts)
            if voltage is not None:
                total_power += joulesToWattHours(voltage * current.get_nearest(ts) * 0.02)
        
        power_integral.append((ts, (power_integral[-1] if len(power_integral) > 0 else (0, 0))[1] + total_power))
    
    log_results.append(LogResult(channel_power_sums, start_offset, power_integral, brownout_timestamps))

# Plot
import matplotlib.pyplot as plt
plt.figure(figsize=(12, 6))
bar_width = 0.5 / len(logs)
for log_idx, (log_path, log_name) in enumerate(logs):
    result = log_results[log_idx]
    total_power = sum(result.power_sum_per_channel)
    plt.bar([x + log_idx * bar_width for x in range(24)], result.power_sum_per_channel, width=bar_width, label=f"{log_name} (Total: {total_power:.2f} Wh, {len(result.brownout_timestamps)} brownouts)")
plt.xlabel("Channel")
plt.ylabel("Total Energy (Joules)")
plt.title("Total Energy per Channel")
plt.legend()
plt.xticks(range(24))
plt.grid(axis="y", linestyle="--", alpha=0.7)

plt.figure(figsize=(12, 6))
for log_idx, (log_path, log_name) in enumerate(logs):
    result = log_results[log_idx]
    timestamps, energies = zip(*result.power_integral)
    plt.plot([ts - result.start_offset for ts in timestamps], energies, label=f"{log_name} (Total: {energies[-1]:.2f} Wh, {len(result.brownout_timestamps)} brownouts)")
    # Plot brownouts as red dots
    for brownout_ts in result.brownout_timestamps:
        plt.plot(brownout_ts - result.start_offset, energies[-1], 'ro', label=f"{log_name} Brownout" if brownout_ts == result.brownout_timestamps[0] else "")

plt.xlabel("Time (s)")
plt.ylabel("Cumulative Energy (Wh)")
plt.title("Cumulative Energy over Time")
plt.legend()
plt.grid(axis="both", linestyle="--", alpha=0.7)

plt.show()