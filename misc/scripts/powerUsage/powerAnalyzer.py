import vlogger
import urllib

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
    # ("../../../dlogs/akit_26-04-18_14-59-19_wicmp_p12.wpilog", "Practice 12"),
    ("../../../dlogs/akit_26-04-18_15-56-45_wicmp_q2.wpilog", "Quals 2"),
    ("../../../dlogs/akit_26-04-18_16-47-42_wicmp_q8.wpilog", "Quals 8")
]


log_power_sums = [
    [0 for _ in range(24)] for _ in logs
]

for log_idx, log in enumerate(logs):
    source = vlogger.get_source(f"wpilog://{log[0]}", "/PowerDistribution")

    voltages = TimedData()
    currents = [TimedData() for i in range(24)]

    for field in source:
        if field["name"].endswith("Voltage"):
            voltages.add(field["timestamp"], field["data"])
        if field["name"].endswith("ChannelCurrent"):
            for i, current in enumerate(field["data"]):
                currents[i].add(field["timestamp"], current)

    for i, current in enumerate(currents):
        for ts, val in zip(current.timestamps, current.values):
            voltage = voltages.get_nearest(ts)
            if voltage is not None:
                log_power_sums[log_idx][i] += voltage * val / 0.92

        print(f"Channel {i}: Total Energy = {log_power_sums[log_idx][i]:.2f} Joules")

# Plot
import matplotlib.pyplot as plt
plt.figure(figsize=(12, 6))
bar_width = 0.4 / len(logs)
for log_idx, (log_path, log_name) in enumerate(logs):
    plt.bar([x + log_idx * bar_width for x in range(24)], log_power_sums[log_idx], width=bar_width, label=log_name)
plt.xlabel("Channel")
plt.ylabel("Total Energy (Joules)")
plt.title("Total Energy per Channel")
plt.legend()
plt.xticks(range(24))
plt.grid(axis="y", linestyle="--", alpha=0.7)
plt.show()