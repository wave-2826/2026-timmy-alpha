import vlogger
import urllib

split = urllib.parse.urlsplit("wpilog://./log.wpilog")
print(split.path.lstrip('/'))

# "" regex matches with anything, i.e. any field
source = vlogger.get_source("wpilog://./log.wpilog", "/PowerDistribution")

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

voltages = TimedData()
currents = [TimedData() for i in range(24)]

for field in source:
    if field["name"].endswith("Voltage"):
        voltages.add(field["timestamp"], field["data"])
    if field["name"].endswith("ChannelCurrent"):
        for i, current in enumerate(field["data"]):
            currents[i].add(field["timestamp"], current)

power_sums = [0 for _ in range(24)]

for i, current in enumerate(currents):
    for ts, val in zip(current.timestamps, current.values):
        voltage = voltages.get_nearest(ts)
        if voltage is not None:
            power_sums[i] += voltage * val / 0.92

    print(f"Channel {i}: Total Energy = {power_sums[i]} J")

# Plot
import matplotlib.pyplot as plt
plt.figure(figsize=(12, 6))
plt.bar(range(24), power_sums)
plt.xlabel("Channel")
plt.ylabel("Total Energy (Joules)")
plt.title("Total Energy per Channel")
plt.xticks(range(24))
plt.grid(axis="y", linestyle="--", alpha=0.7)
plt.show()