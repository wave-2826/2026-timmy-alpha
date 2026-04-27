class TimedNumericData:
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

    def last_or(self, default: float):
        if self.values:
            return self.values[-1]
        return default

    def sum_filtered(self, filtered_by: TimedBooleanData):
        total = 0
        for ts, val in zip(self.timestamps, self.values):
            if filtered_by.get_nearest(ts, False):
                total += val
        return total

    def average_filtered(self, filtered_by: TimedBooleanData):
        total = 0
        count = 0
        for ts, val in zip(self.timestamps, self.values):
            if filtered_by.get_nearest(ts, False):
                total += val
                count += 1
        return total / count if count > 0 else 0

class TimedBooleanData:
    timestamps: list[float]
    values: list[bool]

    def __init__(self):
        self.timestamps = []
        self.values = []

    def add(self, ts: float, val: bool):
        self.timestamps.append(ts)
        self.values.append(val)
    
    def get_nearest(self, ts: float, default: bool = False):
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
            return default
        
        return min(candidates, key=lambda x: x[0])[1]

    def last_or(self, default: bool):
        if self.values:
            return self.values[-1]
        return default