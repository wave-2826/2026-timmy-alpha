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
    
    def get(self, ts: float, threshold_to_next: float = 0.01):
        # Binary search for nearest timestamp
        left, right = 0, len(self.timestamps) - 1
        while left <= right:
            mid = (left + right) // 2
            if self.timestamps[mid] < ts:
                left = mid + 1
            else:
                right = mid - 1
        
        if right >= 0 and self.timestamps[right] - ts < threshold_to_next:
            return self.values[right]
        return self.values[left] if left < len(self.timestamps) else None


    def last_or(self, default: float):
        if self.values:
            return self.values[-1]
        return default

    def sum_filtered(self, filtered_by: TimedBooleanData):
        total = 0
        for ts, val in zip(self.timestamps, self.values):
            if filtered_by.get(ts, False):
                total += val
        return total

    def sum(self):
        return sum(self.values)

    def average_filtered(self, filtered_by: TimedBooleanData):
        total = 0
        count = 0
        for ts, val in zip(self.timestamps, self.values):
            if filtered_by.get(ts, False):
                total += val
                count += 1
        return total / count if count > 0 else 0

    def deduplicate(self, epsilon_time: float):
        new_timestamps = []
        new_values = []
        last_ts = None
        for ts, val in zip(self.timestamps, self.values):
            if last_ts is None or ts - last_ts > epsilon_time:
                new_timestamps.append(ts)
                new_values.append(val)
                last_ts = ts
        self.timestamps = new_timestamps
        self.values = new_values

    def __mul__(self, other: TimedNumericData | int | float):
        if isinstance(other, (int, float)):
            new = TimedNumericData()
            for ts, val in zip(self.timestamps, self.values):
                new.add(ts, val * other)
            return new

        new = TimedNumericData()
        for ts, val in zip(self.timestamps, self.values):
            new.add(ts, val * other.get(ts))
        for ts, val in zip(other.timestamps, other.values):
            new.add(ts, val * self.get(ts))
        new.deduplicate(0.001)
        return new

    def map(self, func):
        new = TimedNumericData()
        for ts, val in zip(self.timestamps, self.values):
            new.add(ts, func(val))
        return new

    def integral(self):
        new = TimedNumericData()
        total = 0
        last_ts = self.timestamps[0] if self.timestamps else 0
        for ts, val in zip(self.timestamps, self.values):
            delta_ts = ts - last_ts
            total += val * delta_ts
            new.add(ts, total)
            last_ts = ts
        return new
    
    def integrate(self):
        total = 0
        last_ts = self.timestamps[0] if self.timestamps else 0
        for ts, val in zip(self.timestamps, self.values):
            delta_ts = ts - last_ts
            total += val * delta_ts
            last_ts = ts
        return total

    def differentiate(self):
        new = TimedNumericData()
        last_val = self.values[0] if self.values else 0
        last_ts = self.timestamps[0] if self.timestamps else 0
        for ts, val in zip(self.timestamps, self.values):
            delta_ts = ts - last_ts
            if delta_ts > 0:
                derivative = (val - last_val) / delta_ts
                new.add(ts, derivative)
            else:
                new.add(ts, 0)
            last_val = val
            last_ts = ts
        return new

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
    
    def get(self, ts: float, default: bool = False, threshold_to_next: float = 0.01):
        # Binary search for nearest timestamp
        left, right = 0, len(self.timestamps) - 1
        while left <= right:
            mid = (left + right) // 2
            if self.timestamps[mid] < ts:
                left = mid + 1
            else:
                right = mid - 1
        
        if right >= 0 and self.timestamps[right] - ts < threshold_to_next:
            return self.values[right]
        return self.values[left] if left < len(self.timestamps) else default

    def last_or(self, default: bool):
        if self.values:
            return self.values[-1]
        return default