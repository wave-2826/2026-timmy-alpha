class SubsystemMap:
    def __init__(self, subsystems: dict[str, list[int]]):
        self.subsystems = subsystems

    def get_subsystem(self, port: int) -> str:
        for subsystem, ports in self.subsystems.items():
            if port in ports:
                return subsystem
        return "unknown"