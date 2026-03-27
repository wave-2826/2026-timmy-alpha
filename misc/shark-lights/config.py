class Configuration:
    fmsAddress: str = "ws://localhost:8080/setup/field_testing/websocket"
    led_count: int = 60
    data_pin: int = 18
    HubAlliance: str = "Blue" # "Red" or "Blue"
    funLogging: bool = True