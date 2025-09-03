# singleton.py
class DroneState:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DroneState, cls).__new__(cls)
            cls._instance.current_mode = None
            cls._instance.is_armed = False
        return cls._instance


# Use this to get the instance
drone_state = DroneState()
