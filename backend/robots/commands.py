from dataclasses import dataclass


@dataclass
class ControlCommand:

    linear: float = 0.0
    angular: float = 0.0
