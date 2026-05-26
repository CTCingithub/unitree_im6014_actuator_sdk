"""
Type stubs for unitree_im6014 package.

All values are in output-end (load-side) units.
"""

class State:
    """Motor feedback state (output-end units)."""

    id: int
    """Motor ID (0-15)."""

    pos: float
    """Output position [rad]."""

    speed: float
    """Output speed [rad/s]."""

    torque: float
    """Output torque [Nm]."""

    valid: bool
    """Whether this feedback packet passed CRC verification."""


class Motor:
    """IM6014 motor controller (output-end units)."""

    def __init__(self) -> None:
        """Create an uninitialized motor controller."""
        ...

    def init(self, port: str, baudrate: int = 4000000) -> bool:
        """
        Open the serial port.

        Args:
            port: Serial port path, e.g. '/dev/ttyUSB0'.
            baudrate: Baud rate (4000000 or 6000000).

        Returns:
            True on success.
        """
        ...

    def close(self) -> None:
        """Close the serial port."""
        ...

    def send_cmd(
        self,
        id: int,
        tor: float,
        spd: float,
        pos: float,
        kp: float,
        kd: float,
        status: int = 1,
        timeout: int = 0,
    ) -> bool:
        """
        Send a control command to a motor.

        Args:
            id: Motor ID (0-15).
            tor: Feed-forward torque [Nm] (output-end).
            spd: Desired speed [rad/s] (output-end).
            pos: Desired position [rad] (output-end).
            kp: Position stiffness [Nm/rad] (output-end).
            kd: Speed damping [Nm/(rad/s)] (output-end).
            status: Motor status (1=enable, 0=disable).
            timeout: Motor-side watchdog (0=disabled, 1=enabled).

        Returns:
            True if the full packet was written to the serial port.
        """
        ...

    def recv_state(self, id: int, state: State, timeout_ms: int = 50) -> bool:
        """
        Read motor feedback.

        Args:
            id: Expected motor ID (0-15). Use 15 as wildcard.
            state: State object to fill with decoded feedback.
            timeout_ms: Read timeout in milliseconds.

        Returns:
            True if a valid feedback packet was received and decoded.
        """
        ...
