"""
Type stubs for unitree_im6014 package.

All values are in output-end (load-side) units:
  torque [Nm], speed [rad/s], position [rad], Kp [Nm/rad], Kd [Nm/(rad/s)]
"""

from typing import Optional

WILDCARD_ID: int = 15
"""Broadcast / match-any motor ID."""

class State:
    """Decoded motor feedback state (output-end units)."""

    id: int
    """Motor ID (0-14 physical, 15 wildcard)."""

    status: int
    """Motor status from protocol (ENABLE=1, DISABLE=0)."""

    timeout: int
    """Motor-side watchdog flag from protocol."""

    temp1: float
    """Temperature sensor 1 [deg C]."""

    temp2: float
    """Temperature sensor 2 [deg C]."""

    voltage: float
    """Bus voltage [V]."""

    torque: float
    """Output torque [Nm]."""

    speed: float
    """Output speed [rad/s]."""

    pos: float
    """Output position [rad]."""

    error: int
    """Motor error flags (bitmask)."""

    warning: int
    """Motor warning flags."""

    valid: bool
    """True if this state passed CRC verification."""

    timestamp_us: int
    """steady_clock timestamp [us] when this frame was received."""

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
        Send a control command (guaranteed full-packet write via write_all).

        Args:
            id: Motor ID (0-14 physical, 15 wildcard).
            tor: Feed-forward torque [Nm] (output-end).
            spd: Desired speed [rad/s] (output-end).
            pos: Desired position [rad] (output-end).
            kp: Position stiffness [Nm/rad] (output-end).
            kd: Speed damping [Nm/(rad/s)] (output-end).
            status: ENABLE (1) or DISABLE (0).
            timeout: Motor-side watchdog (0=off, 1=on).

        Returns:
            True if all 20 bytes were written.
        """
        ...

    def poll_states(self, timeout_ms: int = 2) -> None:
        """
        Drain serial data, parse all complete frames, update internal cache.

        Call once per control cycle.  Use with get_state() for multi-motor
        high-rate (500 Hz – 1 kHz) loops.
        """
        ...

    def get_state(self, id: int, state: State) -> bool:
        """
        Retrieve the most recent cached state for a motor.

        Args:
            id: Motor ID.
            state: State object to fill.

        Returns:
            True if cached state is available and valid.
        """
        ...

    def recv_state(self, id: int, state: State, timeout_ms: int = 50) -> bool:
        """
        Legacy blocking request-response read for a single motor.

        Prefer poll_states() + get_state() for multi-motor or high-rate use.

        Args:
            id: Expected motor ID (use 15 for any).
            state: State object to fill.
            timeout_ms: Read timeout in milliseconds.

        Returns:
            True if a matching valid frame was received.
        """
        ...
