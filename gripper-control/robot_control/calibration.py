"""Pure servo-calibration helpers shared by the GUI and tests.

The Arduino implements the same integer interpolation. Keeping the model
here makes validation and protocol tests possible without PySide6 or hardware.
"""

from __future__ import annotations

from dataclasses import dataclass

from .config import (
    DEFAULT_CALIBRATION_US,
    MAX_ANGLE,
    MIN_ANGLE,
    PULSE_MAX_ALLOWED_US,
    PULSE_MIN_ALLOWED_US,
)


@dataclass(frozen=True)
class ServoCalibration:
    """Pulse widths for the three GUI reference angles."""

    min_us: int = DEFAULT_CALIBRATION_US[0]
    center_us: int = DEFAULT_CALIBRATION_US[1]
    max_us: int = DEFAULT_CALIBRATION_US[2]

    def validate(self) -> str | None:
        values = (self.min_us, self.center_us, self.max_us)
        if any(not isinstance(value, int) for value in values):
            return "Pulsvärden måste vara heltal."
        if any(value < PULSE_MIN_ALLOWED_US or value > PULSE_MAX_ALLOWED_US for value in values):
            return (
                f"Pulsvärden måste ligga mellan {PULSE_MIN_ALLOWED_US} och "
                f"{PULSE_MAX_ALLOWED_US} µs."
            )
        if not self.min_us < self.center_us < self.max_us:
            return "Ordningen måste vara 0° min < 90° center < 180° max."
        return None

    def pulse_for_angle(self, angle: int) -> int:
        """Return the calibrated pulse using piecewise linear interpolation."""
        if not MIN_ANGLE <= angle <= MAX_ANGLE:
            raise ValueError("angle must be between 0 and 180 degrees")
        if angle <= 90:
            return self.min_us + ((self.center_us - self.min_us) * angle + 45) // 90
        return self.center_us + ((self.max_us - self.center_us) * (angle - 90) + 45) // 90


def calibration_from_values(min_us: int, center_us: int, max_us: int) -> ServoCalibration:
    """Build and validate a calibration model."""
    calibration = ServoCalibration(min_us, center_us, max_us)
    error = calibration.validate()
    if error:
        raise ValueError(error)
    return calibration
