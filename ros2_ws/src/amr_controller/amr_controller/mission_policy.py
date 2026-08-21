"""Pure helpers for the IW Hub lift-and-place mission."""


def linear_ramp(start, end, elapsed, duration):
    """Return a clamped linear command and whether the ramp is complete."""

    if duration <= 0.0:
        raise ValueError("duration must be positive")

    progress = min(max(float(elapsed) / float(duration), 0.0), 1.0)
    value = float(start) + (float(end) - float(start)) * progress
    return value, progress >= 1.0

