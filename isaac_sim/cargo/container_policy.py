"""Isaac-independent state policy for releasing an IW Hub payload."""


def should_release_payload(
    distance_to_goal,
    lifted_once,
    lift_delta,
    goal_tolerance=0.30,
    lowered_tolerance=0.04,
):
    """Return True only after a lifted payload is lowered at its goal."""

    if goal_tolerance <= 0.0:
        raise ValueError("goal_tolerance must be positive")
    if lowered_tolerance < 0.0:
        raise ValueError("lowered_tolerance cannot be negative")

    return (
        lifted_once
        and distance_to_goal <= goal_tolerance
        and lift_delta <= lowered_tolerance
    )

