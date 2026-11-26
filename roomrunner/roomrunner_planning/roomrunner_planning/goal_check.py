"""Does a plan do what the request asked? Simulates a step list and compares the end state with a goal.

A goal is a set of facts, written as in the PDDL goals of pddl.py:
    ("on", block, target)   block ends on the tray or at a place
    ("robot-at", target)    the robot ends next to it
    ("holding", block)      the robot ends holding it
The check is exact for blocks: every block the plan moves must be one the goal asks for, and the robot must
end holding exactly what the goal says (nothing, if the goal has no "holding"). So "bring the red block to
the tray" planned as "bring the blue block to the tray" fails, even though that plan is valid.
"""


def simulate(steps):
    """Run (skill, target) steps on an empty start. Returns (robot location, held block, set of (block, target)).
    Raises ValueError on a step that cannot run (use validator.validate first for readable errors)."""
    at, held, on = None, None, set()
    for skill, target in steps:
        if skill == "go_to":
            at = target
        elif skill == "pick":
            if held or at != target:
                raise ValueError(f"cannot pick {target}")
            held = target
            on = {(b, t) for b, t in on if b != target}
        elif skill == "place":
            if not held or at != target:
                raise ValueError(f"cannot place at {target}")
            on.add((held, target))
            held = None
        else:
            raise ValueError(f"unknown skill {skill}")
    return at, held, on


def achieves(steps, goal):
    """True if the steps end in exactly the goal's state (see the module description)."""
    try:
        at, held, on = simulate(steps)
    except ValueError:
        return False
    goal_on = {(f[1], f[2]) for f in goal if f[0] == "on"}
    goal_held = next((f[1] for f in goal if f[0] == "holding"), None)
    goal_at = [f[1] for f in goal if f[0] == "robot-at"]
    return on == goal_on and held == goal_held and all(at == g for g in goal_at)
