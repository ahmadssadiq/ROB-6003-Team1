"""Runs a mission: drives the state machine (fsm.py) with the skills (plain Python, no ROS).

The skills are passed in, so the same code runs with the ROS action clients in mission_fsm and with fake
skills in the tests:

    skills.refresh_map()                      -> None          ask the overhead camera for a new frame
    skills.check(steps)                       -> list of str   validation errors (objects may have moved)
    skills.plan(target)                       -> (bool, str)   path to the target found?
    skills.navigate(target, on_arrived)       -> str           "aligned", "nav_failed" or "localization_lost";
                                                               calls on_arrived() once the base is at the goal
                                                               and only turning / settling is left
    skills.manipulate(skill, target)          -> bool          pick or place succeeded?

A request is split into legs: a go_to, optionally followed by a pick or place at the same target. A failed
leg is retried from its planning state (fsm.RETRY) until the retries run out.
"""
from roomrunner_planning.fsm import MissionStateMachine


def split_legs(steps):
    """[(skill, target)] -> [(target, manipulation or None)]: each go_to with the pick/place that follows it.
    Raises ValueError if a pick or place does not follow a go_to to the same target."""
    legs, i = [], 0
    while i < len(steps):
        skill, target = steps[i]
        if skill != "go_to":
            raise ValueError(f"step {i + 1}: {skill} {target} must follow go_to {target}")
        if i + 1 < len(steps) and steps[i + 1][0] in ("pick", "place"):
            if steps[i + 1][1] != target:
                raise ValueError(f"step {i + 2}: {steps[i + 1][0]} {steps[i + 1][1]} is not at {target}")
            legs.append((target, steps[i + 1][0]))
            i += 2
        else:
            legs.append((target, None))
            i += 1
    return legs


def run_mission(steps, skills, machine=None, on_state=None, request=""):
    """Run the steps. Returns (final state "DONE" or "FAILED", detail text). on_state(state, detail, leg index)
    is called on every state change (the node publishes it as MissionStatus)."""
    m = machine or MissionStateMachine()
    m.reset()
    report = on_state or (lambda state, detail, leg: None)

    def event(name, detail="", leg=-1, **info):
        m.handle(name, **info)
        report(m.state, detail, leg)

    event("request", request)
    errors = skills.check(steps) if steps else ["the request could not be turned into steps"]
    if not errors:
        try:
            legs = split_legs(steps)
        except ValueError as error:
            errors = [str(error)]
    if errors:
        event("parse_failed", "; ".join(errors))
        return m.state, "; ".join(errors)
    event("parsed", f"{len(legs)} legs")

    index = 0
    while m.state not in ("DONE", "FAILED"):
        target, manipulation = legs[index]
        more_steps = index < len(legs) - 1

        if m.state == "RETRY":
            event("retry", f"attempt {m.retries + 1} for {target}", index)
            continue

        # PLAN_TO_PICKUP / PLAN_TO_DROPOFF
        skills.refresh_map()
        ok, message = skills.plan(target)
        if not ok:
            event("no_path", f"no valid path to {target}: {message}", index)
            break
        event("path_found", f"to {target}: {message}", index)

        # NAVIGATE, then ALIGN once the base is at the goal
        outcome = skills.navigate(target, lambda: event("arrived", f"at {target}", index))
        if outcome != "aligned":
            event(outcome, f"{outcome} on the way to {target}", index)
            continue
        if m.state == "NAVIGATE":          # the skill never reported arriving: count it now
            event("arrived", f"at {target}", index)

        if manipulation is None:
            event("goal_reached", f"at {target}", index, more_steps=more_steps)
            index += 1
            continue

        event("aligned", f"aligned and stopped at {target}", index)   # -> PICK or PLACE
        if not skills.manipulate(manipulation, target):
            event("pick_failed" if manipulation == "pick" else "place_failed", f"{manipulation} {target} failed", index)
            continue
        if manipulation == "pick":
            event("picked", f"holding {target}", index, holding=target, more_steps=more_steps)
        else:
            event("placed", f"placed on {target}", index, more_steps=more_steps)
        index += 1
        skills.refresh_map()               # the object moved

    detail = "mission completed" if m.state == "DONE" else "mission failed"
    return m.state, detail
