"""Plan validator: checks a step list before the robot runs it (no ROS).

    errors = validate(steps, objects, places, reachable)
    steps     list of (skill, target), skill in go_to / pick / place
    objects   {name: kind}, kind "block" or "tray", as the overhead camera sees them
    places    set of named places without an object, such as {"home"}
    reachable optional function target -> bool (A* finds a path to it); None skips that check

It returns a list of readable errors, empty if the plan is valid. The errors are sent back to the LLM
when it has to plan again.
"""

SKILLS = {"go_to", "pick", "place"}


def validate(steps, objects, places, reachable=None):
    errors = []
    if not steps:
        return ["the plan has no steps"]
    at, holding = None, None
    for i, (skill, target) in enumerate(steps, 1):
        where = f"step {i} ({skill} {target})"
        if skill not in SKILLS:
            errors.append(f"{where}: unknown skill; use go_to, pick or place")
            continue
        if target not in objects and target not in places:
            errors.append(f"{where}: '{target}' is not in the room; known: {sorted(objects) + sorted(places)}")
            continue
        if skill == "go_to":
            if reachable is not None and not reachable(target):
                errors.append(f"{where}: no path to {target}")
            at = target
        elif skill == "pick":
            if objects.get(target) != "block":
                errors.append(f"{where}: only blocks can be picked")
            elif holding:
                errors.append(f"{where}: already holding {holding}; place it first")
            elif at != target:
                errors.append(f"{where}: go_to {target} before picking it")
            else:
                holding = target
        elif skill == "place":
            if not holding:
                errors.append(f"{where}: not holding anything")
            elif at != target:
                errors.append(f"{where}: go_to {target} before placing there")
            elif objects.get(target) == "block":
                errors.append(f"{where}: cannot place on a block")
            else:
                holding = None
    return errors
