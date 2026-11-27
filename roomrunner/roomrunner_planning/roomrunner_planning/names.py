"""Match the object names in a plan to the names the overhead camera reports (no ROS).

People say "the red block", the mock parser turns that into "red_block", and an LLM may write "red_cube".
Since decision 15 the overhead camera reports the coloured cuboids as "red_cuboid", "blue_cuboid", ... so a
name is matched by its colour when its last word is a word for a block:

    resolve("red_block", {"red_cuboid", "tray"})       -> "red_cuboid"
    resolve("red_block", {"red_block", "red_cuboid"})  -> "red_block"   (an exact name always wins)
    resolve("purple_block", {"red_cuboid"})            -> "purple_block" (unchanged; the validator reports it)

A name is only changed when exactly one known name fits, so nothing is ever guessed between two objects.
"""

BLOCK_WORDS = {"block", "blocks", "cube", "cubes", "cuboid", "cuboids", "box", "brick", "object"}


def _colour(name):
    """'red_block' -> 'red'; None if the name does not end in a block word or has nothing before it."""
    words = name.lower().split("_")
    if len(words) < 2 or words[-1] not in BLOCK_WORDS:
        return None
    return "_".join(words[:-1])


def resolve(name, known):
    """The known name `name` refers to, or `name` itself if it is known already or nothing fits exactly once."""
    if name in known:
        return name
    colour = _colour(name)
    if colour is None:
        return name
    matches = [k for k in known if _colour(k) == colour]
    return matches[0] if len(matches) == 1 else name


def resolve_steps(steps, known):
    """[(skill, target)] with every target resolved against the known names."""
    return [(skill, resolve(target, known)) for skill, target in steps]


def resolve_goal(goal, known):
    """PDDL goal facts (pddl.parse_goal) with every object name resolved, e.g. ('on', 'red_block', 'tray')."""
    return {(fact[0],) + tuple(resolve(n, known) for n in fact[1:]) for fact in goal}
