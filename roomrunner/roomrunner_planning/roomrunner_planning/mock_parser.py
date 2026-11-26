"""Rule-based task parser: the offline stand-in for the LLM (no network, no API key).

It understands a few sentence shapes, split into parts by "then", "and then", "and" or commas:
    bring / take / move / put / carry / fetch / deliver  the <object> to / into / in / on / onto  the <place>
    pick up / grab the <object>
    place / drop / put it  to / in / on / into  the <place>
    go to / drive to  the <place>
    come back / return / go home

Every part becomes skills: go_to, pick and place. Names are written with underscores, as the overhead
camera reports them ("red block" -> "red_block").
"""
import re

FETCH = re.compile(r"^(?:bring|take|move|put|carry|fetch|deliver) (?:the )?(.+?) (?:to|into|in|on|onto) (?:the )?(.+)$")
PICK = re.compile(r"^(?:pick up|pick|grab|get) (?:the )?(.+)$")
PLACE = re.compile(r"^(?:place|drop|put|leave) (?:it )?(?:down )?(?:to|into|in|on|onto) (?:the )?(.+)$")
GO = re.compile(r"^(?:go|drive|move) to (?:the )?(.+)$")
HOME = re.compile(r"^(?:come back|return(?: home)?|go home|go back(?: home)?)$")
FETCH_HOME = re.compile(r"^(?:bring|take|move|carry|fetch|deliver) (?:the )?(.+?) (home)$")
VERBS = r"(?:go|come|return|head|drive|put|place|drop|leave|take|bring|move|carry|fetch|deliver|pick|grab|get)"


def name_of(words):
    """'Red Block' -> 'red_block'."""
    return "_".join(words.lower().split())


def objects_of(words):
    """'the red block and the blue block' / 'red and blue blocks' -> ['red_block', 'blue_block']."""
    words = words.strip()
    if m := re.match(r"^(\w+) and (\w+) (\w+?)s$", words):        # "red and blue blocks"
        return [name_of(f"{m.group(1)} {m.group(3)}"), name_of(f"{m.group(2)} {m.group(3)}")]
    return [name_of(re.sub(r"^the ", "", w.strip())) for w in re.split(r"\band\b", words) if w.strip()]


def parse(request):
    """Request text -> list of (skill, target), or raise ValueError if a part is not understood."""
    text = request.lower().strip().rstrip(".!")
    # Split into commands at commas and "then"; at "and" only when a new command follows ("and go home"),
    # so "the red block and the blue block" stays one object list.
    parts = [p.strip() for p in re.split(r",|\band then\b|\bthen\b|\band (?=" + VERBS + r"\b)", text) if p.strip()]
    steps = []
    for part in parts:
        if m := PLACE.match(part):     # before FETCH, so "put it on the tray" is not "fetch 'it'"
            place = name_of(m.group(1))
            steps += [("go_to", place), ("place", place)]
        elif (m := FETCH.match(part)) or (m := FETCH_HOME.match(part)):
            place = name_of(m.group(2)) if m.re is FETCH else "home"
            for obj in objects_of(m.group(1)):     # "the red block and the blue block" -> two fetches
                steps += [("go_to", obj), ("pick", obj), ("go_to", place), ("place", place)]
        elif m := PICK.match(part):
            obj = name_of(m.group(1))
            steps += [("go_to", obj), ("pick", obj)]
        elif HOME.match(part):
            steps.append(("go_to", "home"))
        elif m := GO.match(part):
            steps.append(("go_to", name_of(m.group(1))))
        else:
            raise ValueError(f"cannot understand {part!r}")
    if not steps:
        raise ValueError("empty request")
    return steps
