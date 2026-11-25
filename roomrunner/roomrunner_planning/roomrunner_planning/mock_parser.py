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


def name_of(words):
    """'Red Block' -> 'red_block'."""
    return "_".join(words.lower().split())


def parse(request):
    """Request text -> list of (skill, target), or raise ValueError if a part is not understood."""
    text = request.lower().strip().rstrip(".!")
    parts = [p.strip() for p in re.split(r",|\band then\b|\bthen\b|\band\b", text) if p.strip()]
    steps = []
    for part in parts:
        if m := PLACE.match(part):     # before FETCH, so "put it on the tray" is not "fetch 'it'"
            place = name_of(m.group(1))
            steps += [("go_to", place), ("place", place)]
        elif m := FETCH.match(part):
            obj, place = name_of(m.group(1)), name_of(m.group(2))
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
