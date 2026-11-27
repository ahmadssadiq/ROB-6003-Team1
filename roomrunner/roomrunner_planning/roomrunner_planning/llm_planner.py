"""LLM task planning: request text -> validated step list, with re-planning on errors (no ROS).

Two versions, compared in experiment E2:
    direct  the LLM writes the steps as JSON          {"steps": [{"skill": "go_to", "target": "red_block"}, ...]}
    llmp    the LLM writes a PDDL goal, and the classical planner in pddl.py finds the steps (LLM+P)

The LLM is any function `llm(messages) -> text`, so tests use a fake one and nothing goes to the network.
make_llm() builds the real one from the environment (.env):
    ROOMRUNNER_LLM_PROVIDER   mock (default), openai or anthropic
    ROOMRUNNER_LLM_MODEL      model name for that provider
    OPENAI_API_KEY / ANTHROPIC_API_KEY
The key is only ever read from the environment; never put it in code or config files.
"""
import json
import os
import re

from roomrunner_planning import pddl
from roomrunner_planning.mock_parser import parse as mock_parse
from roomrunner_planning.names import resolve_goal, resolve_steps
from roomrunner_planning.validator import validate

SKILLS_TEXT = """The robot has three skills:
  go_to(target)  drive next to an object or to a named place
  pick(object)   pick up a block; the robot must be at it and holding nothing
  place(target)  put the held block on a tray or at a place; the robot must be at it"""


def scene_text(objects, places):
    lines = [f"  {name} ({kind})" for name, kind in sorted(objects.items())]
    lines += [f"  {name} (place)" for name in sorted(places)]
    return "Objects and places in the room:\n" + "\n".join(lines)


def direct_messages(request, objects, places):
    return [{"role": "system", "content": f"You plan tasks for a mobile robot.\n{SKILLS_TEXT}\n"
             f"{scene_text(objects, places)}\nAnswer only with JSON: "
             '{"steps": [{"skill": "go_to", "target": "..."}, ...]}. Use only the names listed.'},
            {"role": "user", "content": request}]


def llmp_messages(request, objects, places):
    return [{"role": "system", "content": f"You translate requests into PDDL goals for a mobile robot.\n"
             f"{scene_text(objects, places)}\n{pddl.GOAL_HELP}\nAnswer only with the goal, for example: "
             "(:goal (and (on red_block tray) (robot-at home)))"},
            {"role": "user", "content": request}]


def parse_steps_json(text):
    """Pull the step list out of an LLM answer (tolerates text or a ```json fence around the JSON)."""
    match = re.search(r"\{.*\}", text, re.S)
    if not match:
        raise ValueError("no JSON object in the answer")
    data = json.loads(match.group(0))
    return [(str(s["skill"]), str(s["target"])) for s in data["steps"]]


def plan(request, objects, places, llm, mode="direct", reachable=None, max_replans=3):
    """Ask the LLM for a plan, validate it, and send the errors back until it is valid.
    Object names are matched to the scene first (names.py: "red_block" -> "red_cuboid").
    Returns (steps, replans). Raises ValueError if no valid plan is found."""
    messages = direct_messages(request, objects, places) if mode == "direct" else llmp_messages(request, objects, places)
    errors = []
    for attempt in range(max_replans + 1):
        answer = llm(messages)
        try:
            known = set(objects) | set(places)
            if mode == "direct":
                steps = resolve_steps(parse_steps_json(answer), known)
            else:
                steps = pddl.solve(resolve_goal(pddl.parse_goal(answer), known), objects, places)
            errors = validate(steps, objects, places, reachable)
        except (ValueError, KeyError, TypeError) as error:
            errors = [f"could not use the answer: {error}"]
        if not errors:
            return steps, attempt
        messages = messages + [{"role": "assistant", "content": answer},
                               {"role": "user", "content": "That plan is not valid:\n- " + "\n- ".join(errors)
                                + "\nPlease answer again with a corrected plan."}]
    raise ValueError("no valid plan after re-planning: " + "; ".join(errors))


def mock_llm(objects, places):
    """An offline 'LLM' that answers direct-mode prompts with the rule-based parser (for tests and demos)."""
    def llm(messages):
        request = next(m["content"] for m in reversed(messages) if m["role"] == "user"
                       and not m["content"].startswith("That plan"))
        steps = mock_parse(request)
        return json.dumps({"steps": [{"skill": s, "target": t} for s, t in steps]})
    return llm


def make_llm(objects=None, places=None):
    """The LLM chosen in the environment. Imports the provider's SDK only when it is used."""
    provider = os.environ.get("ROOMRUNNER_LLM_PROVIDER", "mock")
    model = os.environ.get("ROOMRUNNER_LLM_MODEL", "")
    if provider == "mock":
        return mock_llm(objects or {}, places or set())
    if provider == "openai":
        from openai import OpenAI
        client = OpenAI()   # reads OPENAI_API_KEY

        def llm(messages):
            reply = client.chat.completions.create(model=model, messages=messages, temperature=0)
            return reply.choices[0].message.content
        return llm
    if provider == "anthropic":
        import anthropic
        client = anthropic.Anthropic()   # reads ANTHROPIC_API_KEY

        def llm(messages):
            system = "\n".join(m["content"] for m in messages if m["role"] == "system")
            chat = [m for m in messages if m["role"] != "system"]
            reply = client.messages.create(model=model, system=system, messages=chat, max_tokens=1000, temperature=0)
            return reply.content[0].text
        return llm
    raise ValueError(f"unknown ROOMRUNNER_LLM_PROVIDER {provider!r}")
