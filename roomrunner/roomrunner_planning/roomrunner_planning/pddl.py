"""LLM+P: the LLM writes a PDDL goal, and a small classical planner finds the steps (no ROS).

Domain (written out in DOMAIN below):
    (robot-at ?t)  the robot is next to ?t        (holding ?b)  the gripper holds block ?b
    (hand-empty)                                  (on ?b ?t)    block ?b is on ?t
    actions: go_to(?t), pick(?b) at ?b with hand empty, place(?b ?t) holding ?b at ?t

The planner is breadth-first search over these facts, so the plan it finds has the fewest steps. The
scenes are tiny (a few blocks and places), so that is fast enough; the PDDL text could also go to an
off-the-shelf planner such as Fast Downward.
"""
import re
from collections import deque

DOMAIN = """(define (domain roomrunner)
  (:predicates (robot-at ?t) (holding ?b) (hand-empty) (on ?b ?t))
  (:action go_to :parameters (?t) :effect (and (robot-at ?t) (forall (?o) (when (robot-at ?o) (not (robot-at ?o))))))
  (:action pick :parameters (?b) :precondition (and (robot-at ?b) (hand-empty))
          :effect (and (holding ?b) (not (hand-empty))))
  (:action place :parameters (?b ?t) :precondition (and (holding ?b) (robot-at ?t))
          :effect (and (on ?b ?t) (hand-empty) (not (holding ?b)))))"""

GOAL_HELP = "Goal facts you can use: (on <block> <tray or place>), (robot-at <object or place>), (holding <block>)."

FACT = re.compile(r"\((on|robot-at|holding)\s+([\w-]+)(?:\s+([\w-]+))?\)")


def parse_goal(text):
    """'(:goal (and (on red_block tray) (robot-at home)))' -> {('on', 'red_block', 'tray'), ('robot-at', 'home')}."""
    goal = set()
    for name, a, b in FACT.findall(text):
        goal.add((name, a, b) if name == "on" else (name, a))
    if not goal:
        raise ValueError("no goal facts in the answer")
    return goal


def problem_text(objects, places, goal_text):
    """The PDDL problem for the current scene (for the report, or an external planner)."""
    names = " ".join(sorted(objects) + sorted(places))
    return (f"(define (problem fetch) (:domain roomrunner)\n  (:objects {names})\n"
            f"  (:init (hand-empty))\n  {goal_text})")


def solve(goal, objects, places, max_depth=12):
    """Fewest-step plan reaching every goal fact. Returns a list of (skill, target), as the direct LLM mode.
    State: (robot location, held block, frozenset of (block, target) placements)."""
    blocks = sorted(n for n, k in objects.items() if k == "block")
    targets = sorted(objects) + sorted(places)
    for fact in goal:
        for name in fact[1:]:
            if name not in objects and name not in places:
                raise ValueError(f"'{name}' is not in the room")

    def satisfied(state):
        at, held, on = state
        return all((f[0] == "on" and (f[1], f[2]) in on) or (f[0] == "robot-at" and at == f[1])
                   or (f[0] == "holding" and held == f[1]) for f in goal)

    start = (None, None, frozenset())
    queue, seen = deque([(start, [])]), {start}
    while queue:
        state, steps = queue.popleft()
        if satisfied(state):
            return steps
        if len(steps) >= max_depth:
            continue
        at, held, on = state
        moves = [((t, held, on), ("go_to", t)) for t in targets if t != at]
        if held is None and at in blocks and not any(b == at for b, _ in on):
            moves.append(((at, at, on), ("pick", at)))
        if held is not None and at is not None and at != held and objects.get(at) != "block":
            moves.append(((at, None, on | {(held, at)}), ("place", at)))
        for nxt, step in moves:
            if nxt not in seen:
                seen.add(nxt)
                queue.append((nxt, steps + [step]))
    raise ValueError("no plan reaches the goal")
