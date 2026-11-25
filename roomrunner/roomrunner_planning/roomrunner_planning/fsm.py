"""The mission state machine as plain Python: states, events and transitions (no ROS).

The mission_fsm node feeds it events from the skills (path found, arrived, picked, ...) and publishes
its state. Keeping the logic here makes every transition testable without running ROS.

    m = MissionStateMachine(max_retries=2)
    m.handle("request")        # IDLE -> PARSE_TASK
    m.handle("parsed")         # -> PLAN_TO_PICKUP
"""

STATES = ["IDLE", "PARSE_TASK", "PLAN_TO_PICKUP", "PLAN_TO_DROPOFF", "NAVIGATE", "ALIGN", "PICK", "PLACE",
          "RETRY", "DONE", "FAILED"]

# (state, event) -> next state. "PLAN" and "MANIPULATE" are resolved by holding / not holding an object.
TRANSITIONS = {
    ("IDLE", "request"): "PARSE_TASK",
    ("PARSE_TASK", "parsed"): "PLAN",
    ("PARSE_TASK", "parse_failed"): "FAILED",        # object not found, request not understood
    ("PLAN_TO_PICKUP", "path_found"): "NAVIGATE",
    ("PLAN_TO_DROPOFF", "path_found"): "NAVIGATE",
    ("PLAN_TO_PICKUP", "no_path"): "FAILED",
    ("PLAN_TO_DROPOFF", "no_path"): "FAILED",
    ("NAVIGATE", "arrived"): "ALIGN",
    ("NAVIGATE", "nav_failed"): "RETRY",
    ("NAVIGATE", "localization_lost"): "RETRY",
    ("ALIGN", "aligned"): "MANIPULATE",
    ("ALIGN", "goal_reached"): "NEXT",      # a go_to step with nothing to pick or place
    ("ALIGN", "nav_failed"): "RETRY",
    ("ALIGN", "localization_lost"): "RETRY",
    ("PICK", "picked"): "NEXT",            # usually more steps (carry it somewhere); a lone pick is done
    ("PICK", "pick_failed"): "RETRY",
    ("PLACE", "placed"): "NEXT",
    ("PLACE", "place_failed"): "RETRY",
    ("DONE", "reset"): "IDLE",
    ("FAILED", "reset"): "IDLE",
}


class MissionStateMachine:
    def __init__(self, max_retries=2):
        self.max_retries = max_retries
        self.reset()

    def reset(self):
        self.state = "IDLE"
        self.holding = None      # name of the held object, or None
        self.retries = 0
        self.retry_from = None   # the planning state a retry goes back to

    def handle(self, event, **info):
        """Apply one event and return the new state. `info` may carry holding=<object name> for 'picked',
        and more_steps=True for 'picked', 'placed' or 'goal_reached' when the request has further steps.
        Raises ValueError for an event that is not allowed in the current state."""
        if self.state == "RETRY":
            return self._retry(event)
        key = (self.state, event)
        if key not in TRANSITIONS:
            raise ValueError(f"event {event!r} not allowed in state {self.state}")
        target = TRANSITIONS[key]
        if event == "picked":
            self.holding = info.get("holding", "object")
        if event == "placed":
            self.holding = None
        if event == "reset":
            self.reset()
            return self.state
        if target == "NEXT":   # more steps in the request: plan the next leg; otherwise the mission is done
            target = "PLAN" if info.get("more_steps") else "DONE"
        if target == "PLAN":
            target = "PLAN_TO_DROPOFF" if self.holding else "PLAN_TO_PICKUP"
        elif target == "MANIPULATE":
            target = "PLACE" if self.holding else "PICK"
        elif target == "RETRY":
            # A failed pick means the object is not held; plan to it again.
            self.retry_from = "PLAN_TO_DROPOFF" if self.holding else "PLAN_TO_PICKUP"
        self.state = target
        return self.state

    def _retry(self, event):
        if event != "retry":
            raise ValueError(f"in RETRY only 'retry' is allowed, got {event!r}")
        if self.retries >= self.max_retries:
            self.state = "FAILED"
        else:
            self.retries += 1
            self.state = self.retry_from
        return self.state
