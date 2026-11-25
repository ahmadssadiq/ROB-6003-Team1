# What the mission state machine is doing.
std_msgs/Header header
string state         # IDLE, PARSE_TASK, PLAN_TO_PICKUP, NAVIGATE, ALIGN, PICK, PLAN_TO_DROPOFF, PLACE, DONE, FAILED, RETRY
int32 step_index     # index of the current step in the TaskSpec (-1 if none)
string detail        # human-readable reason, for example "no valid path to tray"
bool success         # true only in DONE
