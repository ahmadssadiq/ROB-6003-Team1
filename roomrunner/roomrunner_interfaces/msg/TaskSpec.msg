# A request turned into steps by the task parser.
std_msgs/Header header
string request       # the text the user typed
TaskStep[] steps
string planner       # "mock", "direct" (LLM writes the steps) or "llmp" (LLM writes PDDL, a planner solves it)
int32 replans        # how many times the LLM had to plan again before the plan was valid
