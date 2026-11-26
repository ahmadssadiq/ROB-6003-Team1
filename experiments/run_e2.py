"""Run experiment E2: every request of e2_requests.yaml with each planner, and log the trials.

Offline (planning only, no robot; the LLM key comes from .env / the environment):
    python3 experiments/run_e2.py --planners mock direct llmp
On the running system (stubs, simulation or the real robot; the stack must already be launched):
    python3 experiments/run_e2.py --ros --planners direct llmp [--confirm]

Writes experiments/data/e2_trials.csv (one row per request and planner), then:
    python3 experiments/analyze.py e2

--ros sets the task_parser's mode, sends each request on /task_request and waits for /mission_status to reach
DONE or FAILED. success = the mission ended DONE and its plan reaches the request's goal; with --confirm you
are also asked whether it really happened (use it on the real robot).
"""
import argparse
import csv
import os
import sys
import time
from pathlib import Path

from e2 import COLUMNS, achieves, load_requests, offline_trial, plan_text

DATA = Path(__file__).resolve().parent / "data"


def load_env():
    """Read KEY=value lines from the repository's .env into the environment (without overriding)."""
    env = Path(__file__).resolve().parents[1] / ".env"
    if env.exists():
        for line in env.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                key, value = line.split("=", 1)
                if value.strip():
                    os.environ.setdefault(key.strip(), value.strip())


def run_offline(requests, scene, planners):
    from roomrunner_planning import llm_planner
    llm = None
    if any(p != "mock" for p in planners):
        llm = llm_planner.make_llm(scene["objects"], scene["places"])
        if os.environ.get("ROOMRUNNER_LLM_PROVIDER", "mock") == "mock":
            print("Note: ROOMRUNNER_LLM_PROVIDER is mock, so 'direct' uses the rule-based parser and 'llmp' "
                  "cannot work. Set a provider and key in .env for real LLM results.")
    rows = []
    for planner in planners:
        for r in requests:
            row = offline_trial(r, scene, planner, llm)
            rows.append(row)
            print(f"{planner:6s} #{r['id']:>2} {'OK  ' if row['success'] else 'FAIL'} {r['text']!r} -> "
                  f"{row['plan'] or row['error']}")
    return rows


def run_ros(requests, planners, confirm, timeout):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.parameter_client import AsyncParameterClient
    from std_msgs.msg import String

    from roomrunner_interfaces.msg import MissionStatus, TaskSpec

    rclpy.init()
    node = rclpy.create_node("run_e2")
    last = {}
    node.create_subscription(TaskSpec, "task_spec", lambda m: last.update(spec=m), 10)
    node.create_subscription(MissionStatus, "mission_status", lambda m: last.update(status=m), 10)
    pub = node.create_publisher(String, "task_request", 10)
    params = AsyncParameterClient(node, "task_parser")

    def spin_until(condition, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not condition():
            rclpy.spin_once(node, timeout_sec=0.1)
        return condition()

    if not params.wait_for_services(timeout_sec=5.0):
        sys.exit("task_parser is not running: launch the system first (stubs.launch.py or robot.launch.py).")
    rows = []
    for planner in planners:
        future = params.set_parameters([Parameter("mode", value=planner)])
        spin_until(future.done, 5.0)
        for r in requests:
            spin_until(lambda: "status" in last and last["status"].state == "IDLE", 60.0)
            last.clear()
            start = time.monotonic()
            pub.publish(String(data=r["text"]))
            got_spec = spin_until(lambda: "spec" in last, 120.0)
            plan = [(s.skill, s.target) for s in last["spec"].steps] if got_spec else []
            ended = spin_until(lambda: "status" in last and last["status"].state in ("DONE", "FAILED"), timeout)
            done = ended and last["status"].state == "DONE"
            success = done and achieves(plan, r["goal_facts"])
            if success and confirm:
                success = input(f"#{r['id']} {r['text']!r}: did the robot really do it? [y/n] ").strip().lower() == "y"
            row = {k: r.get(k, "") for k in ("id", "text", "steps", "category", "style")}
            row.update(planner=planner, valid=int(bool(plan)), replans=last["spec"].replans if got_spec else 0,
                       success=int(success), plan=plan_text(plan),
                       error="" if success else (last["status"].detail if ended else "timed out"),
                       seconds=round(time.monotonic() - start, 1))
            rows.append(row)
            print(f"{planner:6s} #{r['id']:>2} {'OK  ' if success else 'FAIL'} {r['text']!r} -> {row['plan'] or row['error']}")
    node.destroy_node()
    rclpy.shutdown()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--planners", nargs="+", default=["direct", "llmp"], choices=["mock", "direct", "llmp"])
    parser.add_argument("--ros", action="store_true", help="run on the launched system instead of offline")
    parser.add_argument("--confirm", action="store_true", help="--ros only: ask whether each success really happened")
    parser.add_argument("--ids", nargs="+", type=int, help="only these request ids")
    parser.add_argument("--timeout", type=float, default=600.0, help="--ros: seconds per mission")
    parser.add_argument("--output", default=str(DATA / "e2_trials.csv"))
    args = parser.parse_args()

    load_env()
    scene, requests = load_requests()
    if args.ids:
        requests = [r for r in requests if r["id"] in args.ids]
    rows = run_ros(requests, args.planners, args.confirm, args.timeout) if args.ros else \
        run_offline(requests, scene, args.planners)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    for planner in args.planners:
        mine = [r for r in rows if r["planner"] == planner]
        print(f"{planner}: success {sum(r['success'] for r in mine)}/{len(mine)}, "
              f"valid {sum(r['valid'] for r in mine)}/{len(mine)}, re-plans {sum(r['replans'] for r in mine)}")
    print(f"Wrote {out}. Plot: python3 experiments/analyze.py e2")


if __name__ == "__main__":
    main()
