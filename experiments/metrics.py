"""Experiment metrics (plain Python): the formulas used in the report.

E1  position RMSE = sqrt( 1/T * sum_t [ (x_hat_t - x_t)^2 + (y_hat_t - y_t)^2 ] )
E2  success rate  SR = N_success / N_trials, valid-plan fraction, mean number of re-plans
"""
import math


def rmse(estimates, truths):
    """Position RMSE (m) over paired (x, y) estimates and ground-truth points."""
    if len(estimates) != len(truths) or not estimates:
        raise ValueError("need the same, non-zero number of estimates and truths")
    return math.sqrt(sum((ex - tx) ** 2 + (ey - ty) ** 2 for (ex, ey), (tx, ty) in zip(estimates, truths))
                     / len(estimates))


def success_rate(outcomes):
    """Fraction of True values: SR = N_success / N_trials."""
    if not outcomes:
        raise ValueError("no trials")
    return sum(bool(o) for o in outcomes) / len(outcomes)


def task_summary(trials):
    """trials: list of dicts with planner, valid (bool), replans (int), success (bool).
    Returns {planner: {"SR", "valid_fraction", "mean_replans", "trials"}}."""
    summary = {}
    for planner in sorted({t["planner"] for t in trials}):
        rows = [t for t in trials if t["planner"] == planner]
        summary[planner] = {"SR": success_rate([r["success"] for r in rows]),
                            "valid_fraction": success_rate([r["valid"] for r in rows]),
                            "mean_replans": sum(r["replans"] for r in rows) / len(rows),
                            "trials": len(rows)}
    return summary
