# Contributing

## Branches

Each owner works on their own branch and merges into `main` through a pull request.

| Branch | Owner | Packages |
| --- | --- | --- |
| `ahmad/planning` | Ahmad Sadiq | `roomrunner_planning` |
| `mourya/perception` | Aditya Mourya | `roomrunner_perception`, `roomrunner_localization`, `roomrunner_control` |
| `sarvanagupta/manipulation` | Aditya Sarvanagupta | `roomrunner_manipulation` |

`roomrunner_interfaces` and `roomrunner_bringup` are shared. Change them in a small pull request of their
own and tell the others, because their code depends on them.

Start a branch from the latest `main`:

```bash
git switch main && git pull
git switch -c ahmad/planning        # once; later: git switch ahmad/planning && git merge main
```

## Pull requests

- Keep them small: one working step per pull request.
- Before opening one: the workspace builds (`colcon build`), the tests pass (`pytest` in the repository
  root and `colcon test`), and `ros2 launch roomrunner_bringup stubs.launch.py` still runs to `DONE`.
- Another team member reviews and merges.
- Do not change assignment code (`assignment_02.py`, `assignment_03/`, `assignment_04/`). Import or launch
  it instead.

## Rules

- Units: metres and radians everywhere. Convert only at the boundary with existing nodes that use other
  units (`/base_commands` turn rate is deg/s, `/arm_commands` is degrees), and say so in a comment.
- Every frame and transform goes into [docs/frames.md](docs/frames.md).
- No secrets in git: API keys go in `.env` (see `.env.example`).
- No large files in git: ROS bags, datasets and model weights are ignored (see `.gitignore`).
- Simple, readable Python that we can explain in the presentation.

## Milestones

| Milestone | Dates | Goal |
| --- | --- | --- |
| Week 1 | Nov 24 to 30 | each module works alone, with stubs or in simulation |
| Week 2 | Dec 1 to 7 | live on hardware; first end-to-end run with a fixed request and scripted grasp by Dec 7 |
| Week 3 | Dec 8 to 11 | LLM and learned pickup, experiments E1 and E2, plots. Code freeze Dec 11 |
| Final | Dec 14 | final presentation and project website |
