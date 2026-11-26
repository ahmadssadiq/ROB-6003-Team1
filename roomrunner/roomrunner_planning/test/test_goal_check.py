from roomrunner_planning.goal_check import achieves, simulate

FETCH_RED = [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]


def test_simulate_fetch():
    assert simulate(FETCH_RED) == ("tray", None, {("red_block", "tray")})


def test_fetch_achieves_goal():
    assert achieves(FETCH_RED, {("on", "red_block", "tray")})


def test_wrong_block_fails():
    blue = [(s, t.replace("red", "blue")) for s, t in FETCH_RED]
    assert not achieves(blue, {("on", "red_block", "tray")})


def test_extra_block_moved_fails():
    extra = FETCH_RED + [("go_to", "blue_block"), ("pick", "blue_block"), ("go_to", "tray"), ("place", "tray")]
    assert not achieves(extra, {("on", "red_block", "tray")})


def test_return_home():
    assert achieves(FETCH_RED + [("go_to", "home")], {("on", "red_block", "tray"), ("robot-at", "home")})
    assert not achieves(FETCH_RED, {("on", "red_block", "tray"), ("robot-at", "home")})


def test_holding_goal():
    pick = [("go_to", "blue_block"), ("pick", "blue_block")]
    assert achieves(pick, {("holding", "blue_block")})
    assert not achieves(pick, {("robot-at", "blue_block")})      # ends holding something not asked for


def test_impossible_plan_fails():
    assert not achieves([("place", "tray")], {("on", "red_block", "tray")})
