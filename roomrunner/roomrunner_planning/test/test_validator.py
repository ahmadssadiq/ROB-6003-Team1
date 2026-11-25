from roomrunner_planning.validator import validate

OBJECTS = {"red_block": "block", "blue_block": "block", "tray": "tray"}
PLACES = {"home"}
FETCH = [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]


def test_valid_fetch():
    assert validate(FETCH, OBJECTS, PLACES) == []


def test_unknown_object():
    errors = validate([("go_to", "green_block")], OBJECTS, PLACES)
    assert len(errors) == 1 and "not in the room" in errors[0]


def test_place_without_holding():
    assert any("not holding" in e for e in validate([("go_to", "tray"), ("place", "tray")], OBJECTS, PLACES))


def test_pick_without_going_there():
    assert any("go_to red_block" in e for e in validate([("pick", "red_block")], OBJECTS, PLACES))


def test_pick_while_holding():
    steps = [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "blue_block"), ("pick", "blue_block")]
    assert any("already holding" in e for e in validate(steps, OBJECTS, PLACES))


def test_cannot_pick_tray():
    assert any("only blocks" in e for e in validate([("go_to", "tray"), ("pick", "tray")], OBJECTS, PLACES))


def test_unreachable():
    errors = validate(FETCH, OBJECTS, PLACES, reachable=lambda target: target != "tray")
    assert any("no path to tray" in e for e in errors)


def test_two_objects_and_home():
    steps = FETCH + [("go_to", "blue_block"), ("pick", "blue_block"), ("go_to", "tray"), ("place", "tray"),
                     ("go_to", "home")]
    assert validate(steps, OBJECTS, PLACES) == []
