import pytest

from roomrunner_planning.mock_parser import parse


def test_fetch():
    assert parse("Bring the red block to the tray.") == [
        ("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]


def test_several_steps():
    assert parse("bring the blue block to the tray then go home") == [
        ("go_to", "blue_block"), ("pick", "blue_block"), ("go_to", "tray"), ("place", "tray"), ("go_to", "home")]


def test_pick_and_place_separately():
    assert parse("pick up the red block, put it on the tray") == [
        ("go_to", "red_block"), ("pick", "red_block"), ("go_to", "tray"), ("place", "tray")]


def test_not_understood():
    with pytest.raises(ValueError):
        parse("dance")


def test_take_home():
    assert parse("take the red block home") == [("go_to", "red_block"), ("pick", "red_block"), ("go_to", "home"),
                                                 ("place", "home")]


def test_two_objects_in_one_command():
    steps = parse("put the red block and the blue block on the tray")
    assert [s for s in steps if s[0] == "pick"] == [("pick", "red_block"), ("pick", "blue_block")]
    assert steps.count(("place", "tray")) == 2


def test_and_before_a_new_command_still_splits():
    assert parse("put the green block on the tray and go home")[-1] == ("go_to", "home")
