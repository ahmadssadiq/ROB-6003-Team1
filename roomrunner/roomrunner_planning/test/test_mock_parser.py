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
