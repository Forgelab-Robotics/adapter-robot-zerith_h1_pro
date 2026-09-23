from __future__ import annotations

from unittest.mock import patch

from typer.testing import CliRunner

from zerith_h1_pro.main import app


runner = CliRunner()


def test_move_actuator_rejects_gravity_before_driver_construction() -> None:
    with patch("zerith_h1_pro.main._build_driver") as build_driver:
        result = runner.invoke(
            app,
            [
                "move-actuator",
                "lift",
                "--target",
                "0.5",
                "--control-mode",
                "gravity_compensation_level",
                "--yes",
            ],
        )

    assert result.exit_code == 2
    assert "只允许 LOW_LEVEL" in result.output
    build_driver.assert_not_called()


def test_cli_help_remains_available_without_driver_construction() -> None:
    with patch("zerith_h1_pro.main._build_driver") as build_driver:
        root_help = runner.invoke(app, ["--help"])
        move_help = runner.invoke(app, ["move-actuator", "--help"])

    assert root_help.exit_code == 0
    assert "move-actuator" in root_help.output
    assert move_help.exit_code == 0
    assert "仅支持 LOW_LEVEL" in move_help.output
    build_driver.assert_not_called()
