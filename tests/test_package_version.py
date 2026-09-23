from __future__ import annotations

from pathlib import Path
import tomllib


PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_VERSION = "1.0.0"


def test_project_and_lock_versions_match_release() -> None:
    with (PROJECT_ROOT / "pyproject.toml").open("rb") as file:
        project = tomllib.load(file)
    with (PROJECT_ROOT / "uv.lock").open("rb") as file:
        lock = tomllib.load(file)

    package = next(
        package
        for package in lock["package"]
        if package["name"] == project["project"]["name"]
    )

    assert project["project"]["version"] == EXPECTED_VERSION
    assert package["version"] == EXPECTED_VERSION
