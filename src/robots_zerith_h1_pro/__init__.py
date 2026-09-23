"""Zerith H1 Pro Forge robot package."""

from __future__ import annotations

from typing import Any

from .config import ZERITH_H1_PRO_ACTUATOR_ORDER, ZERITH_H1_PRO_ACTUATOR_SPECS

__all__ = [
    "ZERITH_H1_PRO_ACTUATOR_ORDER",
    "ZERITH_H1_PRO_ACTUATOR_SPECS",
    "ZerithH1ProDriver",
]


def __getattr__(name: str) -> Any:
    """Load the native-SDK-facing driver only when explicitly requested."""
    if name == "ZerithH1ProDriver":
        from .driver import ZerithH1ProDriver

        return ZerithH1ProDriver
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
