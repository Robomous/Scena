# SPDX-License-Identifier: MIT
"""Kinema: scenario execution engine for autonomous-driving simulation.

Executes scenarios described in ASAM OpenSCENARIO through a step-based API:
the host simulator owns the clock and drives the engine with init / step /
query / close.
"""

from ._kinema import (
    Action,
    Condition,
    ControlMode,
    Engine,
    Entity,
    EntityState,
    Scenario,
    SimulationTimeCondition,
    SpeedAction,
    Status,
    __version__,
    version,
)

__all__ = [
    "Action",
    "Condition",
    "ControlMode",
    "Engine",
    "Entity",
    "EntityState",
    "Scenario",
    "SimulationTimeCondition",
    "SpeedAction",
    "Status",
    "__version__",
    "version",
]
