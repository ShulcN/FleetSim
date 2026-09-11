from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class ResourceUse:
    """Exclusive zone (direction=0), or corridor allowing only one direction."""

    key: str
    direction: int = 0


@dataclass
class MAPFProblem:
    graph: Any
    starts: dict[str, str]
    goals: dict[str, str]
    constraints: list[object] = field(default_factory=list)
    # Ordered, finite paths for a rolling planning window; repeated nodes are allowed.
    fixed_paths: dict[str, list[str]] = field(default_factory=dict)
    # Per-path-position resources. A move holds the union of both endpoints.
    resources: dict[str, list[tuple[ResourceUse, ...]]] = field(default_factory=dict)
    radii: dict[str, float] = field(default_factory=dict)
    clearance: float = 1.0
    time_limit_s: float = 0.2
    max_expansions: int = 100_000


@dataclass
class MAPFSolution:
    paths: dict[str, list[str]]
    cost: float = 0.0
    success: bool = True
    message: str = ""
    status: str = "solved"  # solved | partial | timeout | infeasible
    expanded: int = 0
    elapsed_s: float = 0.0
    metrics: dict[str, Any] = field(default_factory=dict)


class MAPFSolver(ABC):
    name: str = "abstract"

    def configure(self, parameters: dict) -> None:
        self.parameters = parameters

    def solve_timed(self, problem):
        from backend.research.planning import solve_timed

        return solve_timed(problem, self.name, self.parameters)

    @abstractmethod
    def solve(self, problem: MAPFProblem) -> MAPFSolution:
        """Return synchronized node sequences, including explicit waits.

        A partial solution is an executable, validated prefix, not a claim that
        the goals were reached. The executor must retain reservations until all
        robots finish a step; durations are determined by actual motion.
        """
        raise NotImplementedError
