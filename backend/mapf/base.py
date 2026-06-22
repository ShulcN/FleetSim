from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field


@dataclass
class MAPFProblem:
    graph: object
    starts: dict[str, str]
    goals: dict[str, str]
    constraints: list[object] = field(default_factory=list)


@dataclass
class MAPFSolution:
    paths: dict[str, list[str]]
    cost: float = 0.0
    success: bool = True
    message: str = ""


class MAPFSolver(ABC):
    name: str = "abstract"

    @abstractmethod
    def solve(self, problem: MAPFProblem) -> MAPFSolution:
        raise NotImplementedError
