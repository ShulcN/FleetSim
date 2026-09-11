from .base import MAPFSolver


class WHCASolver(MAPFSolver):
    name = "whca"

    def solve(self, problem):
        from backend.research.planning import TimedProblem

        if not isinstance(problem, TimedProblem):
            raise ValueError(
                "WHCA* requires a temporal/mixed scenario; use the research scenario or migrate the legacy scenario"
            )
        return self.solve_timed(problem)
