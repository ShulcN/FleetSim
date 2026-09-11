from copy import deepcopy
from math import isfinite

from .base import MAPFProblem, MAPFSolution, MAPFSolver, ResourceUse
from .joint_astar import JointAStarSolver
from .cbs import CBSSolver

_COMMON = [
    dict(
        key="horizon_steps",
        label="Горизонт",
        description="В исследовательском режиме: число тактов planning_tick_s (4 × 0.5 с = 2 с). Манёвр резервируется целиком. В старом режиме: число следующих точек петли и синхронные раунды. Для WHCA* окно задаётся W.",
        type="integer",
        default=4,
        min=1,
        max=64,
        unit="шагов",
    ),
    dict(
        key="time_limit_s",
        label="Лимит времени поиска",
        description="Бюджет реального времени одного вызова A*. Проверяется между раскрытиями; не является жёстким таймером прерывания. Ускорение симуляции его не меняет.",
        type="number",
        default=0.2,
        min=0,
        max=10,
        step=0.01,
        unit="с",
    ),
    dict(
        key="max_expansions",
        label="Лимит раскрытий",
        description="Максимум обработанных состояний за вызов, включая промежуточный выбор действий AGV. Для первого совместного шага N роботов требуется не менее N раскрытий; конфликты увеличивают затраты. Без готового шага включается автономный режим.",
        type="integer",
        default=100000,
        min=1,
        max=1000000,
    ),
]
_SOLVERS = {}
_DESCRIPTORS = {}


def register_solver(
    name,
    factory,
    *,
    label=None,
    version="1",
    parameters=None,
    metrics=None,
    common_descriptions=None,
    description=None,
    help_url="/api/workspace/algorithm-help",
):
    """A solver supplies declarative fields/metrics; the UI has no solver-specific forms."""
    if name in _SOLVERS:
        raise ValueError(f"Solver already registered: {name}")
    fields = deepcopy(_COMMON + (parameters or []))
    for field in fields:
        if field["key"] in (common_descriptions or {}):
            field["description"] = common_descriptions[field["key"]]
    if len({p["key"] for p in fields}) != len(fields):
        raise ValueError("Duplicate solver parameter")
    _SOLVERS[name] = factory
    _DESCRIPTORS[name] = dict(
        id=name,
        label=label or name,
        version=version,
        parameters=fields,
        metrics=metrics or [],
        description=description or "",
        help_url=help_url,
    )


def solver_catalog():
    return deepcopy(list(_DESCRIPTORS.values()))


def validate_parameters(name, parameters=None):
    if name not in _SOLVERS:
        raise ValueError(f"Unknown MAPF solver: {name}")
    supplied = parameters or {}
    fields = _DESCRIPTORS[name]["parameters"]
    unknown = set(supplied) - {p["key"] for p in fields}
    if unknown:
        raise ValueError(f"Unknown solver parameters: {sorted(unknown)}")
    result = {}
    for field in fields:
        key = field["key"]
        value = supplied.get(key, field.get("default"))
        kind = field["type"]
        if kind in ("number", "integer"):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not isfinite(value)
            ):
                raise ValueError(f"{key}: expected a finite number")
            if kind == "integer" and int(value) != value:
                raise ValueError(f"{key}: expected an integer")
            if value < field.get("min", -float("inf")) or value > field.get(
                "max", float("inf")
            ):
                raise ValueError(f"{key}: outside allowed range")
        elif kind == "boolean" and not isinstance(value, bool):
            raise ValueError(f"{key}: expected boolean")
        elif kind in ("string", "enum") and not isinstance(value, str):
            raise ValueError(f"{key}: expected string")
        if "options" in field and value not in field["options"]:
            raise ValueError(f"{key}: unsupported value")
        result[key] = value
    return result


def create_solver(name, parameters=None):
    if name not in _SOLVERS:
        raise ValueError(f"Unknown MAPF solver: {name}")
    solver = _SOLVERS[name]()
    solver.configure(validate_parameters(name, parameters))
    return solver


register_solver(
    "joint_astar",
    JointAStarSolver,
    label="Совместный A*",
    description="A* в совместном пространстве с последовательным выбором действий каждого робота. В исследовательском режиме учитываются физическое время, ориентации и выполняемые манёвры AGV/AMR; в старом — синхронные шаги петель.",
)


register_solver(
    "cbs_astar",
    CBSSolver,
    label="CBS + A*",
    description="Два уровня поиска: индивидуальный A* строит движение робота (AGV по петле, AMR по свободному графу в исследовательском режиме), CBS находит конфликты и разветвляет дерево запретов. В каждой ветви перепланируется только затронутый робот. Ограничения зон и габаритов общие с совместным A*.",
    help_url="/api/workspace/algorithm-help?solver=cbs_astar",
    common_descriptions={
        "max_expansions": "Общий бюджет раскрытий за вызов: узлы дерева CBS плюс состояния всех индивидуальных A*. Счётчики уровней доступны отдельно. Число раскрытий напрямую не сопоставимо с совместным A*.",
        "time_limit_s": "Общий бюджет реального времени на весь CBS и все вложенные A*, не на каждый отдельный поиск. При исчерпании возвращается только проверенный совместный префикс.",
    },
    parameters=[
        dict(
            key="max_ct_nodes",
            label="Лимит узлов CBS",
            type="integer",
            default=2000,
            min=1,
            max=100000,
            description="Максимум раскрытых узлов дерева ограничений за вызов. Увеличение позволяет разобрать больше конфликтов, но требует времени и памяти; общий бюджет раскрытий также действует.",
        ),
        dict(
            key="max_wait_steps",
            label="Дополнительные ожидания",
            type="integer",
            default=16,
            min=0,
            max=256,
            unit="раундов",
            description="Максимум вставленных ожиданий до достижения цели окна для одного AGV. 0 запрещает задержки в пути. Малое значение может исключить решение; большое расширяет пространство поиска. Это не секунды и не автономная остановка 5 с.",
        ),
        dict(
            key="objective",
            label="Целевая функция",
            type="enum",
            default="makespan",
            options=["makespan", "sum_of_costs"],
            description="makespan — минимум раундов до прибытия последнего AGV; sum_of_costs — минимум суммы раундов прибытия всех AGV. В исследовательском режиме стоимость — длительность в тактах planning_tick_s; в старом — синхронные раунды.",
        ),
        dict(
            key="cache_low_level",
            label="Кэш индивидуальных A*",
            type="boolean",
            default=True,
            description="Повторно использовать результат для того же AGV и того же набора ограничений внутри одного вызова CBS. Уменьшает повторный поиск, расходует память. Между вызовами кэш очищается.",
        ),
    ],
    metrics=[
        dict(key=key, label=label)
        for key, label in [
            ("ct_expanded", "Раскрыто узлов CBS"),
            ("ct_generated", "Создано узлов CBS"),
            ("low_level_expanded", "Раскрыто состояний A*"),
            ("low_level_calls", "Вызовы индивидуального A*"),
            ("cache_hits", "Попадания в кэш"),
            ("conflicts_detected", "Обнаружения конфликтов"),
            ("peak_ct_open", "Максимум узлов в OPEN CBS"),
            ("max_constraint_depth", "Максимум ограничений в ветви"),
            ("makespan", "Длина совместного плана, раунды"),
            ("sum_of_costs", "Сумма раундов прибытия"),
        ]
    ],
)


from .whca import WHCASolver

register_solver(
    "whca",
    WHCASolver,
    label="WHCA* · смешанный флот",
    description="Приоритетное кооперативное A* в окне времени. Абстрактные расстояния по навигационному графу вычисляются обратным Дейкстрой и повторно используются как эвристика. Приоритет зависит от ожидания и расстояния до цели.",
    help_url="/api/workspace/algorithm-help?solver=whca",
    common_descriptions={
        "horizon_steps": "Только для старых моделей. В исследовательском WHCA* используется отдельный параметр W ниже.",
        "max_expansions": "Общий бюджет раскрытий пространственно-временного поиска за вызов. Статическая эвристика кэшируется и измеряется общим временем вызова.",
    },
    parameters=[
        dict(
            key="W",
            label="W · окно планирования",
            type="integer",
            default=16,
            min=2,
            max=128,
            unit="тактов",
            description="Окно во времени: W × planning_tick_s. При такте 0.5 с значение 16 означает 8 секунд. Текущий физический манёвр резервируется целиком, даже если заканчивается за границей окна.",
        ),
        dict(
            key="T_replan",
            label="Период перепланирования",
            type="integer",
            default=4,
            min=1,
            max=128,
            unit="тактов",
            description="Пересчёт каждые T_replan × planning_tick_s секунд. Не больше W. В полёте сохраняются выданные манёвры; новые разрешения выдаются независимо свободным роботам.",
        ),
        dict(
            key="C_wait",
            label="Стоимость ожидания",
            type="number",
            default=1,
            min=0.01,
            max=100,
            step=0.1,
            description="Стоимость одного такта ожидания в A*. Стоимость движения — число тактов. Большая стоимость повышает привлекательность обходов AMR; AGV не могут менять петлю.",
        ),
        dict(
            key="k_wait",
            label="Приоритет ожидания",
            type="number",
            default=1,
            min=0,
            max=100,
            step=0.1,
            description="Коэффициент накопленного вынужденного ожидания в секундах: P = k_wait × t_wait − k_goal × d_goal. Больший P планируется раньше.",
        ),
        dict(
            key="k_goal",
            label="Приоритет близости к цели",
            type="number",
            default=0.1,
            min=0,
            max=100,
            step=0.1,
            description="Коэффициент оставшегося расстояния до станции в метрах. Большое значение предпочитает близкие цели. Равенства разрешаются по ID.",
        ),
    ],
    metrics=[
        dict(key=k, label=l)
        for k, l in [
            ("low_level_expanded", "Раскрытия space-time A*"),
            ("reservation_checks", "Проверки резервирований"),
            ("planned_agents", "Спланировано роботов"),
            ("heuristic_nodes", "Вершины абстрактной эвристики"),
            ("window_s", "Окно, секунды"),
        ]
    ],
)
