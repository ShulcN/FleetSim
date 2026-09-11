from backend.collisions import CollisionChecker
from backend.robots.factory import create_robot_from_config


def robot_at(robot_id, x, y):
    return create_robot_from_config(
        {
            "id": robot_id,
            "type": "differential_drive",
            "initial_pose": {"x": x, "y": y, "theta": 0.0},
            "footprint": {"length": 0.8, "width": 0.45},
        }
    )


def test_robots_overlap_is_a_collision():
    checker = CollisionChecker()
    robots = [robot_at("R1", 1.0, 1.0), robot_at("R2", 1.0, 1.0)]
    events = checker.check(robots, dxf_map=None, sim_time=0.0)
    assert len(events) == 1
    assert events[0].type == "robot_robot"


def test_far_robots_do_not_collide():
    checker = CollisionChecker()
    robots = [robot_at("R1", 0.0, 0.0), robot_at("R2", 5.0, 5.0)]
    events = checker.check(robots, dxf_map=None, sim_time=0.0)
    assert events == []


def test_wall_index_handles_boundaries_and_replacing_map():
    from backend.maps.dxf_map import DxfMap, Segment2D, Point2D
    checker = CollisionChecker()
    robot = robot_at('R',9.8,-0.1)
    wall = DxfMap(segments=[Segment2D(Point2D(10,-20),Point2D(10,20),'wall')])
    events = checker.check([robot],wall,0)
    assert events and events[0].other_id == 'wall:0:wall'
    far = DxfMap(segments=[Segment2D(Point2D(30,-20),Point2D(30,20),'wall')])
    assert not checker.check([robot],far,1)
