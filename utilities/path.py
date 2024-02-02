from dataclasses import dataclass
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from utilities.position import StageLegs


@dataclass
class Path:
    waypoints: list[Translation2d]
    final_heading: Rotation2d


@dataclass
class NotePaths:
    # All paths assume RED alliance
    # They will automatically be flipped if we are blue
    pick_up_path: Path
    shoot_path: Path


class Line:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def closest_point_on_segment(self, point):
        dx, dy = self.end.x - self.start.x, self.end.y - self.start.y
        length_squared = (self.end.x - self.start.x) ** 2 + (
            self.end.y - self.start.y
        ) ** 2
        if length_squared == 0:
            return self.start.x, self.start.y
        t = max(
            0,
            min(
                1,
                ((point.x - self.start.x) * dx + (point.y - self.start.y) * dy)
                / length_squared,
            ),
        )
        closest_x = self.start.x + t * dx
        closest_y = self.start.y + t * dy
        return closest_x, closest_y

    def dist_to(self, point):
        closest_x, closest_y = self.closest_point_on_segment(point)
        return ((closest_x - point.x) ** 2 + (closest_y - point.y) ** 2) ** 0.5

    def dist_to_points(self, points: list[Pose2d]):
        mindist = -1
        mindistid = -1
        for point in points:
            distance = self.dist_to(point.translation())
            if distance < mindist or mindist == -1:
                mindist = distance
                mindistid = points.index(point)
        return mindist, mindistid


def Make_Path(
    current_pose: Pose2d, waypoints: list[Translation2d], final_heading: Rotation2d
) -> Path:
    """This is to make a path but take into account the stage legs and avoid them, using "pathfinding" (My own less-rich algorithms) to do so quickly"""

    # We need to check if the path intersects with any of the stage legs
    # If it does, we need to find a way around it
    line = Line(current_pose, waypoints[0])
    dist, idx = line.dist_to_points(StageLegs)
    if dist < 0.5:
        # We need to go around it
        pass

    return Path(waypoints, final_heading)
