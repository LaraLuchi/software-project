import numpy as np
from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.Geometry import Geometry

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.6):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs 

    def decision(self):
        if len(self.targets) == 0:
            return

        robot_pos = Point(self.robot.x, self.robot.y)
        target_pos = self.targets[0]

        obstacles = [Point(obstacle.x, obstacle.y) for obstacle in self.opponents.values()]

        obstacles_in_path = Geometry.is_obstacle_in_path(robot_pos, target_pos, obstacles, self.min_dist_obs)

        if obstacles_in_path:
            alternative_target = self.plan_alternative_target(robot_pos, target_pos, obstacles_in_path, self.min_dist_obs)
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, alternative_target)
        else:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, target_pos)

        self.set_vel(Point(target_velocity.x, target_velocity.y))
        self.set_angle_vel(target_angle_velocity)

    def plan_alternative_target(self, robot_pos: Point, target_pos: Point, obstacles_in_path: list[Point], min_dist_obs: float) -> Point:
        alternative_target = target_pos

        for obstacle_pos in obstacles_in_path:
            vector_to_obstacle = obstacle_pos - robot_pos
            vector_perpendicular = Point(-vector_to_obstacle.y, vector_to_obstacle.x).normalize()

            option_1 = obstacle_pos + vector_perpendicular * min_dist_obs
            option_2 = obstacle_pos - vector_perpendicular * min_dist_obs

            is_option_1_safe = all(option_1.dist_to(o) > min_dist_obs for o in obstacles_in_path)
            is_option_2_safe = all(option_2.dist_to(o) > min_dist_obs for o in obstacles_in_path)

            
            if is_option_1_safe and option_1.dist_to(target_pos) < option_2.dist_to(target_pos):
                alternative_target = option_1
            elif is_option_2_safe:
                alternative_target = option_2

        return alternative_target

    def post_decision(self):
        pass
