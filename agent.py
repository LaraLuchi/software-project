from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.AStar import AStar
from hungarian_algorithm import Hungarian
import numpy as np
import time
import logging

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.3, target_tolerance=0.15):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs  # distância mínima para evitar obstáculos
        self.target_tolerance = target_tolerance  # tolerância para considerar o alvo atingido
        self.path = []  # caminho calculado pelo A*
        self.current_target = None  # próximo ponto no caminho
        self.critical_obstacle = None  # obstáculo crítico próximo ao alvo
        self.in_critical_zone = False  # estado: está na zona crítica


    def adjust_target_if_near_obstacle(self, target: Point, obstacles: list[Point]) -> Point:
        for obstacle in obstacles:
            if target.dist_to(obstacle) < self.min_dist_obs:
                direction = Point(target.x - obstacle.x, target.y - obstacle.y).normalize()
                adjusted_target = Point(
                    obstacle.x + direction.x * (self.min_dist_obs + 0.1),
                    obstacle.y + direction.y * (self.min_dist_obs + 0.1),
                )
                print(f"Adjusted target from {target} to {adjusted_target} due to proximity to obstacle {obstacle}.")
                return adjusted_target
        return target


    def decision(self):
        if not self.targets:
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        robot_pos = Point(self.robot.x, self.robot.y)
        target_pos = self.targets[0]

        if self.current_target is None or robot_pos.dist_to(self.current_target) < self.target_tolerance:
            if robot_pos.dist_to(target_pos) < self.target_tolerance:
                self.targets.pop(0)
                self.set_vel(Point(0.0, 0.0))
                self.set_angle_vel(0.0)
                return

            self.path = AStar.search(
                start=robot_pos,
                goal=target_pos,
                grid_size=0.2,
                obstacles=[Point(obstacle.x, obstacle.y) for obstacle in self.opponents.values()],
                min_dist=self.min_dist_obs,
                max_time=2.0,
            )

            if not self.path:
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, target_pos, self.min_dist_obs)
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)
                return

            self.path = AStar.smooth_path(self.path, self.opponents, self.min_dist_obs)
            self.current_target = self.path.pop(0)

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.current_target, self.min_dist_obs)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

    @staticmethod
    def assign_targets_hungarian(robots: list[Point], targets: list[Point]) -> dict[int, int]:
        start_time = time.time()

        n_robots, n_targets = len(robots), len(targets)
        cost_matrix = np.zeros((n_robots, n_targets))
        for i, robot in enumerate(robots):
            for j, target in enumerate(targets):
                cost_matrix[i][j] = robot.dist_to(target)

        cost_calc_time = time.time()
        logging.info(f"Time for cost matrix calculation: {cost_calc_time - start_time:.4f}s")

        assignments = Hungarian.solve(cost_matrix)

        solve_time = time.time()
        logging.info(f"Time for Hungarian algorithm: {solve_time - cost_calc_time:.4f}s")
        logging.info(f"Total assignment time: {solve_time - start_time:.4f}s")

        assigned_targets = set()
        assignment_dict = {}
        for robot_idx, target_idx in assignments:
            if target_idx not in assigned_targets:
                assignment_dict[robot_idx] = target_idx
                assigned_targets.add(target_idx)

        return assignment_dict

    def post_decision(self):
        pass