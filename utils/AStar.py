from queue import PriorityQueue
from utils.Point import Point
import time
import logging

class AStar:
    @staticmethod
    def heuristic(a: Point, b: Point) -> float:
        return a.dist_to(b)

    @staticmethod
    def neighbors(node: Point, grid_size: float, obstacles: list[Point], min_dist: float, goal: Point) -> list[Point]:
        directions = [
            Point(1, 0), Point(-1, 0), Point(0, 1), Point(0, -1)  # cima, baixo, esquerda, direita (remover diagonais)
        ]
        neighbors = []

        for direction in directions:
            neighbor = Point(node.x + direction.x * grid_size, node.y + direction.y * grid_size)

            if all(isinstance(obstacle, Point) and neighbor.dist_to(obstacle) > min_dist for obstacle in obstacles) or neighbor.dist_to(goal) < grid_size:
                neighbors.append(neighbor)

        return neighbors

    @staticmethod
    def search(start: Point, goal: Point, grid_size: float, obstacles: list[Point], min_dist: float, max_time: float = 2.0) -> list[Point]:
        start_time = time.time()
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: AStar.heuristic(start, goal)}
        iteration_count = 0

        while not open_set.empty():
            iteration_start_time = time.time()

            if time.time() - start_time > max_time:
                logging.warning("A* search timeout reached.")
                return []

            _, current = open_set.get()
            iteration_count += 1

            if current.dist_to(goal) < max(grid_size, min_dist * 0.5):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                logging.info(f"A* search successful in {iteration_count} iterations.")
                return path

            for neighbor in AStar.neighbors(current, grid_size, obstacles, min_dist, goal):
                tentative_g_score = g_score[current] + current.dist_to(neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + AStar.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

            iteration_time = time.time() - iteration_start_time
            logging.info(f"Iteration {iteration_count} took {iteration_time:.4f} seconds")

        logging.warning("A* search failed to find a path")
        return []

    @staticmethod
    def smooth_path(path: list[Point], obstacles: list[Point], min_dist: float) -> list[Point]:
        if not path:
            return []

        smoothed_path = [path[0]]
        for i in range(1, len(path)):
            if not AStar.line_collides(smoothed_path[-1], path[i], obstacles, min_dist):
                smoothed_path.append(path[i])

        final_path = [smoothed_path[0]]
        for i in range(1, len(smoothed_path) - 1):
            mid_point = Point(
                (smoothed_path[i].x + smoothed_path[i + 1].x) / 2,
                (smoothed_path[i].y + smoothed_path[i + 1].y) / 2
            )
            if not AStar.line_collides(final_path[-1], mid_point, obstacles, min_dist):
                final_path.append(mid_point)

            final_path.append(smoothed_path[i])

        final_path.append(smoothed_path[-1])

        return final_path

    @staticmethod
    def line_collides(start: Point, end: Point, obstacles: list[Point], min_dist: float) -> bool:
        for obstacle in obstacles:
            if not isinstance(obstacle, Point):
                logging.warning(f"Obstacle is not a Point: {obstacle}")
                continue
            distance = AStar.point_to_line_distance(start, end, obstacle)
            if distance < min_dist:
                return True
        return False

    @staticmethod
    def point_to_line_distance(start: Point, end: Point, point: Point) -> float:
        assert isinstance(start, Point), f"start is not a Point: {start}"
        assert isinstance(end, Point), f"end is not a Point: {end}"
        assert isinstance(point, Point), f"point is not a Point: {point}"

        numerator = abs((end.y - start.y) * point.x - (end.x - start.x) * point.y + end.x * start.y - end.y * start.x)
        denominator = ((end.y - start.y) ** 2 + (end.x - start.x) ** 2) ** 0.5
        return numerator / denominator