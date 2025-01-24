from queue import PriorityQueue
from utils.Point import Point
import time


class AStar:
    @staticmethod
    def heuristic(a: Point, b: Point) -> float:
        return a.dist_to(b)

    @staticmethod
    def neighbors(node: Point, grid_size: float, obstacles: list[Point], min_dist: float, goal: Point) -> list[Point]:
        #era vizinhos válidos de um nó
        directions = [
            Point(1, 0), Point(-1, 0), Point(0, 1), Point(0, -1),  #cima, baixo, esquerda, direita
            Point(1, 1), Point(-1, -1), Point(1, -1), Point(-1, 1)  #diagonais
        ]
        neighbors = []

        for direction in directions:
            neighbor = Point(node.x + direction.x * grid_size, node.y + direction.y * grid_size)

            #verifica se está longe o suficiente de obstáculos
            """if all(neighbor.dist_to(obstacle) > min_dist for obstacle in obstacles):
                neighbors.append(neighbor)"""
            if all(neighbor.dist_to(obstacle) > min_dist for obstacle in obstacles) or neighbor.dist_to(goal) < grid_size:
                neighbors.append(neighbor)


        return neighbors

    @staticmethod
    def search(start: Point, goal: Point, grid_size: float, obstacles: list[Point], min_dist: float, max_time: float = 1.0) -> list[Point]:
        #executa o algoritmo A*
        start_time = time.time()
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: AStar.heuristic(start, goal)}

        while not open_set.empty():
            #checa limite de tempo do algoritmo --> pra não rodar infinitamente e travar
            if time.time() - start_time > max_time:
                print("A* search timeout reached.")
                return []

            _, current = open_set.get()

            #verificar se alcançou o objetivo
            """if current.dist_to(goal) < grid_size:"""
            if current.dist_to(goal) < max(grid_size, min_dist * 0.5):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            #expande vizinhos
            for neighbor in AStar.neighbors(current, grid_size, obstacles, min_dist, goal):
                tentative_g_score = g_score[current] + current.dist_to(neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + AStar.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

        print("A* search failed to find a path")
        return []

    @staticmethod
    def smooth_path(path: list[Point], obstacles: list[Point], min_dist: float) -> list[Point]:
        #suaviza o caminho gerado pelo A*
        if not path:
            return []

        smoothed_path = [path[0]]

        for i in range(1, len(path)):
            if not AStar.line_collides(smoothed_path[-1], path[i], obstacles, min_dist):
                smoothed_path.append(path[i])

        return smoothed_path

    @staticmethod
    def line_collides(start: Point, end: Point, obstacles: list[Point], min_dist: float) -> bool:
        #verifica se um segmento de linha colide com algum obstáculo
        for obstacle in obstacles:
            distance = AStar.point_to_line_distance(start, end, obstacle)
            if distance < min_dist:
                return True
        return False

    @staticmethod
    def point_to_line_distance(start: Point, end: Point, point: Point) -> float:
        #calcula a distância de um ponto a uma linha
        numerator = abs((end.y - start.y) * point.x - (end.x - start.x) * point.y + end.x * start.y - end.y * start.x)
        denominator = ((end.y - start.y) ** 2 + (end.x - start.x) ** 2) ** 0.5
        return numerator / denominator