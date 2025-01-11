from queue import PriorityQueue
from utils.Point import Point

class AStar:
    @staticmethod
    def heuristic(a: Point, b: Point) -> float:
        """Heurística: distância euclidiana"""
        return a.dist_to(b)

    @staticmethod
    def neighbors(node: Point, grid_size: float, obstacles: list[Point], min_dist: float) -> list[Point]:
        """Gera vizinhos válidos de um nó"""
        directions = [Point(1, 0), Point(-1, 0), Point(0, 1), Point(0, -1)]  #cima, baixo, esquerda, direita
        neighbors = []

        for direction in directions:
            neighbor = Point(node.x + direction.x * grid_size, node.y + direction.y * grid_size)

            #vrifica se está longe o suficiente de obstáculos
            if all(neighbor.dist_to(obstacle) > min_dist for obstacle in obstacles):
                neighbors.append(neighbor)

        return neighbors

    @staticmethod
    def search(start: Point, goal: Point, grid_size: float, obstacles: list[Point], min_dist: float, max_iterations=5000) -> list[Point]:
        """Executa o algoritmo A* com limite de iterações"""
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: AStar.heuristic(start, goal)}

        iterations = 0
        while not open_set.empty():
            iterations += 1
            if iterations > max_iterations:
                print("A* reached iteration limit without finding a solution.")
                break

            _, current = open_set.get()

            if current.dist_to(goal) < grid_size:  #se chegou ao objetivo
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for neighbor in AStar.neighbors(current, grid_size, obstacles, min_dist):
                tentative_g_score = g_score[current] + current.dist_to(neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + AStar.heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

        print("No path found.")
        return [] 