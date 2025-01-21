import numpy as np
from utils.Point import Point
from utils.AStar import AStar

class TaskAssignment:

    @staticmethod
    def assign_tasks(robots, targets, obstacles, grid_size=0.2, min_dist=0.18):
        """
        Atribui tarefas aos robôs utilizando o Algoritmo Húngaro:
        :param robots: lista de posições dos robôs (como objetos ou pontos)
        :param targets: lista de alvos (pontos)
        :param obstacles: lista de obstáculos no campo
        :param grid_size: tamanho da grade usada pelo A*
        :param min_dist: distância mínima dos robôs aos obstáculos
        :return: retorna um dicionário {robô_id: alvo_id}
        """
        cost_matrix = TaskAssignment.calculate_cost_matrix(robots, targets, obstacles, grid_size, min_dist)

        #resolve o problema de atribuição com o algoritmo húngaro
        from scipy.optimize import linear_sum_assignment
        robot_indices, target_indices = linear_sum_assignment(cost_matrix)

        #cria um dicionário de atribuições
        assignments = {robot_indices[i]: target_indices[i] for i in range(len(robot_indices))}
        return assignments


    @staticmethod
    def calculate_cost_matrix(robots, targets, obstacles, grid_size, min_dist):
        n_robots = len(robots)
        n_targets = len(targets)

        #ajusta dimensões da matriz
        if n_robots > n_targets:
            for _ in range(n_robots - n_targets):
                targets.append(Point(1e6, 1e6))  #alvos fictícios fora do campo
        elif n_targets > n_robots:
            for _ in range(n_targets - n_robots):
                robots.append(Point(-1e6, -1e6))  #robôs fictícios fora do campo

        #cria matriz de custos
        cost_matrix = [[0 for _ in range(len(targets))] for _ in range(len(robots))]

        for i, robot in enumerate(robots):
            for j, target in enumerate(targets):
                #calcular custo com A*
                path = AStar.search(
                    start=robot,
                    goal=target,
                    grid_size=grid_size,
                    obstacles=obstacles,
                    min_dist=min_dist,
                )
                if not path:
                    cost_matrix[i][j] = float('inf')  #caminho inviável
                else:
                    cost_matrix[i][j] = sum(
                        path[k].dist_to(path[k + 1]) for k in range(len(path) - 1)
                    )

        #substitui valores inf por um número grande finito
        max_cost = np.nanmax(cost_matrix)  #maior custo finito
        max_cost = max_cost if max_cost > 0 else 1e6  #garante um valor positivo
        cost_matrix = [
            [min(cost, max_cost * 10) for cost in row] for row in cost_matrix
        ]

        return cost_matrix
