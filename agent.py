from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.AStar import AStar
from hungarian_algorithm import Hungarian
import numpy as np
import time

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.3, target_tolerance=0.15):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs  #distância mínima para evitar obstáculos
        self.target_tolerance = target_tolerance  #tolerância para considerar o alvo atingido
        self.path = []  #caminho calculado pelo A*
        self.current_target = None  #próximo ponto no caminho
        self.critical_obstacle = None  #obstáculo crítico próximo ao alvo
        self.in_critical_zone = False  #estado: está na zona crítica


    def adjust_target_if_near_obstacle(self, target: Point, obstacles: list[Point]) -> Point:
        """
        Método para ajustar o alvo para evitar proximidade com obstáculos:
        se o alvo estiver muito próximo a um obstáculo, desloca o alvo para uma posição segura.
        """
        for obstacle in obstacles:
            if target.dist_to(obstacle) < self.min_dist_obs:
                #calcula a direção pra afastar o alvo do obstáculo
                direction = Point(target.x - obstacle.x, target.y - obstacle.y).normalize()
                adjusted_target = Point(
                    obstacle.x + direction.x * (self.min_dist_obs + 0.1),
                    obstacle.y + direction.y * (self.min_dist_obs + 0.1),
                )
                print(f"Adjusted target from {target} to {adjusted_target} due to proximity to obstacle {obstacle}.")
                return adjusted_target
        return target


    def decision(self):
        #verifica se há alvos disponíveis
        if not self.targets:
            print("No targets available. Stopping robot.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #posição atual do robô
        robot_pos = Point(self.robot.x, self.robot.y)
        target_pos = self.targets[0]

        #verificar se o alvo foi alcançado
        if robot_pos.dist_to(target_pos) < self.target_tolerance:
            print(f"Target reached: {target_pos}")
            self.targets.pop(0)  #remove o alvo que já foi alcançado
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #identificar se está na zona crítica
        obstacles = [Point(obstacle.x, obstacle.y) for obstacle in self.opponents.values()]
        in_critical_zone = any(
            target_pos.dist_to(obstacle) < self.min_dist_obs * 0.8 for obstacle in obstacles
        ) and robot_pos.dist_to(target_pos) < self.min_dist_obs * 1.5

        #tratamento para zona crítica!
        if in_critical_zone:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, target_pos, self.min_dist_obs)
            #reduz velocidade para que a colisão seja leve
            target_velocity = Point(target_velocity.x * 0.3, target_velocity.y * 0.3)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
            return

        #fora da zona crítica
        self.path = AStar.search(
            start=robot_pos,
            goal=target_pos,
            grid_size=0.2,
            obstacles=obstacles,
            min_dist=self.min_dist_obs,
            max_time=0.5,
        )

        if not self.path:
            print("No valid path found. Moving directly towards target.")
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, target_pos, self.min_dist_obs)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
            return

        #suaviza o caminho e vai para o próximo ponto
        self.path = AStar.smooth_path(self.path, obstacles, self.min_dist_obs)
        self.current_target = self.path.pop(0)

        target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.current_target, self.min_dist_obs)
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)

    @staticmethod
    def assign_targets_hungarian(robots: list[Point], targets: list[Point]) -> dict[int, int]:
        start_time = time.time()  # início da medição

        n_robots, n_targets = len(robots), len(targets)

        #criação da matriz de custo baseada na distância euclidiana
        cost_matrix = np.zeros((n_robots, n_targets))
        for i, robot in enumerate(robots):
            for j, target in enumerate(targets):
                cost_matrix[i][j] = robot.dist_to(target)

        cost_calc_time = time.time()
        print(f"Tempo para calcular a matriz de custo: {cost_calc_time - start_time:.4f}s")

        #solução com o algoritmo Húngaro
        assignments = Hungarian.solve(cost_matrix)

        solve_time = time.time()
        print(f"Tempo para resolver o algoritmo Húngaro: {solve_time - cost_calc_time:.4f}s")
        print(f"Tempo total de atribuição: {solve_time - start_time:.4f}s")

        #garante que a atribuição é válida e exclusiva
        assigned_targets = set()
        assignment_dict = {}
        for robot_idx, target_idx in assignments:
            if target_idx not in assigned_targets:
                assignment_dict[robot_idx] = target_idx
                assigned_targets.add(target_idx)

        return assignment_dict





    def post_decision(self):
        pass