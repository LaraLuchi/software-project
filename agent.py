from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.AStar import AStar

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.3, target_tolerance=0.15):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs  #distância mínima para evitar obstáculos
        self.target_tolerance = target_tolerance  #tolerância para considerar o alvo atingido
        self.path = []  #caminho calculado pelo A*
        self.current_target = None  #próximo ponto no caminho

    def adjust_target_if_near_obstacle(self, target: Point, obstacles: list[Point]) -> Point:
        """
        Ajusta o alvo para o ponto mais próximo seguro caso esteja muito próximo de um obstáculo.
        """
        for obstacle in obstacles:
            if target.dist_to(obstacle) < self.min_dist_obs:
                #desloca o alvo para um ponto seguro próximo
                direction = Point(target.x - obstacle.x, target.y - obstacle.y).normalize()
                adjusted_target = Point(
                    obstacle.x + direction.x * (self.min_dist_obs + 0.1),
                    obstacle.y + direction.y * (self.min_dist_obs + 0.1),
                )
                print(f"Adjusted target from {target} to {adjusted_target} due to proximity to obstacle.")
                return adjusted_target
        return target

    def decision(self):
        #verifica se há alvos disponíveis
        if not self.targets:
            print("No targets available. Stopping robot.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #oosição atual do robô
        robot_pos = Point(self.robot.x, self.robot.y)

        # Verificar se o alvo foi alcançado
        if self.current_target and robot_pos.dist_to(self.current_target) < self.target_tolerance:
            if self.path:
                self.current_target = self.path.pop(0)  #próximo ponto no caminho
            else:
                print(f"Target reached: {self.targets[0]}")
                self.targets.pop(0)  #remove o alvo atingido
                self.current_target = None

        #calcular novo caminho se necessário
        if not self.path or not self.current_target:
            if not self.targets:
                print("All targets reached.")
                self.set_vel(Point(0.0, 0.0))
                self.set_angle_vel(0.0)
                return

            target_pos = self.targets[0]
            print(f"New target acquired: {target_pos}")

            obstacles = [Point(obstacle.x, obstacle.y) for obstacle in self.opponents.values()]

            #ajustar o alvo para evitar proximidade com obstáculos
            adjusted_target = self.adjust_target_if_near_obstacle(target_pos, obstacles)

            #calcular o caminho com A*
            self.path = AStar.search(
                start=robot_pos,
                goal=adjusted_target,
                grid_size=0.2,
                obstacles=obstacles,
                min_dist=self.min_dist_obs,
                max_time=0.5,
            )

            if not self.path:
                print("No valid path found. Moving directly towards adjusted target.")
                #mover diretamente p alvo ajustado
                target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, adjusted_target)
                self.set_vel(target_velocity)
                self.set_angle_vel(target_angle_velocity)
                return

            #suaviza o caminho para melhorar os movimentos
            self.path = AStar.smooth_path(self.path, obstacles, self.min_dist_obs)
            print(f"Path calculated: {self.path}")
            self.current_target = self.path.pop(0)

        #navega para o próximo ponto no caminho
        if self.current_target:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.current_target)

            #verifica se alcançou o ponto atual
            if robot_pos.dist_to(self.current_target) < 0.1:
                if self.path:
                    self.current_target = self.path.pop(0)
                else:
                    self.current_target = None  #concluir o caminho

            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
        else:
            print("Path completed. Waiting for next target.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)

    def post_decision(self):
        pass
