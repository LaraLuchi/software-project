from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.AStar import AStar


class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.3, target_tolerance=0.1):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs  #margem de segurança para obstáculos
        self.target_tolerance = target_tolerance  #tolerância para considerar um alvo atingido
        self.path = []  #caminho calculado pelo A*
        self.current_target = None  #alvo atual no caminho

    def decision(self):
        #verificar se existem alvos disponíveis
        if not self.targets:
            print("No targets available. Stopping robot.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #posição atual do robô
        robot_pos = Point(self.robot.x, self.robot.y)

        #verificar se chegou ao alvo final
        if self.current_target and robot_pos.dist_to(self.current_target) < self.target_tolerance:
            if self.path:
                self.current_target = self.path.pop(0)  #próximo ponto no caminho
            else:
                print(f"Target reached: {self.targets[0]}")
                self.targets.pop(0)  #remove o alvo alcançado
                self.current_target = None

        #calcula um novo caminho, caso  necessário
        if not self.path or not self.current_target:
            if not self.targets:
                print("All targets reached.")
                self.set_vel(Point(0.0, 0.0))
                self.set_angle_vel(0.0)
                return

            target_pos = self.targets[0]
            print(f"New target acquired: {target_pos}")
            print("Calculating new path...")

            obstacles = [Point(obstacle.x, obstacle.y) for obstacle in self.opponents.values()]
            
            #calcula o caminho a partir do algoritmo A*
            self.path = AStar.search(
                start=robot_pos,
                goal=target_pos,
                grid_size=0.2,
                obstacles=obstacles,
                min_dist=self.min_dist_obs,
                max_time=1.5
            )

            if not self.path:
                print("No valid path found. Stopping robot.")
                self.set_vel(Point(0.0, 0.0))
                self.set_angle_vel(0.0)
                return

            #suaviza o caminho para melhorar os movimentos
            self.path = AStar.smooth_path(self.path, obstacles, self.min_dist_obs)
            print(f"Path calculated: {self.path}")
            self.current_target = self.path.pop(0)

        #navega para o próximo ponto no caminho
        if self.current_target:
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, self.current_target)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)
        else:
            print("Path completed. Waiting for next target.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)

    def post_decision(self):
        pass
