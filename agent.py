from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.Geometry import Geometry
from utils.AStar import AStar  

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.5, grid_size=0.5):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs 
        self.grid_size = grid_size       #define o grid pra usar o algoritmo A*
        self.path = []                   #caminho planejado pelo A*

    def decision(self):
        if len(self.targets) == 0:
            print("No targets available.")
            return

        #posição atual, alvo e obstáculos
        robot_pos = Point(self.robot.x, self.robot.y)
        target_pos = self.targets[0]
        obstacles = [Point(obs.x, obs.y) for obs in self.opponents.values()]
        print(f"Robot Position: {robot_pos}, Target: {target_pos}, Obstacles: {obstacles}")

        #se não tiver caminho ou o alvo mudou, recalcula o caminho
        if not self.path or self.path[-1] != target_pos:
            print("Calculating new path...")
            self.path = AStar.search(
                start=robot_pos,              #posição inicial
                goal=target_pos,              #posição do alvo
                grid_size=self.grid_size,     #tamanho do grid // definir como 0.5 ou maior
                obstacles=obstacles,          #lista de obstáculos
                min_dist=self.min_dist_obs,   #distância mínima dos obstáculos
                max_iterations=5000           #limite de iterações pra não ficar rodando infinitamente
            )
            print(f"Generated Path: {self.path}")       #teste no terminal

        #seguir o próximo ponto no caminho
        if self.path:
            next_point = self.path.pop(0)  #remove o próximo ponto do caminho
            print(f"Next Point: {next_point}")

            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_point)
        else:
            #se não tiver pontos no caminho, segue direto para o alvo
            print("No path found, going directly to target.")
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, target_pos)

        #define velocidades p simulador
        print(f"Velocity: {target_velocity}, Angular Velocity: {target_angle_velocity}")
        self.set_vel(target_velocity)
        self.set_angle_vel(target_angle_velocity)


    def post_decision(self):
        pass
