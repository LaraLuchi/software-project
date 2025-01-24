from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from utils.AStar import AStar
from utils.task_assignment import closest_task_assignment

def greedy_task_assignment(robot_positions, target_positions):
    """
    Distribui alvos entre os robôs usando uma abordagem gananciosa baseada na distância,
    garantindo que cada alvo seja atribuído a apenas um robô.
    :param robot_positions: Lista de posições dos robôs.
    :param target_positions: Lista de posições dos alvos.
    :return: Um dicionário {robô_id: alvo_id}.
    """
    assignments = {}
    assigned_targets = set()

    #calcula as distâncias entre cada robô e cada alvo
    distances = [
        (robot_id, target_id, robot_positions[robot_id].dist_to(target_positions[target_id]))
        for robot_id in range(len(robot_positions))
        for target_id in range(len(target_positions))
    ]

    #ordena os pares (robô, alvo) pela distância mais próxima primeiro
    distances.sort(key=lambda x: x[2])

    #realiza a alocação gananciosa, garantindo que cada alvo seja atribuído apenas uma vez
    for robot_id, target_id, _ in distances:
        if robot_id not in assignments and target_id not in assigned_targets:
            assignments[robot_id] = target_id
            assigned_targets.add(target_id)

    return assignments


class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, min_dist_obs=0.3, target_tolerance=0.15):
        super().__init__(id, yellow)
        self.min_dist_obs = min_dist_obs  #distância mínima para evitar obstáculos
        self.target_tolerance = target_tolerance  #tolerância para considerar o alvo atingido
        self.path = []  #caminho calculado pelo A*
        self.current_target = None  #próximo ponto no caminho

    def adjust_target_if_near_obstacle(self, target: Point, obstacles: list[Point]) -> Point:
        """
        Método para ajustar o alvo para evitar proximidade com obstáculos.
        """
        for obstacle in obstacles:
            if target.dist_to(obstacle) < self.min_dist_obs:
                direction = Point(target.x - obstacle.x, target.y - obstacle.y).normalize()
                adjusted_target = Point(
                    obstacle.x + direction.x * (self.min_dist_obs + 0.1),
                    obstacle.y + direction.y * (self.min_dist_obs + 0.1),
                )
                return adjusted_target
        return target

    def decision(self):
        """
        Lógica de decisão para o agente.
        """
        #verifica se há alvos disponíveis
        if not self.targets:
            print("No targets available. Stopping robot.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #obtem as posições dos robôs (atualmente, apenas este robô) e dos alvos
        robot_positions = [Point(self.robot.x, self.robot.y)]
        target_positions = self.targets

        #realiza a atribuição de tarefas com base no alvo mais próximo
        assignments = closest_task_assignment(robot_positions, target_positions)

        #verifica se este robô recebeu um alvo
        if 0 not in assignments:  # O ID do robô atual é 0
            print("No target assigned to this robot.")
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #obtem o alvo atribuído para este robô
        target_pos = target_positions[assignments[0]]

        #verifica se o alvo foi alcançado
        robot_pos = Point(self.robot.x, self.robot.y)
        if robot_pos.dist_to(target_pos) < self.target_tolerance:
            print(f"Target reached: {target_pos}")
            self.targets.remove(target_pos)  #remove o alvo alcançado
            self.set_vel(Point(0.0, 0.0))
            self.set_angle_vel(0.0)
            return

        #planejamento do caminho usando A*
        obstacles = [Point(obstacle.x, obstacle.y) for obstacle in self.opponents.values()]
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




    def post_decision(self):
        pass
