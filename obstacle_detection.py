"""from utils.Geometry import Geometry
from utils.Point import Point


#primeiro, deve ser feita uma verificação se existe algum obstáculo no caminho entre o meu robô e o alvo
def is_obstacle_in_path(robot_pos: Point, target_pos: Point, obstacles: list[Point], min_dist_obs: float) -> list[Point]:
    obstacles_in_path = []


    for obstacle in obstacles:
        #calcula a distância perpendicular do obstaculo pro caminho
        distance = Geometry.perpendicular_distance(robot_pos, target_pos, obstacle)

        #verifica se o obstáculo está suficientemente próximo do caminho
        if distance <= min_dist_obs:
            #depois verifica se o obstáculo está entre o robô e o alvo
            if Geometry.is_between(robot_pos, target_pos, obstacle):
                obstacles_in_path.append(obstacle)

    return obstacles_in_path"""