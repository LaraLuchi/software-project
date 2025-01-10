import math
from utils.Point import Point

class Geometry:

    @staticmethod
    def modularize(x, mod) -> float:
        """Make a value modular between 0 and mod"""
        if not (-mod <= x < mod):
            if isinstance(x, float):
                x = math.fmod(x, mod)
            else:
                x %= mod

        if x < 0:
            x += mod

        return x

    @staticmethod
    def normalize_angle(value, center=0, amplitude=math.pi) -> float:
        value = value % (2 * amplitude)
        if value < -amplitude + center:
            value += 2 * amplitude
        elif value > amplitude + center:
            value -= 2 * amplitude
        return value
    
    @staticmethod
    def dist_to(p_1: Point, p_2: Point) -> float:
        """Returns the distance between two points"""
        return ((p_1.x - p_2.x) ** 2 + (p_1.y - p_2.y) ** 2) ** 0.5

    @staticmethod
    def smallest_angle_diff(angle_a: float, angle_b: float) -> float:
        """Returns the smallest angle difference between two angles"""
        angle: float = Geometry.modularize(angle_b - angle_a, 2 * math.pi)
        if angle >= math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def abs_smallest_angle_diff(angle_a: float, angle_b: float) -> float:
        """Returns the absolute smallest angle difference between two angles"""
        return abs(Geometry.smallest_angle_diff(angle_a, angle_b))
    
    @staticmethod
    def from_polar(length: float, angle: float) -> Point:
        return Point(math.cos(angle) * length, math.sin(angle) * length)
    

    #......... Modificações feitas por mim a partir daqui ...........


#primeiro, deve ser feita uma verificação se existe algum obstáculo no caminho entre o meu robô e o alvo
    @staticmethod
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

        return obstacles_in_path
    
    @staticmethod
    def perpendicular_distance(p1: Point, p2: Point, p3: Point) -> float:
        """calcula a distância perpendiculare do obstáculo (ponto p3) até o caminho "traçado" entre os pontos p1 e p2.
        esse cálculo se refere ao cálculo da distância entre um ponto e um plano na geometria analítica, e devido a 
        complexidade para representar a fórmula inteira ela será dividida em duas partes (numerador e divisor) para facilitar o cálculo pelo programa.
        """
        numerator = abs(p3.x * (p2.y - p1.y) - p3.y * (p2.x - p1.x) + p1.y * p2.x  -  p1.x * p2.y)
        denominator = ((p2.y -p1.y)**2 + (p2.x - p1.x)**2)**0.5
        return numerator / denominator


    @staticmethod
    def is_between(p1: Point, p2: Point, p3: Point) -> bool:
        #checar se o ponto p3 está entre p1 e p2
        """cross_product: produto vetorial entre os vetores p1p2 e p1p3.
        verifica se o produto vetorial é 0, pois, caso seja, significa que esses vetores estão na mesma linha,
        indicando que o ponto p3 está na linha p1-p2.
        """
        cross_product = (p3.y - p1.y) * (p2.x - p1.x) - (p3.x - p1.x) * (p2.y - p1.y)
        if abs(cross_product) > 1e-6:
            return False
        
        """" dot_product: produto escalar entre os vetores p1p2 e p1p3
        se o resultado for positivo, p3 está na direção de p2, caso contrário tá na direção "oposta"
        (antes de p1, fora do segmento p1p2) """

        dot_product = (p3.x - p1.x) * (p2.x - p1.x) + (p3.y - p1.y) * (p2.y - p1.y)
        if dot_product < 0:
            return False
        
        """"squared_lenght: comprimento do segmento p1p2;
            e depois compara o comprimento com o produto escalar, para verificar se o produto escalar cresce com a distância 
            ao longo do segmento (se o produto escalar for maior que o comprimento, significa que p3 está depois de p2)
        """
        squared_length = (p2.x - p1.x)**2 + (p2.y - p1.y)**2
        if dot_product > squared_length:
            return False
        
        return True
    
    """"se todas essas verificações forem verdadeiras, significa que p3 está no caminho p1-p2 (p3 é um obstáculo)"""