import math

# Clase Map
class Map:
    def __init__(self):
        self.obstacles_coordinates = [] # Conjunto de coordenadas de obstáculos descubiertos

    def add_obstacle(self, x, y):
        self.obstacles_coordinates.append([int(x),int(y)])

    def is_obstacle(self, x, y):
        if [int(x),int(y)] is self.obstacles_coordinates:
            return True
        else:
            return False
            
class Mapping:
    def __init__(self, my_map, receiver_data):
        self.map = my_map
        self.receiver_data = receiver_data
        self.index = 0
        self.len = 300

    def map_the_space(self):
        self.receiver_data.refresh_data()
        x, y, angle, distance_to_obstacle = self.receiver_data.get_data()
        if distance_to_obstacle  < 4.5:
           # obstacle_position = pymunk.vec2d.Vec2d(x,y) + distance*pymunk.vec2d.Vec2d(1,0).rotated(self.bot.body.angle)
            obstacle_position = [x + distance_to_obstacle *math.sin(angle), y + distance_to_obstacle *math.cos(angle)]
            obstacle_position = [int(obstacle_position[0]),int(obstacle_position[1])]
            if not(obstacle_position in self.map.obstacles_coordinates): 
                if(self.len > len(self.map.obstacles_coordinates)):
                    self.map.add_obstacle(obstacle_position[0],obstacle_position[1])
                elif(self.index < self.len):
                   self.map.obstacles_coordinates[self.index] = [obstacle_position[0],obstacle_position[1]]
                   self.index = self.index + 1
                else:
                    print("LLENO EL VECTOR DE OBSTACULOS")
                    self.index = 0
                    self.map.obstacles_coordinates[self.index] = [obstacle_position[0],obstacle_position[1]]
                    self.index = self.index + 1

def distance_between_points(p1, p2):
    """ Calcula la distancia euclidiana entre dos puntos p1 y p2. """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def calculate_angle(p1, p2, p3):
    """ Calcula el ángulo entre los segmentos (p1-p2) y (p2-p3) """
    vector_1 = (p2[0] - p1[0], p2[1] - p1[1])
    vector_2 = (p3[0] - p2[0], p3[1] - p2[1])
    dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]
    magnitude_1 = math.sqrt(vector_1[0]**2 + vector_1[1]**2)
    magnitude_2 = math.sqrt(vector_2[0]**2 + vector_2[1]**2)
    if magnitude_1 * magnitude_2 == 0:  # Para evitar división por 0
        return 0
    angle = math.acos(dot_product / (magnitude_1 * magnitude_2))
    return math.degrees(angle)

def new_path_too_similar(prev_path, new_path, distance_threshold=2, angle_threshold=15):
    """
    Compara dos caminos y determina si son demasiado similares.
    
    prev_path: lista de coordenadas [(x1, y1), (x2, y2), ...]
    new_path: lista de coordenadas [(x1, y1), (x2, y2), ...]
    distance_threshold: distancia mínima entre nodos para considerar diferentes los caminos
    angle_threshold: diferencia mínima de ángulo para considerar segmentos diferentes
    """
    # Si los caminos tienen una cantidad de nodos muy similar
    if len(prev_path) != len(new_path):
        return False  # Los caminos son diferentes si tienen diferentes cantidades de nodos

    # Revisar la distancia entre nodos correspondientes
    similar_nodes = 0
    for i in range(len(prev_path)):
        distance = distance_between_points(prev_path[i], new_path[i])
        if distance < distance_threshold:
            similar_nodes += 1

    # Si la mayoría de los nodos son demasiado cercanos, los caminos son similares
    if similar_nodes > 0.8 * len(prev_path):
        return True

    # Revisar los ángulos entre segmentos consecutivos
    similar_angles = 0
    for i in range(1, len(prev_path) - 1):
        prev_angle = calculate_angle(prev_path[i-1], prev_path[i], prev_path[i+1])
        new_angle = calculate_angle(new_path[i-1], new_path[i], new_path[i+1])
        if abs(prev_angle - new_angle) < angle_threshold:
            similar_angles += 1

    # Si la mayoría de los segmentos tienen ángulos similares, los caminos son similares
    if similar_angles > 0.8 * (len(prev_path) - 2):  # Restamos 2 porque no se pueden comparar extremos
        return True

    return False