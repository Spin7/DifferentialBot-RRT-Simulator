import pygame
import pymunk
import pymunk.pygame_util
import math
import random
import numpy as np
from collections import deque

# Configuración inicial
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Seguimiento de GPS e IMU con Filtro de Kalman")
clock = pygame.time.Clock()
space = pymunk.Space()
font = pygame.font.Font(None, 36)
tile_size = 10  # Tamaño de la cuadrícula
pymunk.pygame_util.positive_y_is_up = False

# Dibujar la cuadrícula
def draw_grid():
    for x in range(0, width, tile_size):
        pygame.draw.line(screen, (50, 50, 50), (x, 0), (x, height))
    for y in range(0, height, tile_size):
        pygame.draw.line(screen, (50, 50, 50), (0, y), (width, y))

# Clase Bot con ruedas independientes
class Bot:
    def __init__(self, x, y):
        self.grid_position = (x, y)  # Posición en coordenadas de grid
        self.body = pymunk.Body(1, pymunk.moment_for_box(1, (tile_size, tile_size)), 0)
        self.body.position = (x * tile_size + tile_size // 2, y * tile_size + tile_size // 2)
        self.shape = pymunk.Poly.create_box(self.body, (tile_size, tile_size))
        self.shape.elasticity = 0.7
        self.shape.friction = 100
        space.add(self.body, self.shape)
        self.rueda_izquierda = 0  # Velocidad inicial de la rueda izquierda
        self.rueda_derecha = 0    # Velocidad inicial de la rueda derecha

    def update_movement(self):
        velocity = (self.rueda_izquierda + self.rueda_derecha) / 2
        angular_velocity = (self.rueda_derecha - self.rueda_izquierda) / 20  # Factor para ajustar la rotación

        # Actualizar la velocidad y la velocidad angular del cuerpo
        self.body.velocity = pymunk.Vec2d(velocity, 0).rotated(self.body.angle)
        self.body.angular_velocity = angular_velocity

        # Actualizar la posición en el grid
        new_x = int(self.body.position.x // tile_size)
        new_y = int(self.body.position.y // tile_size)
        self.grid_position = (new_x, new_y)

    def draw(self, screen):
        vertices = self.shape.get_vertices()
        points = [(int(self.body.position.x + v.rotated(self.body.angle).x),
                   int(self.body.position.y + v.rotated(self.body.angle).y)) for v in vertices]
        pygame.draw.polygon(screen, (0, 255, 0), points)

# Clase GPS_Module
class GPS_Module:
    def __init__(self, bot):
        self.bot = bot
        self.uncertain_grids = 8  # Número de grids de incertidumbre
        self.x = self.bot.grid_position[0] +  np.random.normal(0, self.uncertain_grids)
        self.y = self.bot.grid_position[1] +  np.random.normal(0, self.uncertain_grids)

    def update_coordenates(self):
        self.x = self.bot.grid_position[0] +  np.random.normal(0, self.uncertain_grids)
        self.y = self.bot.grid_position[1] +  np.random.normal(0, self.uncertain_grids)

    def get_position(self):
        # Convertir grid a píxeles
        return (self.x * tile_size + tile_size // 2, self.y * tile_size + tile_size // 2)

    def draw(self, screen):
        
        gps_text = f"GPS_Bot: X={self.x:.2f}, Y={self.y:.2f}"
        gps_surface = font.render(gps_text, True, (255, 255, 255))
        screen.blit(gps_surface, (10, 10))

# Clase IMU_Module
class IMU_Module:
    def __init__(self, bot):
        self.bot = bot
        self.angle = 0
        self.velocity = pymunk.Vec2d(0, 0)

    def update_orientation(self):
        # Obtener el ángulo actual del bot
        self.angle = self.bot.body.angle
        # Obtener la velocidad actual del bot
        self.velocity = self.bot.body.velocity.rotated(-self.bot.body.angle)

        # Agregar incertidumbre
        angle_noise = random.uniform(-math.radians(5), math.radians(5))  # Ruido de ±5 grados
        speed_noise = random.uniform(-10, 10)  # Ruido en píxeles por segundo

        self.angle += angle_noise
        self.velocity += pymunk.Vec2d(speed_noise, speed_noise).rotated(self.angle)

    def get_data(self):
        return self.angle, self.velocity.length

    def draw(self, screen):
        angle_deg = math.degrees(self.angle) % 360
        velocity = self.velocity.length
        imu_text = f"IMU: Angle={angle_deg:.2f}°, Speed={velocity:.2f} px/s"
        imu_surface = font.render(imu_text, True, (255, 255, 255))
        screen.blit(imu_surface, (10, 50))

# Clase Controlador
class Controlador:
    def __init__(self, bot):
        self.bot = bot

    def controlar_mov(self, vr, vl):
        self.bot.rueda_derecha = vr
        self.bot.rueda_izquierda = vl

# Clase KalmanFilter para estimar la posición
class KalmanFilter:
    def __init__(self, dt, 
                 process_variance=1e-4, 
                 measurement_variance_gps=1e-2, 
                 measurement_variance_imu=1e-2):
        """
        Inicializa el Filtro de Kalman.
        
        :param dt: Intervalo de tiempo entre actualizaciones.
        :param process_variance: Varianza del proceso.
        :param measurement_variance_gps: Varianza de la medición del GPS.
        :param measurement_variance_imu: Varianza de la medición de la IMU.
        """
        # Estado inicial [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        
        # Matriz de covarianza del estado
        self.P = np.eye(4) * 1.0
        
        # Matriz de transición de estado (F)
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0 ],
                           [0, 0, 0, 1 ]])
        
        # Matriz de control (B) - No se usa en este caso
        self.B = np.zeros((4, 2))
        
        # Matriz de observación (H)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        
        # Matriz de covarianza de medición (R)
        self.R = np.diag([measurement_variance_gps, 
                          measurement_variance_gps, 
                          measurement_variance_imu, 
                          measurement_variance_imu])
        
        # Matriz de covarianza del proceso (Q)
        self.Q = np.eye(4) * process_variance

    def predict(self):
        """
        Predice el siguiente estado y la covarianza.
        """
        # Predicción del estado
        self.x = np.dot(self.F, self.x)
        
        # Predicción de la covarianza
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        """
        Actualiza el estado con una nueva medición.
        
        :param z: Vector de medición [x_gps, y_gps, vx_imu, vy_imu]
        """
        z = np.array(z).reshape((4, 1))
        
        # Residuo de la medición
        y = z - np.dot(self.H, self.x)
        
        # S = HPH^T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        
        # Ganancia de Kalman
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # Actualización del estado
        self.x = self.x + np.dot(K, y)
        
        # Actualización de la covarianza
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

    def get_estimated_position(self):
        """
        Devuelve la posición estimada.
        """
        return (self.x[0, 0], self.x[1, 0])

# Clase Line para dibujar la trayectoria
class Line:
    def __init__(self, screen, max_points=10, line_color=(255, 0, 0), line_width=2):
        """
        Inicializa la clase Line.

        :param screen: Superficie de pygame donde se dibujará la línea.
        :param max_points: Número máximo de puntos a almacenar para dibujar la línea.
        :param line_color: Color de la línea.
        :param line_width: Ancho de la línea.
        """
        self.screen = screen
        self.max_points = max_points
        self.line_color = line_color
        self.line_width = line_width
        self.points = deque(maxlen=max_points)

    def update(self, position):
        """
        Agrega una nueva posición a la lista de puntos.

        :param position: Tuple (x, y) en píxeles.
        """
        self.points.append(position)

    def draw(self):
        """
        Dibuja la línea conectando los puntos almacenados.
        """
        if len(self.points) > 1:
            pygame.draw.lines(self.screen, self.line_color, False, list(self.points), self.line_width)

# Inicialización de objetos
chuck = Bot(5,5)
control = Controlador(chuck)
GPS = GPS_Module(chuck)
IMU = IMU_Module(chuck)
line_gps = Line(screen, max_points=10, line_color=(255, 0, 0), line_width=2)       # Línea de GPS (Rojo)
line_estimated = Line(screen, max_points=10, line_color=(0, 0, 255), line_width=2)  # Línea estimada (Azul)

# Inicializar el Filtro de Kalman
dt = 1/60.0  # Intervalo de tiempo (60 FPS)
kalman = KalmanFilter(dt=dt,
                      process_variance=1e-4, 
                      measurement_variance_gps=1e-2, 
                      measurement_variance_imu=1e-2)

# Loop principal del juego
running = True
while running:
    dt = clock.tick(60) / 1000.0  # Delta time en segundos (60 FPS)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Manejo de teclas
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_a:
                control.controlar_mov(-100, 100)
            if event.key == pygame.K_d:
                control.controlar_mov(100, -100)
            if event.key == pygame.K_w:
                control.controlar_mov(100, 100)
            if event.key == pygame.K_s:
                control.controlar_mov(-100, -100)

    # Actualizar movimientos y sensores
    chuck.update_movement()
    GPS.update_coordenates()
    IMU.update_orientation()

    # Obtener datos de GPS e IMU
    gps_pos = GPS.get_position()
    angle, speed = IMU.get_data()

    # Convertir velocidad y ángulo a componentes vx e vy
    vx_imu = speed * math.cos(angle)
    vy_imu = speed * math.sin(angle)

    # Actualizar el Filtro de Kalman con las mediciones
    kalman.predict()
    kalman.update([gps_pos[0], gps_pos[1], vx_imu, vy_imu])
    estimated_pos = kalman.get_estimated_position()

    # Actualizar las líneas con las nuevas posiciones
    line_gps.update(gps_pos)
    line_estimated.update(estimated_pos)

    # Actualizar la física de pymunk
    space.step(dt)

    # Limpiar la pantalla
    screen.fill((0, 0, 0))
    draw_grid()

    # Dibujar elementos
    chuck.draw(screen)
    GPS.draw(screen)
    IMU.draw(screen)
    line_gps.draw()        # Dibujar línea de GPS en rojo
    line_estimated.draw()  # Dibujar línea estimada en azul

    # Mostrar la posición estimada
    estimated_text = f"Estimación Kalman: X={estimated_pos[0]:.2f}, Y={estimated_pos[1]:.2f}"
    estimated_surface = font.render(estimated_text, True, (0, 255, 255))
    screen.blit(estimated_surface, (10, 90))

    # Actualizar la pantalla
    pygame.display.flip()

pygame.quit()
