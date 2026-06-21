import pygame
import pymunk
import pymunk.pygame_util
import math
import random
import numpy as np
from collections import deque

pymunk.pygame_util.positive_y_is_up = False
# Class GPS_Module
class GPS_Module:
    def __init__(self, bot, tile_size, uncertain_grids=0):
        self.bot = bot
        self.uncertain_grids = uncertain_grids  # Number of uncertainty grids
        self.x = self.bot.grid_position[0] +  np.random.normal(0, self.uncertain_grids)
        self.y = self.bot.grid_position[1] +  np.random.normal(0, self.uncertain_grids)
        self.tile_size = tile_size

    def update_coordinates(self):
        self.x = self.bot.grid_position[0] +  np.random.normal(0, self.uncertain_grids)
        self.y = self.bot.grid_position[1] +  np.random.normal(0, self.uncertain_grids)

    def get_coordinates(self):
        # Convert grid to pixels (Note: Logic seems to return grid coords here based on init, despite comment saying pixels?
        # In init: x = grid_position. Main loop uses x_bot from receiver.get_data.
        # receiver.refresh_data calls get_coordenates.
        # It seems it returns GRID coordinates with noise.
        return self.x , self.y

    def draw(self, screen,):
        font = pygame.font.Font(None, 36)
        gps_text = f"GPS_Bot: X={self.x:.2f}, Y={self.y:.2f}"
        gps_surface = font.render(gps_text, True, (255, 255, 255))
        screen.blit(gps_surface, (10, 10))

# Class IMU_Module
class IMU_Module:
    def __init__(self, bot):
        self.bot = bot
        self.angle = 0
        self.velocity = pymunk.Vec2d(0, 0)

    def update_orientation(self):
        # Get current bot angle
        self.angle = self.bot.body.angle
        # Get current bot velocity
        self.velocity = self.bot.body.velocity.rotated(-self.bot.body.angle)

        # Add uncertainty
        # angle_noise = random.uniform(-math.radians(5), math.radians(5))  # Noise of ±5 degrees
        # speed_noise = random.uniform(-10, 10)  # Noise in pixels per second

        # self.angle += angle_noise
        # self.velocity += pymunk.Vec2d(speed_noise, speed_noise).rotated(self.angle)

    def get_data(self):
        return self.angle, self.velocity.length

    def draw(self, screen,):
        font = pygame.font.Font(None, 36)
        angle_deg = math.degrees(self.angle) % 360
        velocity = self.velocity.length
        imu_text = f"IMU: Angle={angle_deg:.2f}°, Speed={velocity:.2f} px/s"
        imu_surface = font.render(imu_text, True, (255, 255, 255))
        screen.blit(imu_surface, (10, 50))


# Clase UltraSonicSensor
class UltraSonicSensor:
    def __init__(self, bot,tile_size, obstacles):
        self.bot = bot
        self.map = map
        self.tile_size = tile_size
        self.radius = 5 * tile_size  # Radio en grids
        self.angle_range = math.radians(25)  # Arco en grados
        self.obstacles = obstacles

    def display(self, screen):
        # Dibujar el arco de detección (Transparent)
        start_angle = self.bot.body.angle - self.angle_range / 2
        end_angle = self.bot.body.angle + self.angle_range / 2
        center = self.bot.body.position
        
        # Surface temporal para alpha
        sensor_surf = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
        
        for angle in range(int(math.degrees(start_angle)), int(math.degrees(end_angle)) + 1):
            radians = math.radians(angle)
            endpoint = (center[0] + self.radius * math.cos(radians),
                        center[1] + self.radius * math.sin(radians))
            # Color amarillo transparente (255, 255, 0, 50)
            pygame.draw.line(sensor_surf, (255, 255, 0, 30), center, endpoint)
            
        screen.blit(sensor_surf, (0,0))

    def display_distance(self):
        closest_distance = self.radius
        center = self.bot.body.position

        for obstacle in self.obstacles:
            obstacle_rect = pygame.Rect(obstacle.grid_x - self.tile_size // 2, obstacle.grid_y - self.tile_size // 2, self.tile_size, self.tile_size)
            # Chequear si algún punto del rectángulo del obstáculo está en el área del arco
            for corner in [(obstacle_rect.left, obstacle_rect.top),
                           (obstacle_rect.right, obstacle_rect.top),
                           (obstacle_rect.left, obstacle_rect.bottom),
                           (obstacle_rect.right, obstacle_rect.bottom)]:
                vector_to_corner = pymunk.Vec2d(corner[0],corner[1]) - center
                distance = vector_to_corner.length
                angle_to_corner = vector_to_corner.angle - self.bot.body.angle

                if distance < closest_distance and abs(angle_to_corner) <= self.angle_range / 2:
                    closest_distance = distance
        #print(closest_distance / tile_size)
        return closest_distance / self.tile_size  # Convertir a distancia en unidades de grid