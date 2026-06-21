import pygame
import pymunk
import pymunk.pygame_util
import math
import numpy as np
from collections import deque

space = pymunk.Space()
# Obstacles
class Obstacle:
    def __init__(self, x, y, tile_size):
        self.body = pymunk.Body(body_type=pymunk.Body.STATIC)
        # Convert grid coordinates to position coordinates
        self.grid_x = x * tile_size + tile_size // 2
        self.grid_y = y * tile_size + tile_size // 2
        self.body.position = (self.grid_x, self.grid_y)
        self.shape = pymunk.Poly.create_box(self.body, (tile_size, tile_size))
        self.shape.elasticity = 0.7
        self.shape.friction = 1.0  # Friction to avoid unwanted sliding
        space.add(self.body, self.shape)


def obstacles1(tile_size):
    obstacles = []
    for i in range(14):  
        obstacles.append(Obstacle(10, i, tile_size))
        obstacles.append(Obstacle(10, i + 0.5, tile_size))

    for i in range(10):  
        obstacles.append(Obstacle(i, 0, tile_size))
        obstacles.append(Obstacle(i + 0.5, 0, tile_size))

    for i in range(15):  
        obstacles.append(Obstacle(1, i, tile_size))
        obstacles.append(Obstacle(1, i+0.5, tile_size))

    for i in range(50):  
        obstacles.append(Obstacle(i+8, 14, tile_size))
        obstacles.append(Obstacle(i+0.5+8, 14, tile_size))
    for i in range(55):  
        obstacles.append(Obstacle(i, 20, tile_size))
        obstacles.append(Obstacle(i+0.5, 20, tile_size))

    for i in range(20):  
        obstacles.append(Obstacle(55, 20+i, tile_size))
        obstacles.append(Obstacle(55, 20+i+0.5, tile_size))

    for i in range(25):  
        obstacles.append(Obstacle(64, 15+i, tile_size))
        obstacles.append(Obstacle(64, 15+i+0.5, tile_size))
    
    return obstacles

def obstacles2(tile_size):
    obstacles = []
    return obstacles

def obstacles3(tile_size):
    obstacles = []

    for i in range(50):  
        obstacles.append(Obstacle(i+8, 14, tile_size))
        obstacles.append(Obstacle(i+0.5+8, 14, tile_size))

    for i in range(20):  
        obstacles.append(Obstacle(55, 20+i, tile_size))
        obstacles.append(Obstacle(55, 20+i+0.5, tile_size))

    return obstacles

def hallway_obstacles(tile_size):
    obstacles = []
    width_grids = 80
    height_grids = 60

    # Horizontal Walls (Defining Hallways and Rooms)
    for x in range(width_grids):
        # Top wall (y=15) with gaps
        if not (10 <= x <= 15) and not (60 <= x <= 65):
            obstacles.append(Obstacle(x, 15, tile_size))
            obstacles.append(Obstacle(x, 15 + 0.5, tile_size)) # Double thickness

        # Bottom wall (y=45) with gaps
        if not (20 <= x <= 25) and not (50 <= x <= 55) and not (70 <= x <= 75):
            obstacles.append(Obstacle(x, 45, tile_size))
            obstacles.append(Obstacle(x, 45 + 0.5, tile_size))

    # Vertical Walls (Separating sections)
    for y in range(height_grids):
        # Left vertical wall (x=25)
        # Partially leave central hallway free (15 < y < 45)
        if (y < 15) or (y > 45) or (25 < y < 35):
            obstacles.append(Obstacle(25, y, tile_size))
            obstacles.append(Obstacle(25 + 0.5, y, tile_size))

        # Right vertical wall (x=55)
        if (y < 10) or (y > 50) or (20 < y < 40):
            obstacles.append(Obstacle(55, y, tile_size))
            obstacles.append(Obstacle(55 + 0.5, y, tile_size))
            
    return obstacles