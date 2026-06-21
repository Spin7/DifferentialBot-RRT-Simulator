import pygame
import pymunk
import pymunk.pygame_util
import math
import numpy as np
from collections import deque


class Line:
    def __init__(self, tile_size ,screen, max_points=3000, line_color=(255, 0, 0), line_width=2):
        """
        Initializes the Line class.

        :param screen: Pygame surface where the line will be drawn.
        :param max_points: Maximum number of points to store for drawing the line.
        :param line_color: Color of the line.
        :param line_width: Width of the line.
        """
        self.screen = screen
        self.max_points = max_points
        self.line_color = line_color
        self.line_width = line_width
        self.points = deque(maxlen=max_points)
        self.tile_size = tile_size

    def update(self, position):
        """
        Adds a new position to the list of points.

        :param position: Tuple (x, y) in pixels.
        """
        position = (position[0] * self.tile_size + self.tile_size // 2, position[1] * self.tile_size + self.tile_size // 2)
        self.points.append(position)

    def draw(self):
        """
        Draws the line connecting the stored points.
        """
        if len(self.points) < 2:
            return
        
        # Crear surface temporal para transparencia (Trails)
        trail_surf = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        
        # Añadir alpha al color si no lo tiene
        if len(self.line_color) == 3:
            col_alpha = (self.line_color[0], self.line_color[1], self.line_color[2], 100)
        else:
            col_alpha = self.line_color
            
        pygame.draw.lines(trail_surf, col_alpha, False, self.points, self.line_width)
        self.screen.blit(trail_surf, (0,0))


import pygame

class TreeLine:
    def __init__(self, path, screen, tile_size, point_color=(0, 200, 255), line_color=(0, 240, 255), point_radius=5, line_width=2):
        """
        path: List of nodes [(x1, y1), (x2, y2)] in grid coordinates
        screen: The Pygame surface where the tree will be drawn
        tile_size: Size of the tile in pixels
        point_color: Color of the points (default red)
        line_color: Color of the lines (default green)
        point_radius: Radius of the points in pixels
        line_width: Width of the lines in pixels
        """
        self.path = path
        self.screen = screen
        self.tile_size = tile_size  # Size of the tile in pixels
        self.point_color = point_color
        self.line_color = line_color
        self.point_radius = point_radius
        self.line_width = line_width

    def draw(self):
        """Draws the nodes as thick points and connects the nodes with thin lines."""
        if not self.path or len(self.path) < 2:
            return

        # Crear surface temporal para transparencia
        path_surf = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        
        # Colores con Alpha
        # Cyan transparente para lineas, un poco más opaco para puntos
        line_col_alpha = (self.line_color[0], self.line_color[1], self.line_color[2], 100)
        point_col_alpha = (self.point_color[0], self.point_color[1], self.point_color[2], 150)

        # Ajustar grosor y radio para ser más sutiles
        draw_radius = 3
        draw_width = 2
        
        for i in range(len(self.path) - 1):
            # Convertir las coordenadas del grid a píxeles
            node_pixel = (self.path[i][0] * self.tile_size, self.path[i][1] * self.tile_size)
            next_node_pixel = (self.path[i+1][0] * self.tile_size, self.path[i+1][1] * self.tile_size)
            
            # Conecta los nodos con líneas finas en píxeles
            pygame.draw.line(path_surf, line_col_alpha, node_pixel, next_node_pixel, draw_width)

            # Dibuja los nodos como círculos
            pygame.draw.circle(path_surf, point_col_alpha, node_pixel, draw_radius)
            
        # Dibujar el último punto
        last_node = (self.path[-1][0] * self.tile_size, self.path[-1][1] * self.tile_size)
        pygame.draw.circle(path_surf, point_col_alpha, last_node, draw_radius)

        self.screen.blit(path_surf, (0,0))