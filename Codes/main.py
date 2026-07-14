import pygame
import pymunk
import pymunk.pygame_util
from collections import deque

import KalmanFilter as FK
import HeadingController as h_contol  
import RRT_Search as RRT
import SensorModules as sensors
import Lines
import Mapping as MP
import Obstacles
import Control
import WheelMotorDrivers as WMdrivers
import UARTReceiver as Receiver

import numpy as np
import math

# --------------------------------------------
# AESTHETICS / COLORS (Dark Sci-Fi Theme)
# --------------------------------------------
COLORS = {
    "background": (15, 15, 20),      # Dark blue-grey
    "sidebar_bg": (25, 25, 30),      # Slightly lighter
    "grid_lines": (40, 45, 50),      # Subtle grid
    "text": (220, 220, 220),         # Off-white
    "accent_1": (100, 130, 250),     # Cyan/Teal (Robot, paths)
    "accent_2": (255, 100, 50),      # Orange (Obstacles)
    "accent_3": (0, 200, 100),       # Green (Success/Ok)
    "alert": (180, 50, 50),          # Red 
    "minimap_border": (100, 100, 120),
    "metric_ruler": (100, 100, 100)
}

# --------------------------------------------
# CONFIGURACIÓN GENERAL
# --------------------------------------------
pygame.init()

SIDEBAR_WIDTH = 300
MAP_WIDTH  = 800
WINDOW_WIDTH = SIDEBAR_WIDTH + MAP_WIDTH
WINDOW_HEIGHT = 600

screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Simulation")

clock = pygame.time.Clock()
space = pymunk.Space()
font = pygame.font.Font(None, 24)
tile_size = 10
pymunk.pygame_util.positive_y_is_up = False

# --------------------------------------------
# FUNCIONES
# --------------------------------------------
def draw_grid(target_surface):
    for x in range(0, MAP_WIDTH, tile_size):
        pygame.draw.line(target_surface, COLORS["grid_lines"], (x, 0), (x, WINDOW_HEIGHT))
    for y in range(0, WINDOW_HEIGHT, tile_size):
        pygame.draw.line(target_surface, COLORS["grid_lines"], (0, y), (MAP_WIDTH, y))

def draw_metric_rulers(surface):
    step_grids = 10
    step_px = step_grids * tile_size

    for gx in range(0, MAP_WIDTH+1, step_px):
        pygame.draw.line(surface, COLORS["metric_ruler"], (gx, 0), (gx, 8))
        label = font.render(str(gx // tile_size), True, COLORS["metric_ruler"])
        surface.blit(label, (gx + 2, 10))

    for gy in range(0, WINDOW_HEIGHT+1, step_px):
        pygame.draw.line(surface, COLORS["metric_ruler"], (0, gy), (8, gy))
        label = font.render(str(gy // tile_size), True, COLORS["metric_ruler"])
        surface.blit(label, (10, gy + 2))

# --------------------------------------------
# BOT
# --------------------------------------------
class Bot:
    def __init__(self, x, y):
        self.grid_position = (x, y)
        self.size = tile_size * 0.5

        self.body = pymunk.Body(
            1,
            pymunk.moment_for_box(1, (self.size, self.size)),
            0
        )

        self.start_px = (x * tile_size + tile_size // 2,
                         y * tile_size + tile_size // 2)
        self.body.position = self.start_px

        self.shape = pymunk.Poly.create_box(self.body, (self.size, self.size))
        self.shape.elasticity = 0.7
        self.shape.friction = 100
        space.add(self.body, self.shape)

        self.left_wheel_speed = 10
        self.right_wheel_speed = 10

    def update_movement(self):
        velocity = (self.left_wheel_speed + self.right_wheel_speed) / 2
        angular_velocity = (self.right_wheel_speed - self.left_wheel_speed) / 20

        self.body.velocity = pymunk.Vec2d(velocity, 0).rotated(self.body.angle)
        self.body.angular_velocity = angular_velocity

        new_x = int(self.body.position.x // tile_size)
        new_y = int(self.body.position.y // tile_size)
        self.grid_position = (new_x, new_y)

    def draw(self, surf):
        # --- Configuración visual ---
        w = self.size
        h = self.size
        wheel_w = w * 0.3
        wheel_h = h * 0.8
        offset_wheel = w * 0.4
        
        # Posición y ángulo
        x, y = self.body.position
        angle = self.body.angle
        
        # Función helper para rotar puntos
        def rotate_point(px, py, cx, cy, th):
            dx = px - cx
            dy = py - cy
            rx = dx * math.cos(th) - dy * math.sin(th)
            ry = dx * math.sin(th) + dy * math.cos(th)
            return int(cx + rx), int(cy + ry)

        # 1. DIBUJAR RUEDAS (Izquierda / Derecha)
        # Definir rectangulos locales (sin rotar)
        wb_l = pygame.Rect(0, 0, wheel_h, wheel_w)
        wb_r = pygame.Rect(0, 0, wheel_h, wheel_w)
        
        # Centros locales de ruedas
        # Rueda izquierda (arriba en eje Y local)
        # Rueda derecha (abajo en eje Y local)
        # Asumiendo que el robot mira hacia +X
        
        # Vamos a dibujar poligonos rotados para las ruedas
        # Wheel offsets relative to center (0,0)
        # Front-Right, Back-Right, Back-Left, Front-Left
        
        def get_rect_corners(cx, cy, width, height):
            return [
                (cx + width/2, cy - height/2),
                (cx + width/2, cy + height/2),
                (cx - width/2, cy + height/2),
                (cx - width/2, cy - height/2)
            ]

        # Ruedas desplazadas del centro
        # Rueda Izq: offset en -Y
        l_wheel_corners = get_rect_corners(0, -offset_wheel, wheel_h, wheel_w)
        # Rueda Der: offset en +Y
        r_wheel_corners = get_rect_corners(0, offset_wheel, wheel_h, wheel_w)
        
        # Transformar y dibujar ruedas
        for corners in [l_wheel_corners, r_wheel_corners]:
            poly_points = []
            for (cx, cy) in corners:
                poly_points.append(rotate_point(x + cx, y + cy, x, y, angle))
            pygame.draw.polygon(surf, (20, 20, 20), poly_points) # Negro goma
            pygame.draw.lines(surf, (60,60,60), True, poly_points, 1)

        # 2. DIBUJAR CHASIS (Cuerpo principal)
        vertices = self.shape.get_vertices()
        chassis_points = [
            (int(self.body.position.x + v.rotated(self.body.angle).x),
             int(self.body.position.y + v.rotated(self.body.angle).y))
            for v in vertices
        ]
        pygame.draw.polygon(surf, COLORS["accent_1"], chassis_points)
        pygame.draw.lines(surf, COLORS["text"], True, chassis_points, 2)
        
        # 3. DETALLES (Luces y Dirección)
        # Headlights (círculos amarillos en la parte frontal)
        # Front is +X axis
        light_offset_x = w * 0.4
        light_offset_y = w * 0.25
        
        l_light = rotate_point(x + light_offset_x, y - light_offset_y, x, y, angle)
        r_light = rotate_point(x + light_offset_x, y + light_offset_y, x, y, angle)
        
        pygame.draw.circle(surf, (255, 255, 100), l_light, 3) 
        pygame.draw.circle(surf, (255, 255, 100), r_light, 3)

        # Indicador de dirección (Triangulo en el techo)
        tip = rotate_point(x + w * 0.3, y, x, y, angle)
        back_l = rotate_point(x - w * 0.2, y - w * 0.2, x, y, angle)
        back_r = rotate_point(x - w * 0.2, y + w * 0.2, x, y, angle)
        
        pygame.draw.polygon(surf, (0, 100, 100), [tip, back_l, back_r])

# --------------------------------------------
# SYSTEM INITIALIZATION
# --------------------------------------------
bot = Bot(4, 24)
obstacles = Obstacles.hallway_obstacles(tile_size)

gps_module = sensors.GPS_Module(bot, tile_size)
imu_module = sensors.IMU_Module(bot)
ultrasonic_sensor = sensors.UltraSonicSensor(bot, tile_size, obstacles)

heading_controller = h_contol.HeadingController()
driver = WMdrivers.WheelMotorDrivers(bot, 0, 0)
receiver = Receiver.UARTReceiver(1, 2, gps_module, imu_module, ultrasonic_sensor)

my_map = MP.Map()
mapping = MP.Mapping(my_map, receiver)

kalman = FK.KalmanFilter(
    dt=1/60.0,
    process_variance=1e-4,
    measurement_variance_gps=1e-2,
    measurement_variance_imu=1e-2
)

destination = (61, 37)

controller = Control.Controller(
    destination,
    tile_size,
    MAP_WIDTH,
    WINDOW_HEIGHT,
    my_map,
    heading_controller,
    kalman,
    5,
    driver,
    receiver
)


# --------------------------------------------
# WORLD SURFACE
# --------------------------------------------
world = pygame.Surface((MAP_WIDTH, WINDOW_HEIGHT))
world.fill(COLORS["background"])

line_gps       = Lines.Line(tile_size, world, max_points=10, line_color=COLORS["accent_2"], line_width=2)
line_estimated = Lines.Line(tile_size, world, max_points=10, line_color=(0,0,255), line_width=2)
line_tree      = Lines.TreeLine(controller.current_path, world, tile_size)

# ============================================
# MAIN LOOP
# ============================================

# UI VARIABLES
# ============================================
simulation_paused = False

# Button Rects
# Centered in sidebar
btn_pause = pygame.Rect(50, 450, 200, 50)

# ============================================
# LOOP PRINCIPAL
# ============================================
running = True
while running:

    dt = clock.tick(120) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        # Mouse Click Events for UI
        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            
            # 1. PAUSE / RESUME
            if btn_pause.collidepoint(mx, my):
                simulation_paused = not simulation_paused
                print(f"Simulation Paused: {simulation_paused}")

    if not simulation_paused:
        bot.update_movement()
        gps_module.update_coordinates()
        imu_module.update_orientation()
        controller.update()

        x_bot, y_bot, angle, d_obs = controller.receiver_data.get_data()

        line_gps.update([x_bot, y_bot])
        line_estimated.update(controller.get_state_data())
        line_tree = Lines.TreeLine(controller.current_path, world, tile_size)

        mapping.map_the_space()
        space.step(1/60.0)
    else:
        # Keep updating sensor data variables to avoid crash in stats display if accessed
        # But do not run physics or control lopp
        if 'x_bot' not in locals():
            x_bot, y_bot, angle, d_obs = controller.receiver_data.get_data()


    screen.fill(COLORS["sidebar_bg"])

    sidebar = pygame.Surface((SIDEBAR_WIDTH, WINDOW_HEIGHT))
    sidebar.fill(COLORS["sidebar_bg"])

    # ==========================================================
    # NUEVA SECCIÓN → MINIMAPA
    # ==========================================================
    
    # Configuración del minimapa
    minimap_scale = 0.35
    minimap_w = int(MAP_WIDTH * minimap_scale)
    minimap_h = int(WINDOW_HEIGHT * minimap_scale)
    
    minimap_surface = pygame.Surface((minimap_w, minimap_h))
    minimap_surface.fill(COLORS["background"]) # Fondo negro para el mapa
    
    # Dibujar borde
    pygame.draw.rect(minimap_surface, COLORS["minimap_border"], (0, 0, minimap_w, minimap_h), 1)

    # 1. Dibujar obstáculos detectados
    # my_map.obstacles_coordinates almacena coordenadas en GRIDS, no pixels.
    # Convertir: Grid -> Pixel -> Minimap
    for obs in my_map.obstacles_coordinates:
        mx = int(obs[0] * tile_size * minimap_scale)
        my = int(obs[1] * tile_size * minimap_scale)
        # Dibujar un pequeño punto/rectángulo rojo
        pygame.draw.rect(minimap_surface, COLORS["accent_2"], (mx, my, 2, 2))

    # 2. Dibujar robot
    # Posición actual del robot
    rx = int(bot.body.position.x * minimap_scale)
    ry = int(bot.body.position.y * minimap_scale)
    pygame.draw.circle(minimap_surface, COLORS["accent_1"], (rx, ry), 3)
    
    # 3. Dibujar meta (opcional, útil referencia)
    dx = int((destination[0] * tile_size + tile_size//2) * minimap_scale)
    dy = int((destination[1] * tile_size + tile_size//2) * minimap_scale)
    pygame.draw.circle(minimap_surface, (255, 255, 0), (dx, dy), 3)

    # Blitear minimapa en la sidebar
    sidebar.blit(minimap_surface, (10, 10))
    
    # ==========================================================

    stats_y = 250
    
    # Helper para dibujar texto con sombra
    def draw_text(surf, text, x, y, color=COLORS["text"], font=font):
        shadow = font.render(text, True, (0,0,0))
        surf.blit(shadow, (x+1, y+1))
        label = font.render(text, True, color)
        surf.blit(label, (x, y))

    # Título de estadísticas
    draw_text(sidebar, "--- STATES ---", 10, stats_y, COLORS["accent_1"])
    stats_y += 30

    stats = [
        f"Robot X: {x_bot:.1f}",
        f"Robot Y: {y_bot:.1f}",
        f"Grid: {bot.grid_position}",
        f"Vel L: {bot.left_wheel_speed:.1f}",
        f"Vel R: {bot.right_wheel_speed:.1f}",
        f"Dist Obs: {d_obs:.2f}",
        f"Mode: {('RRT' if not controller.generate_path_flag else 'PLANNING')}"
    ]

    for s in stats:
        draw_text(sidebar, s, 10, stats_y)
        stats_y += 24

    # ==========================================================
    # UI CONTROLS DRAWING
    # ==========================================================
    
    # DRAW PAUSE BUTTON
    # Colors suitable for Sci-Fi theme
    if simulation_paused:
        # Resume state
        btn_color = COLORS["accent_1"] # Cyan
        btn_text_str = "RESUME SYSTEM"
        btn_text_color = (0, 0, 0)
    else:
        # Pause state
        btn_color = COLORS["alert"]    # Red
        btn_text_str = "PAUSE SIMULATION"
        btn_text_color = (255, 255, 255)

    pygame.draw.rect(sidebar, btn_color, btn_pause, border_radius=8)
    pygame.draw.rect(sidebar, (255, 255, 255), btn_pause, 2, border_radius=8)
    
    # Center text
    text_surf = font.render(btn_text_str, True, btn_text_color)
    text_rect = text_surf.get_rect(center=btn_pause.center)
    sidebar.blit(text_surf, text_rect)


    screen.blit(sidebar, (0,0))

    world.fill(COLORS["background"])
    draw_grid(world)
    draw_metric_rulers(world)

    pygame.draw.circle(world, COLORS["accent_3"], (3, 3), 4)

    bx0, by0 = bot.start_px
    pygame.draw.circle(world, COLORS["accent_1"], (int(bx0), int(by0)), 5)

    for ob in obstacles:
        # Obstacle Rect
        rect_obj = pygame.Rect(ob.grid_x, ob.grid_y, tile_size/2, tile_size/2)
        
        # Fill
        pygame.draw.rect(world, COLORS["accent_2"], rect_obj)
        
        # Tech Border
        pygame.draw.rect(world, (255, 150, 100), rect_obj, 1)
        
        # "X" detail inside
        pygame.draw.line(world, (100, 50, 0), rect_obj.topleft, rect_obj.bottomright, 1)
        pygame.draw.line(world, (100, 50, 0), rect_obj.bottomleft, rect_obj.topright, 1)

    dx = destination[0] * tile_size + tile_size//2
    dy = destination[1] * tile_size + tile_size//2
    pygame.draw.rect(
        world, COLORS["accent_3"],
        (dx - tile_size//2, dy - tile_size//2, tile_size, tile_size)
    )

    bot.draw(world)
    #gps_module.draw(world)
    #imu_module.draw(world)
    ultrasonic_sensor.display(world)

    line_gps.draw()
    line_estimated.draw()
    line_tree.draw()

    screen.blit(world, (SIDEBAR_WIDTH, 0))

    pygame.display.flip()

pygame.quit()

