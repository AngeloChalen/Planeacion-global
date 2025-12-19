import os
import cv2
import csv
import yaml
import numpy as np
import sys
import math
import random
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid, SearchFactory
from pathlib import Path

# ==========================================
# 1. CLASE RRT PARA GRILLA (NUEVO)
# ==========================================
class RRT_Grid:
    def __init__(self, start, goal, env, step_size=3.0, max_iter=2000, goal_sample_rate=0.10):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.env = env
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.node_list = [self.start]

    def plan(self):
        print(f"   -> RRT buscando camino (Max iter: {self.max_iter})...")
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.step_size)

            if self.check_collision(new_node, self.env):
                self.node_list.append(new_node)
                
                # Chequear si llegamos cerca de la meta
                dx = new_node.x - self.goal.x
                dy = new_node.y - self.goal.y
                d = math.hypot(dx, dy)

                if d <= self.step_size:
                    print(f"   -> ¡Meta RRT encontrada en iteración {i}!")
                    final_node = self.steer(new_node, self.goal, self.step_size)
                    if self.check_collision(final_node, self.env):
                        return self.generate_final_course(len(self.node_list) - 1)
        
        print("   -> ❌ RRT no encontró camino (Intenta subir max_iter o step_size)")
        return None

    def steer(self, from_node, to_node, extend_length=float('inf')):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_dist_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length // self.step_size) # floor division
        
        for _ in range(n_expand):
            new_node.x += self.step_size * math.cos(theta)
            new_node.y += self.step_size * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_dist_angle(new_node, to_node)
        if d <= self.step_size:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

    def generate_final_course(self, goal_ind):
        path = [(self.goal.x, self.goal.y)]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.append((self.start.x, self.start.y))
        return path # Retorna lista [(x,y), ...]

    def calc_dist_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def get_random_node(self):
        if random.random() > self.goal_sample_rate:
            rnd = Node(random.uniform(0, self.env.width), 
                       random.uniform(0, self.env.height))
        else:
            rnd = Node(self.goal.x, self.goal.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        min_ind = dlist.index(min(dlist))
        return min_ind

    @staticmethod
    def check_collision(node, env):
        # Verifica si el punto o su path cruzan un obstáculo
        # Verificación simple punto final + puntos intermedios
        for x, y in zip(node.path_x, node.path_y):
            ix, iy = int(round(x)), int(round(y))
            if ix < 0 or ix >= env.width or iy < 0 or iy >= env.height:
                return False
            # env.obstacles es un set de tuplas (x, y)
            if (ix, iy) in env.obstacles:
                return False
        return True

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None

# ==========================================
# 2. FUNCIONES DE UTILIDAD (MODIFICADAS)
# ==========================================
def resample_path_by_distance(path_grid, spacing_meters, resolution, origin, image_height):
    """
    Toma el camino crudo (grid), lo pasa a metros e interpola a distancia exacta.
    """
    if not path_grid:
        return []

    # 1. Convertir a Metros
    path_world = []
    # Dependiendo del algoritmo, el path viene invertido o no. 
    # LPA suele venir Start->Goal, RRT Goal->Start.
    # Aquí asumimos que lo ordenaremos fuera o lo tratamos como conjunto de puntos.
    
    for p in path_grid:
        # p puede ser tupla (x, y) o objeto. Asumimos tupla.
        if hasattr(p, 'x'): px, py = p.x, p.y # Si es nodo objeto
        else: px, py = p[0], p[1] # Si es tupla
        
        x_w, y_w = map_to_world(px, py, resolution, origin, image_height)
        path_world.append(np.array([x_w, y_w]))

    # Asegurar orden Start -> Goal (Si el primer punto está lejos del Start, invertir)
    # (Opcional: aquí asumimos que el usuario invierte antes de llamar si es necesario)
    
    # 2. Interpolación
    new_path = [path_world[0]]
    for i in range(len(path_world) - 1):
        p1 = path_world[i]
        p2 = path_world[i+1]
        dist = np.linalg.norm(p2 - p1)
        
        if dist > spacing_meters:
            num_steps = int(np.ceil(dist / spacing_meters))
            vector = p2 - p1
            for k in range(1, num_steps + 1):
                new_pt = p1 + vector * (k / num_steps)
                new_path.append(new_pt)
        else:
            new_path.append(p2)
            
    return new_path

def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path) 
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)

    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Binarizar: 1 = ocupado, 0 = libre
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    # Engrosar obstáculos (Importante para que RRT no roce paredes)
    # RRT necesita más margen que A* porque corta esquinas
    kernel_size = 3
    if downsample_factor >= 4: kernel_size = 5
    map_bin = cv2.dilate(map_bin, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)

    # Downsampling
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    map_bin = cv2.resize(map_bin, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

    map_bin = (map_bin > 0.5).astype(np.uint8)
    resolution *= downsample_factor

    return map_bin, resolution, origin

def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(w, h)
    obstacles = {(x, h - 1 - y) for y in range(h) for x in range(w) if map_bin[y, x] == 1}
    env.update(obstacles)
    return env

def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)

def map_to_world(x_map, y_map, resolution, origin, image_height):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)

def save_final_csv(path_in_meters, filename):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        # Aquí ya vienen en metros, no convertimos nada
        for p in path_in_meters:
            writer.writerow([p[0], p[1]])
    print(f"✅ Guardado: {filename}")

# ==========================================
# MAIN
# ==========================================
if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "Oschersleben_map.yaml"
    
    # RRT puede necesitar menos resolución (factor más alto) para ser rápido
    downsample_factor = 6 

    # Coordenadas (Ajusta si caes en obstáculo)
    x_start, y_start = -0.5, 0.0
    x_goal, y_goal = 0.5, -0.5

    print("--- Cargando Mapa ---")
    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    env = grid_from_map(map_bin)

    start = world_to_map(x_start, y_start, resolution, origin)
    goal = world_to_map(x_goal, y_goal, resolution, origin)
    
    # Validaciones
    if start in env.obstacles or goal in env.obstacles:
        print("❌ ERROR: Start o Goal caen en obstáculo. Cambia las coordenadas.")
        sys.exit()

    print(f"Start (grid): {start}, Goal (grid): {goal}")

    # ---------------------------------------------------------
    # PARTE 1: LPA* (Tu código original)
    # ---------------------------------------------------------
 #   print("\n🔹 Ejecutando LPA*...")
 #   planner = SearchFactory()("lpa_star", start=start, goal=goal, env=env)
 #   planner.run()
 #   _, path_lpa, _ = planner.plan()
#
 #   if path_lpa:
 #       # Invertir porque la libreria suele dar Goal->Start
 #       path_lpa = list(reversed(path_lpa)) 
        
        # Guardar 0.5m
 #       lpa_05 = resample_path_by_distance(path_lpa, 0.5, resolution, origin, map_bin.shape[0])
 #       save_final_csv(lpa_05, "trayectoria_lpa_05.csv")
        
        # Guardar 1.0m
 #       lpa_10 = resample_path_by_distance(path_lpa, 1.0, resolution, origin, map_bin.shape[0])
  #      save_final_csv(lpa_10, "trayectoria_lpa_10.csv")
  #  else:
  #      print("❌ LPA* falló.")

    # ---------------------------------------------------------
    # PARTE 2: RRT (Nueva implementación compatible)
    # ---------------------------------------------------------
    print("\n🔹 Ejecutando RRT...")
    # step_size está en PIXELES de la grilla reducida. 
    # Con downsample=6, 5 pixeles son como 1.5 metros aprox.
    rrt = RRT_Grid(start=start, goal=goal, env=env, step_size=5.0, max_iter=5000)
    path_rrt = rrt.plan()

    if path_rrt:
        # RRT devuelve Goal -> Start, invertimos
        path_rrt = list(reversed(path_rrt))
        
        # Guardar 0.5m
        rrt_05 = resample_path_by_distance(path_rrt, 0.5, resolution, origin, map_bin.shape[0])
        save_final_csv(rrt_05, "trayectoria_rrt_05.csv")
        
        # Guardar 1.0m
        rrt_10 = resample_path_by_distance(path_rrt, 1.0, resolution, origin, map_bin.shape[0])
        save_final_csv(rrt_10, "trayectoria_rrt_10.csv")
    else:
        print("❌ RRT falló.")