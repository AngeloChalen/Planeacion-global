# Planificador de Trayectorias A* para F1TENTH

Este repositorio permite cargar mapas tipo `YAML + PNG`, realizar la binarización, aplicar reducción de resolución (*downsampling*), convertir a formato de cuadrícula, ejecutar varios algoritmos de planificación y exportar el camino generado en un archivo `.csv`.

---

## Requisitos

- Python 3.8 o superior
- pip
- `python3-venv` (para crear entornos virtuales)
- Sistema operativo Linux (probado en Ubuntu 22.04)

---

## Instalación en entorno virtual (Linux)

### 1. Instalar librería para crear entornos virtuales (si no está instalada)

```bash
sudo apt update
sudo apt install python3-venv
```

### 2. Clonar el repositorio

```bash
git clone https://github.com/widegonz/Global_Planner.git
cd Global_Planner
```

### 3. Crear el entorno virtual y activarlo

```bash
python3 -m venv venv
source venv/bin/activate
```

### 4. Instalar dependencias

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### 5. Ejecutar el planificador

```bash
cd f1tenth
python3 f1tenth_map.py
```

Es importante que dentro de la carpeta se encuentre el archivo `.png` y `.yaml` para poder realizar correctamente el calculo del camino.

## Resultados
Se genera un archivo astar_path_real.csv con el camino óptimo en coordenadas del mundo real:

```bash
x,y
-1.73,2.12
-1.67,2.06
...
```
