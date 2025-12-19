# Planificador Global de Trayectorias: LPA* y RRT (F1TENTH - Oschersleben)

**Estudiante:** Angelo Chalen
**Mapa Asignado:** Oschersleben
**Algoritmos:** LPA* (Parte A) y RRT (Parte B)

---

## 📘 Descripción del Proyecto

Este repositorio contiene la implementación de dos algoritmos de planificación de trayectorias globales sobre un mapa de ocupación (GridMap).

### 1. Algoritmo LPA* (Lifelong Planning A*)
Es un algoritmo de búsqueda incremental que combina la eficiencia de A* con la capacidad de reutilizar cálculos previos. Aunque en esta práctica se usa en un mapa estático, LPA* calcula la ruta óptima minimizando el costo `g(n) + h(n)` desde el inicio hasta la meta, garantizando la trayectoria más corta posible evitando obstáculos.

### 2. Algoritmo RRT (Rapidly-exploring Random Tree)
Es un algoritmo basado en muestreo probabilístico. En lugar de buscar en una grilla sistemática, RRT construye un árbol explorando el espacio libre mediante puntos aleatorios. Esto genera trayectorias factibles rápidamente, aunque no necesariamente óptimas, caracterizadas por su forma irregular o en zig-zag.

---

## ⚙️ Requisitos

- Python 3.8 o superior
- pip
- `python3-venv` (para entornos virtuales)
- Sistema operativo Linux (probado en Ubuntu 22.04)
- Archivos del mapa: `Oschersleben_map.png` y `.yaml` (incluidos en la carpeta `f1tenth`)

---

## 🚀 Instalación y Ejecución

Siga estos pasos exactos para replicar los resultados:

### 1. Preparar el entorno

```bash
# Instalar venv si no lo tiene
sudo apt update
sudo apt install python3-venv

# Clonar el repositorio
git clone [https://github.com/AngeloChalen/Planeacion-global.git](https://github.com/AngeloChalen/Planeacion-global.git)
cd Global_Planner

# Crear y activar entorno virtual
python3 -m venv venv
source venv/bin/activate

# Instalar dependencias
pip install --upgrade pip
pip install -r requirements.txt
