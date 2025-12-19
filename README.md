# Planificador Global de Trayectorias: LPA* y RRT (F1TENTH - Oschersleben)

**Estudiante:** Angelo Chalen
**Mapa Asignado:** Oschersleben
**Algoritmos:** LPA* (Parte A) y RRT (Parte B)



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
git clone https://github.com/AngeloChalen/Planeacion-global.git
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

### 5. Acceder a carpetaa f1tenth
```bash
cd f1tenth

```
### 6. Ejecutar el planificador
-LPA*

```bash
cd f1tenth
python3 f1tenth_map.py
```
- RRT
```bash
cd f1tenth
python3 rrtf1tenth_map.py
```
Es importante que dentro de la carpeta se encuentre el archivo `.png` y `.yaml` para poder realizar correctamente el calculo del camino.

