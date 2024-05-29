# SocialTech Challenge - Universidad de Deusto

![Universidad de Deusto](assets/deusto-logo.png)

## Introducción

Bienvenido al repositorio de la aplicación desarrollada para el **SocialTech Challenge** representando a la **Universidad de Deusto**. Esta aplicación está diseñada para gestionar y controlar un robot utilizando ROS (Robot Operating System) y una interfaz gráfica desarrollada en PyQt5. La aplicación permite a los usuarios iniciar sesión, seleccionar rutas predefinidas, crear rutas personalizadas, visualizar cuadros de mando, acceder a un mapa interactivo y ajustar configuraciones.

## Funcionalidades

- **Inicio de Sesión**: Permite a los usuarios iniciar sesión o registrarse en la aplicación.
- **Rutas Disponibles**: Muestra una lista de rutas predefinidas que el robot puede seguir.
- **Crea tu Ruta**: Permite a los usuarios crear y guardar rutas personalizadas.
- **Cuadros de Mando**: Visualiza estadísticas y cuadros de mando del robot.
- **Mapa Interactivo**: Accede a un mapa interactivo para ver la posición y el estado del robot.
- **Ajustes**: Ajusta configuraciones de la aplicación.
- **Cerrar Sesión**: Permite a los usuarios cerrar sesión y volver a la pantalla de inicio.

## Estructura del Proyecto

```plaintext
socialtech/
├── main.py
├── gui/
│   ├── __init__.py
│   ├── welcome_window.py
│   ├── login_window.py
│   ├── main_menu_window.py
│   ├── register_window.py
│   ├── default_routes_window.py
├── assets/
│   └── login_image.jpg
├── database/
│   ├── __init__.py
│   ├── create_db.py
│   └── robot_data.db
├── scripts/
│   ├── __init__.py
│   ├── ros_routes.py
│   └── ros_scripts.py
├── README.md
└── requirements.txt
```

## Instalación

### Requisitos

- Python 3.x
- ROS Noetic (para la ejecución en el robot)
- PyQt5
- SQLite3

### Instalación de Dependencias

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y python3 python3-venv python3-pip git sqlite3
sudo apt install -y ros-noetic-desktop-full
```
### Clonar el Repositorio

```bash
git clone https://github.com/username/socialtech-challenge.git
cd socialtech-challenge
```

### Crear un Entorno Virtual

```bash
python3 -m venv venv
source venv/bin/activate  # Para desactivar el entorno, usa `deactivate`
```

### Instalar Dependencias

```bash
pip install -r requirements.txt
```

### Crear la Base de Datos

```bash
python database/create_db.py
```

### Ejecución

```bash
python main.py
```