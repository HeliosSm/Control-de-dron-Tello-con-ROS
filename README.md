
# Control de dron Tello con ROS 2

Repositorio del proyecto **Control de dron Tello con ROS 2**, que integra comunicación con un dron DJI Tello, visión por computadora en tiempo real y despliegue contenerizado mediante Docker.  
El proyecto está organizado como un workspace de ROS 2 (`ros2_ws`) con nodos para:

- Conexión y control del dron  
- Visualización de video en vivo  
- Monitoreo de telemetría (batería, altura, estado)  
- Sistema de seguridad por batería baja  
- Ejecución de una misión autónoma  
- Detección y conteo de objetos rojos y negros

---

## Tecnologías utilizadas

- **ROS 2 Humble** (middleware robótico)
- **Python 3**
- **Docker + Docker Compose**
- **OpenCV** (visión por computador)
- **djitellopy** (API para drones DJI Tello)
- **cv\_bridge** (conversión entre mensajes ROS e imágenes OpenCV)

---

## Estructura del repositorio

El repositorio contiene el workspace completo de ROS 2:

```text
ros2_ws/
├── src/
│   └── tello_project/
│       ├── tello_driver.py
│       ├── video_viewer.py
│       ├── telemetry_monitor.py
│       ├── battery_failsafe.py
│       ├── mission_planner.py
│       ├── object_detector.py
│       ├── debug_tello.py
│       ├── drone_conector.py
│       ├── __init__.py
│       └── setup.py
├── Dockerfile
├── docker-compose.yml
├── start.sh
├── .devcontainer/
│   └── devcontainer.json
├── build/          # (se genera al compilar)
├── install/        # (se genera al compilar)
└── log/            # (se genera al ejecutar)
````

> ⚠️ Las carpetas `build/`, `install/` y `log/` se pueden regenerar; lo importante es el contenido de `src/` y los archivos de configuración.

---

## Requisitos previos

1. **Hardware**

   * Dron **DJI Tello** (o compatible con `djitellopy`).
   * PC / Laptop con conexión WiFi (para conectarse a la red del Tello).

2. **Software**

   * **Docker** y **Docker Compose** instalados.
   * Sistema operativo tipo Linux (nativo o WSL2/VM) con soporte para X11.
   * Servidor gráfico X11 habilitado (para mostrar ventanas de OpenCV desde el contenedor).

3. **Conectividad**

   * Conectarse a la red WiFi del Tello (por ejemplo, `TELLO-XXXXXX`) antes de ejecutar el sistema.
   * Verificar que se puede hacer ping a la IP del dron (típicamente `192.168.10.1`).

---

## Puesta en marcha rápida

Desde el directorio `ros2_ws/`:

### 1. Permitir acceso gráfico a Docker (host)

En el sistema anfitrión (máquina física):

```bash
xhost +local:docker
```

### 2. Construir y desplegar el contenedor

```bash
cd ros2_ws
docker-compose up --build
```

Esto hará:

* Construir la imagen basada en `ros:humble`.
* Instalar dependencias (OpenCV, `djitellopy`, `cv_bridge`, etc.).
* Montar el workspace en el contenedor.
* Ejecutar el script `start.sh`, que inicia todos los nodos ROS 2 en orden.

Si todo es correcto, deberías ver mensajes similares a:

```text
=== INICIANDO SISTEMA TELLO ROS2 ===
[1/6] Iniciando tello_driver...
[2/6] Iniciando video_viewer...
...
[6/6] Iniciando object_detector...
=== TODOS LOS NODOS INICIADOS ===
Ctrl+C para detener
```

---

## Nodos implementados

Todos los nodos viven en el paquete `tello_project`.

### `tello_driver`

Nodo central que se comunica directamente con el Tello usando `djitellopy`.

* **Publica:**

  * `/tello/image_raw` (`sensor_msgs/msg/Image`) – video en bruto.
  * `/tello/battery` (`std_msgs/msg.Int32`) – nivel de batería (%).
  * `/tello/height` (`std_msgs/msg.Float32`) – altura (cm).
  * `/tello/status` (`std_msgs/msg.String`) – resumen de estado.

* **Suscribe:**

  * `/tello/control` (`std_msgs/msg.String`) – comandos de vuelo:

    * `takeoff`, `land`, `forward 50`, `back 50`, `left 50`,
      `right 50`, `up 50`, `down 50`, `cw 90`, `ccw 90`, `flip f`, etc.

### `video_viewer`

Visualiza el video en tiempo real.

* **Suscribe:** `/tello/image_raw`
* **Función:** convierte el mensaje `Image` a matriz OpenCV (BGR) y muestra una ventana de video en vivo.

### `telemetry_monitor`

Monitorea telemetría básica.

* **Suscribe:**

  * `/tello/battery`
  * `/tello/height`
* **Función:** imprime en consola el nivel de batería y la altura del dron.

### `battery_failsafe`

Sistema de seguridad por batería baja.

* **Suscribe:** `/tello/battery`
* **Publica:** `/tello/control` (cuando la batería cae por debajo del umbral).
* **Función:** si el nivel de batería es crítico, envía el comando `land` de forma automática.

### `mission_planner`

Ejecuta una misión autónoma predefinida.

* **Publica:** `/tello/control`
* **Secuencia típica:**

  1. `command`
  2. `takeoff`
  3. `back 50`
  4. `cw 90`
  5. `ccw 90`
  6. `forward 50`
  7. `land`

### `object_detector`

Detecta y cuenta objetos rojos y negros en el video.

* **Suscribe:** `/tello/image_raw`
* **Publica:** `/detected_objects` (`std_msgs/msg.Int32`) – total de objetos detectados.
* **Función:**

  * Convierte la imagen a HSV.
  * Aplica máscaras para rojo (dos rangos) y negro.
  * Encuentra contornos, filtra por área mínima.
  * Dibuja rectángulos y etiquetas en la imagen.
  * Muestra el conteo de objetos en pantalla y lo publica vía ROS.

---

## Flujo típico de ejecución

1. El contenedor se levanta con `docker-compose up --build`.
2. `start.sh` inicia el nodo `tello_driver` y espera a que el dron responda.
3. Se encienden los nodos de visualización y monitoreo.
4. Se activa el nodo de misión autónoma (`mission_planner`), que envía comandos al Tello.
5. El nodo `object_detector` procesa cada frame y publica el conteo de objetos.
6. El nodo `battery_failsafe` supervisa la batería y aterriza el dron si es necesario.
7. El usuario puede detener todo el sistema con `Ctrl + C`.

---

## Desarrollo dentro del contenedor

El repositorio incluye configuración para Dev Containers de VS Code:

* Carpeta: `.devcontainer/devcontainer.json`
* Permite abrir el workspace `ros2_ws` directamente dentro del contenedor.
* Ejecuta `colcon build` automáticamente tras la creación del contenedor.
* Incluye extensiones recomendadas:

  * `ms-iot.vscode-ros`
  * `ms-python.python`

### Comandos útiles

Dentro del contenedor (o devcontainer):

```bash
# Activar entorno
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Ver tópicos activos
ros2 topic list

# Ver mensajes de batería
ros2 topic echo /tello/battery

# Ver el conteo de objetos detectados
ros2 topic echo /detected_objects
```

---

## Notas y buenas prácticas

* Asegúrate de tener espacio libre alrededor del dron al probar la misión autónoma.
* Verifica siempre el nivel de batería del Tello antes de despegar.
* Si no se visualiza el video:

  * Comprueba que estás en la red WiFi del Tello.
  * Revisa que X11 está habilitado y el comando `xhost +local:docker` fue ejecutado.
* Para depurar problemas de imagen, puedes usar el nodo auxiliar `debug_tello.py` para guardar una captura en disco.

---

## Licencia

Define aquí la licencia de tu proyecto (por ejemplo, MIT, Apache 2.0, GPL, etc.):

```text
Licencia pendiente de definir.
```

---

## Autores

Proyecto desarrollado para el control de un dron DJI Tello usando ROS 2, Docker y OpenCV en el contexto de prácticas y proyectos de redes de sensores.

```
Franklin Josue León Guanoquiza
Erick Alexander Ramón Chávez
Carlos Martin Vinces Segovi
```
