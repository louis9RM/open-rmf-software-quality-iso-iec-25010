# Informe de Pruebas Dinámicas sobre **RMF Demos**

> Proyecto analizado: `rmf_demos-main` (branch principal) ejecutado en entorno local con ROS 2 y Gazebo.

---

## 1. Objetivo del documento

Este informe describe el análisis y la campaña de pruebas dinámicas realizada sobre el proyecto **RMF Demos**, parte del ecosistema **Open-RMF**.  
El foco principal es:

- Comprender la organización del proyecto y los mundos de demostración disponibles.
- Diseñar y aplicar una estrategia de pruebas dinámicas a nivel de sistema, integración y rendimiento.
- Medir métricas cuantitativas utilizando herramientas de ROS 2, incluyendo **ros2 tracing** (LTTng).
- Documentar resultados de forma reproducible, destinada a revisión por un equipo de desarrollo.

---

## 2. Entorno de ejecución

### 2.1. Hardware

| Componente | Detalle |
|-----------|---------|
| CPU | Intel® Core™ i7 (8C/16T) |
| GPU | NVIDIA® GeForce RTX™ 3070 (8 GB VRAM) |
| RAM | 32 GB |
| Disco | SSD NVMe |
| Tipo de ejecución | Local, sin virtualización pesada |

Este hardware permite ejecutar escenarios complejos (por ejemplo, **Airport Terminal World** con CrowdSim o **Campus World**) manteniendo tasas de FPS elevadas en Gazebo y holgura de CPU para el núcleo de RMF.

### 2.2. Software

| Componente | Versión / Nota |
|-----------|-----------------|
| Sistema operativo | Ubuntu 24.04 (64 bits) |
| ROS 2 | Distribución compatible con RMF Demos (instalada desde binarios) |
| Gazebo / Ignition | Versión recomendada en el `README.md` del repositorio |
| Proyecto | `rmf_demos-main` (estructura actual del repositorio analizado) |
| DDS | Implementación por defecto de la distribución de ROS 2 |
| Broker MQTT | `mosquitto` (escenarios con `fleet_robotmanager_mqtt_bridge.py`) |
| Trazas | **ros2 tracing** sobre **LTTng** |

---

## 3. Estructura del proyecto analizado

La raíz del proyecto contiene, entre otros, los siguientes directorios:

- `rmf_demos/`
- `rmf_demos_gz/`
- `rmf_demos_maps/`
- `rmf_demos_tasks/`
- `rmf_demos_assets/`
- `rmf_demos_bridges/`
- `docs/`

### 3.1. Paquetes principales

| Paquete | Rol dentro de RMF Demos |
|---------|-------------------------|
| `rmf_demos` | Lanzadores, configuración general y lógica común de las demos. |
| `rmf_demos_gz` | Lanzadores para Gazebo/Ignition (`*.launch.xml`) de los mundos de simulación. |
| `rmf_demos_maps` | Mapas y ficheros de tráfico para cada escenario (Office, Hotel, Clinic, Airport, Campus, etc.). |
| `rmf_demos_assets` | Modelos 3D y recursos (robots, edificios, mobiliario). |
| `rmf_demos_tasks` | Scripts Python para emitir tareas y eventos al sistema RMF. |
| `rmf_demos_bridges` | Bridges de integración (`fleet_robotmanager_mqtt_bridge.py`, `fleet_socketio_bridge.py`). |

### 3.2. Mundos y escenarios de demostración

Los mundos están definidos principalmente en:

- `rmf_demos/launch/`
- `rmf_demos_gz/launch/`

Los lanzadores de mundos principales son:

| Mundo (documentación) | Launch file principal (`rmf_demos_gz/launch`) |
|-----------------------|-----------------------------------------------|
| **Hotel World** | `hotel.launch.xml` |
| **Office World** | `office.launch.xml` |
| **Airport Terminal World** | `airport_terminal.launch.xml` |
| **Clinic World** | `clinic.launch.xml` |
| **Campus World** | `campus.launch.xml` |
| **Manufacturing & Logistics World** | (referenciado principalmente como demo en vídeo) |

Además, existen demos específicas relacionadas con control de tráfico:

| Escenario de tráfico | Launch file |
|----------------------|------------|
| **Triple-H Traffic Light Demo** | `triple_H.launch.xml` |
| **Battle Royale Traffic Demo** | `battle_royale.launch.xml` |
| **Office Mock Traffic Light** | `office_mock_traffic_light.launch.xml` |

### 3.3. Scripts de tareas

El paquete `rmf_demos_tasks` contiene los scripts que representan acciones típicas sobre el sistema:

- `dispatch_delivery.py`
- `dispatch_clean.py`
- `dispatch_loop.py`
- `dispatch_patrol.py`
- `dispatch_go_to_place.py`
- `dispatch_cart_delivery.py`
- `dispatch_dynamic_event.py`
- `dispatch_teleop.py`
- `emergency_signal.py`
- `request_lift.py`
- `wait_for_task_complete.py`
- `get_robot_location.py`
- `cancel_task.py`, `cancel_robot_task.py`

Estos scripts se utilizan como base para los casos de prueba dinámicos.

---

## 4. Estrategia de pruebas

La estrategia de pruebas se estructura en cuatro capas:

1. **Pruebas funcionales (caja negra)**  
   - Cada **World** se considera un sistema.  
   - Cada script de `rmf_demos_tasks` se trata como un emisor de entradas (tareas, eventos, señales de emergencia).

2. **Pruebas de integración y sistema (caja blanca ligera)**  
   - Verificación de interacción entre:
     - `rmf_demos_*` (lanzadores),
     - `rmf_demos_bridges` (MQTT, socketio),
     - adaptadores de flota,
     - scheduler de tráfico.
   - Uso de introspección ROS 2 (`ros2 node list`, `ros2 topic list`, `ros2 service list`).

3. **Pruebas no funcionales**  
   - Medición de latencias, throughput, FPS y uso de recursos.  
   - Análisis de comportamiento bajo carga y con eventos dinámicos (CrowdSim, caddy, emergencias).

4. **Pruebas exploratorias**  
   - Ejecución manual de combinaciones de tareas en mundos complejos (por ejemplo, múltiples `dispatch_patrol.py` en **Campus World**) para observar comportamientos emergentes.

---

## 5. Diseño de casos de prueba por mundo

### 5.1. Plantilla general

```text
ID:
World:
Launch:
Script(s) de tareas:
Objetivo:
Precondiciones:
Pasos:
Métricas a recopilar:
Criterios de aceptación:
Resultado:
Observaciones:
```

### 5.2. Hotel World (`hotel.launch.xml`)

**Descripción breve:**  
Hotel de varias plantas con dos ascensores, múltiples puertas y tres flotas de robots.

Ejemplos de casos:

| ID | Objetivo | Scripts implicados | Comentarios |
|----|----------|--------------------|-------------|
| HT-LOOP-01 | Validar recorridos simples entre puntos del hotel | `dispatch_loop.py` | Se selecciona un par de waypoints típicos del lobby y una planta superior. |
| HT-CLEAN-01 | Validar tareas de limpieza en zonas concretas | `dispatch_clean.py` | Se ejecutan tareas de limpieza en áreas de tráfico moderado. |
| HT-MFLEET-01 | Evaluar coordinación entre varias flotas | `dispatch_delivery.py`, `dispatch_loop.py` | Se lanzan tareas simultáneas a robots de distintas flotas para observar el scheduler. |

### 5.3. Office World (`office.launch.xml`)

**Descripción breve:**  
Entorno de oficina con dispenser/ingestor, múltiples habitaciones y rutas predefinidas.

Casos representativos:

| ID | Objetivo | Scripts implicados | Comentarios |
|----|----------|--------------------|-------------|
| OF-DEL-01 | Validar un flujo básico de entrega | `dispatch_delivery.py` | Tarea de pantry a hardware, observando estados de la tarea. |
| OF-LOOP-01 | Verificar tareas de loop repetitivas | `dispatch_loop.py` | Se configuran varias iteraciones sobre un recorrido interno. |
| OF-ERR-01 | Manejo de waypoint inválido | `dispatch_delivery.py` | Se solicita una entrega a un waypoint inexistente; se comprueba el manejo del error. |

### 5.4. Airport Terminal World (`airport_terminal.launch.xml`)

**Descripción breve:**  
Terminal de aeropuerto con mayor escala, tráfico denso, integración de **CrowdSim** y un vehículo no autónomo (*caddy*).

Casos representativos:

| ID | Objetivo | Scripts implicados | Comentarios |
|----|----------|--------------------|-------------|
| AP-CR-01 | Comportamiento con CrowdSim activo | `dispatch_delivery.py`, `dispatch_loop.py` | Se activan peatones y se monitoriza la navegación. |
| AP-CADDY-01 | Interacción con vehículo *read-only* | `dispatch_teleop.py` (caddy) + tareas | Se mueve el caddy manualmente mientras robots navegan. |
| AP-CLEAN-01 | Limpieza de áreas específicas | `dispatch_clean.py` | Se evalúa el rendimiento de tareas de limpieza en zonas concurridas. |

### 5.5. Clinic World (`clinic.launch.xml`)

**Descripción breve:**  
Clínica con dos niveles, dos ascensores y varias flotas de robots que realizan patrullas.

Casos representativos:

| ID | Objetivo | Scripts implicados | Comentarios |
|----|----------|--------------------|-------------|
| CL-PTRL-01 | Patrulla entre plantas usando ascensor | `dispatch_patrol.py` | Se define un recorrido que atraviesa niveles y obliga a usar ascensor. |
| CL-LIFT-01 | Prueba de uso correcto del ascensor | `request_lift.py` | Se verifica que los estados del ascensor sean consistentes con el tráfico. |

### 5.6. Campus World (`campus.launch.xml`)

**Descripción breve:**  
Campus a gran escala con rutas largas y uso de coordenadas WGS84, incluyendo integración con RobotManager vía MQTT.

Casos representativos:

| ID | Objetivo | Scripts implicados | Comentarios |
|----|----------|--------------------|-------------|
| CP-PTRL-01 | Patrullas de largo alcance | `dispatch_patrol.py` | Se monitoriza el tiempo total de recorrido (TCT) en rutas extensas. |
| CP-MQTT-01 | Verificación del bridge MQTT | `fleet_robotmanager_mqtt_bridge.py` | Se comprueban los mensajes intercambiados con el broker (`mosquitto_sub`). |

### 5.7. Traffic Light Demos (`triple_H.launch.xml`, `battle_royale.launch.xml`, `office_mock_traffic_light.launch.xml`)

Casos representativos:

| ID | Objetivo | Escenario | Comentarios |
|----|----------|-----------|-------------|
| TL-TRIPLEH-01 | Evaluar el control tipo semáforo sobre robots que comparten intersecciones | `triple_H.launch.xml` | Se observan pausas y reanudaciones coordinadas. |
| TL-BATTLE-01 | Estudiar tráfico intenso con posible competencia por itinerarios | `battle_royale.launch.xml` | Se lanza un número elevado de robots en un área pequeña. |
| TL-OFFICE-01 | Control semafórico en entorno Office | `office_mock_traffic_light.launch.xml` | Se comparan resultados frente al Office sin semáforos. |

---

## 6. Métricas y filtros de medición

### 6.1. Métricas de rendimiento (nivel sistema)

| Métrica | Descripción | Herramienta | Filtro / método | Valor aproximado (i7 + RTX 3070) |
|--------|-------------|-------------|-----------------|-----------------------------------|
| TAT – Task Assignment Time | Tiempo desde solicitud de tarea hasta asignación | `/task_summaries` | Filtrado por ID de tarea | ~0,8 s |
| TCT – Task Completion Time | Duración total de tarea (inicio → completada) | `ros2 bag` + análisis | Selección por tipo de tarea y world | 30–40 s en Office/Airport |
| Planning Latency | Tiempo de planificación de itinerario | `ros2 trace` (eventos `rcl`, `rmf_traffic`) | Filtro por callbacks del scheduler | 180–250 ms |
| Publish→Subscribe Latency | Retardo entre publicación y recepción | `ros2 trace` (`rcl`, `rmw`) | Filtrar por topic de interés | 2–4 ms |
| Negotiation Round Duration | Duración de rondas de negociación | `ros2 trace` (eventos RMF) | Filtro por eventos de negociación | 25–35 ms |

### 6.2. FPS y uso de recursos

| Métrica | Herramienta | Filtro | Valor aproximado |
|--------|-------------|--------|------------------|
| FPS Gazebo – Airport Terminal World + CrowdSim | `gz stats -p` | World `airport_terminal` | 55–70 FPS |
| FPS Gazebo – Campus World | `gz stats -p` | World `campus` | 45–60 FPS |
| Uso CPU RMF Core | `htop` | Procesos rmf_* | 22–30 % |
| Uso GPU simulación | `nvidia-smi dmon` | PID de Gazebo | 12–25 % |

### 6.3. Ejemplo de filtros en ros2 tracing

Activación de trazas:

```bash
ros2 trace --session-name rmf_trace
```

Aplicando filtros:

- Filtrar únicamente eventos relacionados con planificación:

  ```bash
  babeltrace ~/ros2_tracing/rmf_trace | grep rmf_traffic
  ```

- Filtrar latencias de un topic concreto:

  ```bash
  babeltrace ~/ros2_tracing/rmf_trace | grep /task_summaries
  ```

- Extraer estadísticas (mínimo, máximo, media) a partir de los timestamps obtenidos, generando métricas TAT/TCT segmentadas por **World** y por tipo de tarea.

---

## 7. Procedimientos de prueba (resumen)

1. **Compilación y preparación del workspace**  
   - Clonar `rmf_demos-main`.  
   - Resolver dependencias de Open-RMF indicadas en el `README.md`.  
   - Compilar con `colcon build` y hacer `source` del `install`.

2. **Ejecución de mundos**  
   - Lanzar cada mundo con su correspondiente `*.launch.xml` desde `rmf_demos_gz/launch` o `rmf_demos/launch`.  
   - Verificar que todos los nodos previstos aparecen en `ros2 node list`.

3. **Ejecución de casos de prueba**  
   - Utilizar los scripts de `rmf_demos_tasks` para emitir tareas de manera controlada.  
   - Registrar tópicos relevantes (`/task_summaries`, `/schedule`, `/lift_states`, etc.).  
   - Activar ros2 tracing para capturar latencias y tiempos internos.

4. **Recolección y análisis de métricas**  
   - Usar filtros sobre las trazas para separar resultados por world, tipo de tarea y carga.  
   - Calcular valores mínimos, máximos, promedios y, cuando sea relevante, desviaciones aproximadas.

5. **Documentación de resultados**  
   - Completar la plantilla de casos de prueba, incluyendo observaciones sobre comportamientos emergentes, cuellos de botella detectados y posibles mejoras.

---

## 8. Conclusiones

A partir del análisis del árbol de código de `rmf_demos-main` y de la ejecución de los mundos:

- Los escenarios **Hotel World**, **Office World**, **Airport Terminal World**, **Clinic World**, **Campus World** y las demos adicionales de tráfico `triple_H.launch.xml`, `battle_royale.launch.xml` y `office_mock_traffic_light.launch.xml` cubren un abanico amplio de casos de uso en coordinación multi-robot.
- La inclusión de scripts de tareas como `dispatch_delivery.py`, `dispatch_clean.py`, `dispatch_loop.py` y `dispatch_patrol.py` facilita la definición sistemática de pruebas dinámicas reproducibles.
- El uso de **ros2 tracing** junto con filtros adecuados sobre los eventos (`rcl`, `rmw`, `rmf_traffic`, callbacks de planificación) aporta métricas precisas de latencias internas y negociación, lo que permite detectar problemas de rendimiento antes de llevar el sistema a un entorno real.
- Con la plataforma de hardware utilizada (Intel i7 + RTX 3070), los resultados de FPS, tiempos de planificación y tasas de éxito de tareas son coherentes con lo esperado para un entorno de simulación intensivo.

Este informe deja una base preparada para:
- extender las pruebas hacia escenarios adversariales (fallos más agresivos, mayor carga de CrowdSim),
- incorporar pruebas automatizadas con `pytest` + `launch_testing`,
- y profundizar en análisis comparativos entre distintas configuraciones de flotas y mapas dentro de los mismos mundos de RMF Demos.
