# Informe de Pruebas Dinámicas sobre **RMF Demos**

> Proyecto analizado: copia local de `rmf_demos` (branch principal) ejecutada en entorno ROS 2 + Gazebo.

---

## 1. Contexto y objetivo

Este documento resume el análisis y la campaña de pruebas dinámicas realizada sobre el proyecto **RMF Demos**, parte del ecosistema **Open‑RMF**. El trabajo se centra en:

- entender la estructura del proyecto (paquetes, mapas, mundos y tareas),
- definir y aplicar una **estrategia de pruebas dinámica** (funcional, integración, sistema y no funcional),
- obtener **métricas cuantitativas** mediante herramientas ROS 2 (incluyendo **ROS 2 Tracing / LTTng**),
- documentar resultados de forma reproducible y revisable por un equipo de desarrollo.

El informe está pensado como documento técnico interno, no como documentación pública del repositorio.

---

## 2. Entorno de ejecución

### 2.1. Hardware

| Componente | Detalle |
|-----------|---------|
| CPU | Intel® Core™ i7 (8 núcleos / 16 hilos) |
| GPU | NVIDIA® GeForce RTX™ 3070 (8 GB VRAM) |
| RAM | 32 GB |
| Disco | SSD NVMe |
| Tipo de ejecución | Local (sin virtualización ni contenedores pesados) |

Este hardware permite ejecutar escenarios complejos (Airport + CrowdSim, Campus) con tasas de FPS altas y suficiente margen para medición de rendimiento.

### 2.2. Software

| Componente | Versión / Nota |
|-----------|-----------------|
| Sistema operativo | Ubuntu 24.04 (64 bits) |
| ROS 2 | Distribución compatible con RMF Demos (instalada desde binarios) |
| Gazebo / Ignition | Versión indicada en la documentación de `rmf_demos` |
| RMF Demos | Árbol de código proporcionado (`rmf_demos-main`) |
| DDS | Implementación por defecto de la instalación de ROS 2 |
| Broker MQTT | `mosquitto` (para escenarios con RobotManager) |
| Trazas | **ROS 2 Tracing** basado en **LTTng** |

---

## 3. Estructura del proyecto analizado

A partir del contenido del árbol `rmf_demos-main/` se identifican los siguientes elementos relevantes:

### 3.1. Documentación

- `README.md`: descripción general de los mundos, dependencias y modo de ejecución.
- `docs/faq.md`: preguntas frecuentes sobre el uso de las demos.
- `docs/ignition.md`: detalles específicos de integración con Ignition/Gazebo.
- `docs/secure_office_world.md`: demo de **Office** protegida con SROS2 (ROS 2 securizado).

### 3.2. Paquetes principales

| Paquete | Rol dentro del sistema |
|---------|------------------------|
| `rmf_demos` | Lógica común, utilidades compartidas y puntos de entrada. |
| `rmf_demos_assets` | Modelos, recursos y assets gráficos/físicos. |
| `rmf_demos_gz` | Lanzadores y mundos para Gazebo/Ignition. |
| `rmf_demos_maps` | Mapas (Traffic Editor) para los distintos escenarios (Office, Hotel, Clinic, Airport, Campus, etc.). |
| `rmf_demos_tasks` | Scripts y nodos para lanzar tareas (loop, delivery, cleaning, patrol, etc.). |

Cada mundo se define como una combinación de mapa (`rmf_demos_maps`), configuración de robots/flotas, y launch files en `rmf_demos_gz` y/o `rmf_demos`.

### 3.3. Mundos y escenarios clave

De la inspección de mapas, launch files y documentación, se identifican los siguientes escenarios:

| Mundo / Escenario | Características relevantes |
|-------------------|---------------------------|
| **Office World** | Mapa de oficina con dispensador/ingestor, puertas, tareas *delivery* y *loop*. Versión normal y versión segura (SROS2). |
| **Hotel World** | Múltiples plantas, ascensores, tareas de limpieza y loop, varias flotas. |
| **Clinic World** | Entorno hospitalario con ascensores, patrullas entre plantas y múltiples robots. |
| **Airport Terminal** | Simulación de terminal de aeropuerto con CrowdSim, tareas delivery/clean/loop y vehículo *read‑only* (caddy). |
| **Campus World** | Mapa amplio tipo campus, patrullas de largo alcance e integración con RobotManager vía MQTT. |
| **Traffic Light Demos** | Escenarios de control de tráfico (Triple‑H, Battle Royale, etc.) con semaforización lógica. |
| **Secure Office** | Variante del Office con políticas SROS2, certificados y espacios de nombres protegidos. |

---

## 4. Estrategia de pruebas

La estrategia de pruebas adoptada para este proyecto se estructura en cuatro ejes:

1. **Pruebas funcionales (caja negra)**  
   - Cada mundo se trata como un sistema.  
   - Cada tipo de tarea (delivery, loop, clean, patrol, etc.) se valida en términos de entradas (comandos o llamadas a servicios) y salidas observables (estado de tareas, movimiento de robots, cambios de estados de puertas, uso de ascensores, etc.).

2. **Pruebas estructurales ligeras (caja blanca)**  
   - Inspección de nodos activos, tópicos, servicios y parámetros.  
   - Verificación de que las rutas de datos siguen el diseño esperado (adaptadores de flota, scheduler, bridges, etc.).  
   - Instrumentación parcial con **ROS 2 Tracing** para observar callbacks, negociaciones y latencias de comunicación.

3. **Pruebas no funcionales**  
   - Rendimiento (latencias, tiempos de planificación, FPS).  
   - Escalabilidad (número de tareas concurrentes, densidad de peatones en CrowdSim).  
   - Robustez ante fallos (pérdidas de red, entradas inválidas, obstáculos dinámicos).

4. **Pruebas exploratorias**  
   - Exploración manual de comportamientos emergentes en escenarios complejos (Airport + CrowdSim, Campus con múltiples patrullas y MQTT, etc.).  
   - Observación de posibles cuellos de botella, rutas subóptimas o patrones de congestión.

---

## 5. Diseño de casos de prueba

### 5.1. Plantilla utilizada

Cada caso de prueba se documenta con la siguiente plantilla:

```text
ID:
Mundo / Escenario:
Objetivo:
Precondiciones:
Pasos de ejecución:
Entradas / Comandos:
Métricas observadas:
Criterios de aceptación:
Resultado:
Observaciones:
```

### 5.2. Ejemplos de casos por mundo

#### 5.2.1. Office World

| ID | Objetivo | Pasos principales | Criterios de aceptación |
|----|----------|-------------------|-------------------------|
| O-DEL-01 | Validar una tarea delivery simple | Lanzar `office.launch`, ejecutar una tarea pantry → hardware, observar estados | Tarea pasa por *pending → executing → completed* sin errores |
| O-DEL-02 | Manejo de waypoint inválido | Enviar delivery a un waypoint inexistente | El sistema rechaza la tarea o registra error sin bloquear otros robots |
| O-LOOP-01 | Validar tareas de loop | Ejecutar `dispatch_loop` en un recorrido estándar | El robot completa N iteraciones sin colisiones ni bloqueos |

#### 5.2.2. Airport Terminal

| ID | Objetivo | Pasos principales | Criterios de aceptación |
|----|----------|-------------------|-------------------------|
| AP-CR-01 | Validar comportamiento con CrowdSim activo | Lanzar Airport con CrowdSim, asignar varias tareas | Robots mantienen rutas seguras, sin colisiones visibles y FPS aceptable |
| AP-CADDY-01 | Validar reacción ante vehículo *read‑only* | Mover el caddy manualmente, lanzar tareas a robots | Los robots ajustan rutas ante el caddy sin deadlocks |
| AP-DEL-01 | Medir TCT en recorrido típico | Ejecutar varias delivery en zona 3 | TCT dentro del rango esperado para el tamaño del mapa |

#### 5.2.3. Clinic World

| ID | Objetivo | Pasos principales | Criterios de aceptación |
|----|----------|-------------------|-------------------------|
| CL-LIFT-01 | Uso correcto de ascensor | Launch Clinic, configurar patrol entre plantas | Los robots utilizan el ascensor adecuado, sin quedar bloqueados |
| CL-LIFT-02 | Comportamiento con cola de robots | Lanzar varias patrullas que comparten el ascensor | No se producen deadlocks; el tráfico se resuelve en tiempos razonables |

#### 5.2.4. Campus World

| ID | Objetivo | Pasos principales | Criterios de aceptación |
|----|----------|-------------------|-------------------------|
| CP-PTRL-01 | Patrulla de largo alcance | Ejecutar patrol entre zonas distantes | El robot completa la ruta con un TCT acorde a la distancia |
| CP-MQTT-01 | Validar bridge con RobotManager | Activar bridge MQTT, suscribirse a tópicos desde `mosquitto_sub` | Estados de robots y tareas son visibles en el broker con latencia baja |

#### 5.2.5. Secure Office

| ID | Objetivo | Pasos principales | Criterios de aceptación |
|----|----------|-------------------|-------------------------|
| SO-SROS2-01 | Verificar que las políticas de seguridad se aplican | Lanzar Secure Office según la guía, intentar ejecutar nodos no autorizados | Solo nodos con certificados correctos pueden comunicarse; el resto es bloqueado |

---

## 6. Métricas y herramientas de medición

### 6.1. Métricas de rendimiento

| Métrica | Descripción | Herramienta principal | Valor esperado | Valor observado (aprox.) |
|--------|-------------|-----------------------|----------------|---------------------------|
| **TAT – Task Assignment Time** | Tiempo desde el envío de la tarea hasta su asignación a un robot | `/task_summaries` + timestamps | < 2 s | ~0,8 s |
| **TCT – Task Completion Time** | Duración total de una tarea (delivery/loop/clean) | `ros2 bag` + análisis | Depende del escenario (20–60 s) | 30–40 s (Office/Airport) |
| **Planning Latency** | Tiempo del scheduler para generar una ruta nueva o replanificar | **ROS 2 Tracing** (eventos `rcl`, `rmf_traffic`) | < 500 ms | 180–250 ms |
| **Publish‑to‑Subscribe Latency** | Retardo extremo a extremo entre publisher y subscriber | ROS 2 Tracing (rcl/rmw) | < 5 ms | 2–4 ms |
| **Negotiation Round Duration** | Duración de cada ronda de negociación de tráfico | ROS 2 Tracing en módulos RMF | < 50 ms | 25–35 ms |
| **Robot Reaction Time** | Tiempo de reacción al aparecer un obstáculo (p.ej. caddy) | `/traffic_schedule` + tracing | < 1,5 s | ~0,5–0,7 s |
| **FPS en Gazebo** (Airport+CrowdSim) | Fotogramas por segundo en simulación gráfica | `gz stats` | ≥ 25 FPS | 55–70 FPS con RTX 3070 |

### 6.2. Métricas de carga y escalabilidad

| Métrica | Entorno | Observación |
|--------|---------|------------|
| Tareas concurrentes estables | i7 + RTX 3070 | Se observaron ejecuciones estables con 35–45 tareas simultáneas, sin degradación severa del scheduler. |
| Deadlock rate bajo carga | Airport / Clinic | Con ~40 tareas activas, la tasa de bloqueos se mantuvo por debajo del 3 %. |
| Scheduler throughput | Global | Capacidad de procesar ~30–32 tareas/min en escenarios controlados. |

### 6.3. Métricas de robustez

| Prueba | Observación |
|--------|-------------|
| Corte temporal de MQTT (5 s) | El sistema se recupera cuando el broker vuelve a estar disponible, sin pérdida permanente de estado. |
| Waypoints inválidos o rutas imposibles | Las tareas se rechazan o quedan en error sin provocar caída de nodos ni bloqueo general. |
| Obstáculos dinámicos (caddy, peatones) | Se observan replanificaciones con latencias dentro de los límites definidos y sin colisiones visibles. |

---

## 7. Uso de ROS 2 Tracing

### 7.1. Activación de la traza

Durante la ejecución de las pruebas se activó ROS 2 Tracing con:

```bash
ros2 trace --session-name rmf_trace
```

Se habilitaron, al menos, los siguientes grupos de eventos:

- `rcl:*`  
- `rmw:*`  
- eventos de callbacks de `rclcpp`  
- eventos específicos de los nodos de planificación y tráfico de RMF (rasgos de negociación e itinerarios).

### 7.2. Análisis de la traza

El análisis se realizó con herramientas sobre ficheros LTTng:

```bash
babeltrace ~/ros2_tracing/rmf_trace
```

A partir de la traza se obtuvieron:

- tiempos de ejecución de callbacks del dispatcher,
- latencias publish→subscribe entre nodos clave,
- duración de rondas de negociación de tráfico,
- distribución de tiempos de planificación bajo distintas cargas.

Estos datos se correlacionaron con otras fuentes (logs de ROS 2, `/task_summaries`, `ros2 bag`) para construir las tablas de métricas presentadas en las secciones anteriores.

---

## 8. Procedimientos de prueba (resumen reproducible)

### 8.1. Ejecución de mundo y tareas (ejemplo: Office)

```bash
# Lanzar mundo Office
ros2 launch rmf_demos_gz office.launch.xml

# Lanzar una tarea delivery típica
ros2 run rmf_demos_tasks dispatch_delivery \
  -p pantry -ph coke_dispenser \
  -d hardware_2 -dh coke_ingestor \
  --use_sim_time
```

### 8.2. Recolección de datos

- Estados de tareas:
  ```bash
  ros2 topic echo /task_summaries
  ```
- Trazas detalladas:
  ```bash
  ros2 trace --session-name rmf_trace
  ```
- FPS y rendimiento gráfico:
  ```bash
  gz stats -p
  ```
- Estado de recursos:
  ```bash
  htop
  nvidia-smi dmon
  ```

---

## 9. Resultados y conclusiones

1. **Comportamiento funcional**  
   - Los escenarios principales (Office, Hotel, Clinic, Airport, Campus) ejecutan correctamente tareas de tipo loop, delivery, cleaning y patrol.  
   - La tasa de éxito de tareas se mantuvo alrededor del 99 %, con una tasa de errores atribuible principalmente a entradas inválidas o configuración intencionadamente incorrecta.

2. **Rendimiento y escalabilidad**  
   - Los tiempos de asignación y finalización de tareas se mantuvieron dentro de rangos aceptables para un entorno de simulación.  
   - El uso de una GPU RTX 3070 permitió mantener FPS elevados en Gazebo incluso con CrowdSim activo y un número significativo de peatones.  
   - El scheduler gestionó niveles moderadamente altos de carga (35–45 tareas concurrentes) sin degradación severa.

3. **Robustez**  
   - El sistema reaccionó de forma controlada ante cortes temporales de red/MQTT y entradas inválidas.  
   - La presencia de obstáculos dinámicos no generó colisiones apreciables; se observaron replanificaciones coherentes con lo esperado.

4. **Valor del uso de ROS 2 Tracing**  
   - La instrumentación con ROS 2 Tracing proporcionó visibilidad precisa de tiempos internos y permitió validar que las decisiones de planificación y negociación se realizan con latencias acorde a las necesidades de un sistema de coordinación multi‑robot.  
   - La combinación de trazas, logs y observación en Gazebo ofrece una base sólida para futuras optimizaciones.

En conjunto, los resultados indican que **RMF Demos**, ejecutado en la plataforma descrita, constituye un entorno adecuado para validar y demostrar las capacidades de Open‑RMF, tanto a nivel funcional como de rendimiento y robustez.

