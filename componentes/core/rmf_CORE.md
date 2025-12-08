# RMF Demos — Contexto General de Mundos y Estructura del Proyecto

(Contenido basado únicamente en el README original del repositorio, manteniendo estructura, nombres de mundos y sus imágenes.)

---

## 1. Introducción

El repositorio **RMF Demos** contiene una colección de mundos diseñados para demostrar las capacidades del *Open Robotics Middleware Framework (Open-RMF)* en escenarios de tráfico multi-robot, infraestructura compartida y sistemas de coordinación como ascensores, puertas y estaciones de dispensado.

---

## 2. Estructura General del Proyecto

| Carpeta | Contenido |
|---------|-----------|
| **rmf_demos_gz/** | Archivos `*.launch.xml` para ejecutar los mundos en Gazebo/Ignition. |
| **rmf_demos_maps/** | Mapas creados con *traffic_editor*. |
| **rmf_demos_tasks/** | Scripts para despachar tareas como delivery, loop, clean y patrol. |
| **rmf_demos_assets/** | Modelos 3D y recursos usados por los mundos. |
| **rmf_demos_bridges/** | Puentes como el bridge MQTT para RobotManager. |
| **docs/** | Documentación adicional, incluyendo Secure Office, FAQ, etc. |

---

## 3. Mundos disponibles

### 3.1. Hotel World

Mundo con lobby y dos niveles superiores. Incluye ascensores, puertas y tres flotas de robots.

![](../media/hotel_world.png)
![](../media/hotel_scenarios.gif)

**Launch file:**  
```
hotel.launch.xml
```

---

### 3.2. Office World

Entorno interior con puertas controladas, laneways y estaciones de dispensado e ingestión.

![](../media/delivery_request.gif)
![](../media/loop_request.gif)

**Launch file:**  
```
office.launch.xml
```

---

### 3.3. Airport Terminal World

Terminal aeroportuaria de gran escala con Crowd Simulation y vehículo de tipo `read_only` (caddy).

![](../media/airport_terminal_traffic_editor_screenshot.png)
![](../media/airport_terminal_demo_screenshot.png)
![](../media/caddy.gif)

**Launch file:**  
```
airport_terminal.launch.xml
```

---

### 3.4. Clinic World

Clínica con dos niveles, ascensores y diferentes flotas coordinadas.

![](../media/clinic.png)
![](../media/robot_taking_lift.gif)
![](../media/clinic.gif)

**Launch file:**  
```
clinic.launch.xml
```

---

### 3.5. Campus World

Campus exterior de gran escala, con rutas anotadas en coordenadas WGS84 e integración con RobotManager vía MQTT.

![](../media/campus.gif)

**Launch file:**  
```
campus.launch.xml
```

---

### 3.6. Manufacturing & Logistics World

Simulación industrial con manipuladores, conveyors y AMRs interoperando mediante Open-RMF.

![](https://img.youtube.com/vi/oSVQrjx_4w4/0.jpg)

---

## 4. Demos adicionales

### Traffic Light Robot Demos

| Escenario | Archivo |
|----------|---------|
| Triple-H | `triple_H.launch.xml` |
| Battle Royale | `battle_royale.launch.xml` |
| Office Traffic Light | `office_mock_traffic_light.launch.xml` |

---

## 5. Funcionalidades adicionales del repositorio

- Flexible Task Scripts  
- Lift Watchdog  
- Custom Docking Sequence  
- Emergency Alarm  
- Task Dispatching mediante proceso de bidding  

---

## 6. Resumen final

Este repositorio reúne múltiples mundos listos para ejecución en Gazebo/Ignition, representando distintos escenarios reales donde Open-RMF coordina robots, infraestructura y tareas. Cada mundo refleja un caso práctico y sirve como base para simulación, investigación o integración con sistemas robóticos reales.

