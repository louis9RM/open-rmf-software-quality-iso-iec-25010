# Informe de Pruebas Dinámicas para **Open-RMF / rmf_demos**

## 1. Introducción

Este documento presenta una estrategia profesional de pruebas dinámicas aplicada al repositorio **open-rmf/rmf_demos**, cuyo propósito es evaluar el comportamiento funcional, estructural y no funcional de los escenarios de simulación ofrecidos por Open-RMF.

Se adopta una metodología basada en técnicas dinámicas, caja negra y caja blanca ligera, aplicando partición de equivalencia, valores límite, pruebas exploratorias y pruebas no funcionales.

---

## 2. Alcance del Sistema

El repositorio `rmf_demos` ofrece múltiples mundos y escenarios de prueba para evaluar componentes clave de RMF:

### **2.1. Worlds principales**
| World | Características clave |
|-------|------------------------|
| **Hotel World** | Tareas Loop y Clean, múltiples plantas, ascensores, varias flotas. |
| **Office World** | Delivery y Loop, puertas automáticas, dispenser e ingestor. |
| **Airport Terminal** | Loop, Delivery, Clean, crowd simulation, vehículo read-only (“caddy”). |
| **Clinic World** | Multiplanta, patrullas, ascensores. |
| **Campus World** | Escenario georreferenciado, integración MQTT, patrullas extensas. |
| **Manufacturing World** | Flujo logístico industrial (video demostrativo). |

### **2.2. Scenarios adicionales**
- Traffic Light Demos (Triple H, Battle Royale, Office mock).
- Flexible task scripts.
- Docking personalizado.
- Emergency alarm.
- Lift watchdog.

---

## 3. Estrategia de Pruebas

### **3.1. Tipos de pruebas**

| Tipo | Aplicación |
|------|------------|
| **Funcionales (caja negra)** | Validación de tareas y flujos en cada world. |
| **Estructurales (caja blanca ligera)** | Validación de nodos, tópicos, estados e interacciones ROS 2. |
| **No funcionales** | Carga, desempeño, robustez, comportamiento ante escenarios complejos. |
| **Exploratorias** | Descubrimiento de comportamientos emergentes y potenciales fallos. |

---

## 4. Metodología de Diseño de Pruebas

### **4.1. Partición de equivalencia**
Ejemplo para parámetro `-n` (patrol count):
- Valores válidos: n = 1, 2, …  
- Valores límite: 0, 1, 100.  
- Valores inválidos: negativos, no numéricos.

### **4.2. Análisis de valores límite**
Ejemplo sobre waypoints:
- Válido: waypoint existente.
- Límite: nombre mal escrito, vacío, no encontrado.

### **4.3. Plantilla de Caso de Prueba**
```
ID:
World:
Funcionalidad:
Precondiciones:
Entrada:
Resultado Esperado:
Criterios de Éxito:
```

---

## 5. Matriz General de Casos de Prueba

### **5.1. Hotel World**
| ID | Función | Entrada | Resultado esperado |
|----|---------|---------|---------------------|
| H-LOOP-01 | Loop | restaurant ↔ suite | Ruta válida, sin colisiones. |
| H-CLEAN-01 | Clean | clean_lobby | Robot completa limpieza. |
| H-MFLEET-01 | Multiflota | 3 tareas simultáneas | Scheduler coordina sin deadlocks. |

### **5.2. Office World**
| ID | Función | Entrada | Resultado esperado |
|----|---------|---------|---------------------|
O-DEL-01 | Delivery | pantry → hardware_2 | Estado: assigned → executing → completed. |
O-LOOP-01 | Loop | standard loop | Trayectoria sin bloqueos. |
O-ERR-01 | Delivery inválida | waypoint inexistente | Error manejado correctamente. |

### **5.3. Airport Terminal**
| ID | Función | Entrada | Resultado esperado |
|----|---------|---------|---------------------|
AP-CR-01 | CrowdSim | use_crowdsim=1 | Robots evitan peatones. |
AP-CADDY-01 | Read-only fleet | Caddy en teleop | Nueva planificación reactiva. |
AP-CLEAN-01 | Clean | from zone_3 | Limpieza exitosa. |

### **5.4. Clinic World**
| ID | Función | Entrada | Resultado esperado |
|----|---------|---------|---------------------|
CL-PTRL-01 | Patrol | patrol route 1 | Uso correcto de ascensor. |
CL-PTRL-02 | Multi-floor | route 2 → 1 | Sin atascos. |

### **5.5. Campus World**
| ID | Función | Entrada | Resultado esperado |
|----|---------|-----------|--------------------|
CP-MQTT-01 | MQTT bridge | mosquitto_sub | Recepción de estados. |
CP-PTRL-03 | Macro–patrol | campus_1–campus_4 | Tiempos adecuados. |

### **5.6. Traffic Light Demos**
| ID | Función | Entrada | Resultado esperado |
|----|---------|-----------|--------------------|
TL-TRIPLEH-01 | Pause/Resume | traffic light cmd | Robots respetan semáforo. |
TL-BATTLE-01 | Battle Royale | multi-robots | Flujo sin colisiones. |

---

## 6. Pruebas No Funcionales

### **6.1. Rendimiento**
| Caso | Métrica | Criterio |
|------|---------|----------|
C-RND-01 | Tiempo asignación de tarea | < 2 s |
C-RND-02 | Tiempo total entrega | Basado en mapa |

### **6.2. Carga**
- Lanzar 20+ tareas en Airport o Campus.
- Evaluar:
  - Deadlocks
  - Retrasos del scheduler
  - Consumo CPU/GPU

### **6.3. Robustez**
Pruebas:
- Añadir/remover caddy en movimiento.
- Saturar crowd simulation.
- Interrumpir MQTT temporalmente.

---

## 7. Herramientas Utilizadas

### **7.1. Ejecución y Simulación**
| Herramienta | Uso |
|-------------|-----|
| ROS 2 (CLI) | Ejecutar mundos y tareas. |
| Gazebo (Ignition) | Visualización 3D. |
| rmf-web | Dashboard y API server. |

### **7.2. Observación y Diagnóstico**
| Herramienta | Uso |
|-------------|------|
| ros2 topic/echo/list | Validación caja blanca. |
| ros2 bag | Grabación de escenarios. |
| RViz / schedule visualizer | Representación de rutas. |

### **7.3. Automatización**
| Herramienta | Uso |
|-------------|------|
| pytest + launch_testing | Automatizar escenarios funcionales. |

---

## 8. Procedimiento de Ejecución (Ejemplo)

### **8.1. Lanzar escenario**
```bash
ros2 launch rmf_demos_gz office.launch.xml
```

### **8.2. Lanzar tarea Delivery**
```bash
ros2 run rmf_demos_tasks dispatch_delivery   -p pantry -ph coke_dispenser   -d hardware_2 -dh coke_ingestor   --use_sim_time
```

### **8.3. Validación**
- Verificar trayectorias en Gazebo.
- Revisar estados en dashboard.
- Registrar tópicos:
```bash
ros2 topic echo /task_states
```

---

## 9. Resultados Esperados

- Cero colisiones.
- Rutas óptimas según el scheduler.
- Manejo apropiado de errores.
- Resiliencia ante escenarios complejos (crowdsim, caddy, multitareas).

---

## 10. Conclusiones

El repositorio `rmf_demos` permite validar integralmente las capacidades de Open-RMF en ambientes simulados realistas.  
La estrategia aquí presentada constituye un marco profesional aplicable tanto para auditorías técnicas como para desarrollo académico o industrial.

---

## 11. Trabajo Futuro
- Añadir pruebas unitarias a fleet adapters.
- Extender pruebas adversariales (sensor noise, fallos de red).
- Añadir pipelines CI/CD para ejecución automática de tests.

---

**Autor:** _(Usuario)_  
**Versión:** 1.0  
