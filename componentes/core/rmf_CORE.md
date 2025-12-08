# RMF Demos

![](https://github.com/open-rmf/rmf_demos/workflows/build/badge.svg)
![](https://github.com/open-rmf/rmf_demos/workflows/style/badge.svg)

The Open Robotics Middleware Framework (Open-RMF) enables interoperability among heterogeneous robot fleets while managing robot traffic that share resources such as space, building infrastructure systems (lifts, doors, etc) and other automation systems within the same facility. Open-RMF also handles task allocation and conflict resolution among its participants. These capabilities are provided by various libraries in [Open-RMF](https://github.com/open-rmf/rmf).

For more details about Open-RMF, refer to the documentation provided [here](https://osrf.github.io/ros2multirobotbook/intro.html).

This repository contains demonstrations of the above capabilities of Open-RMF. It serves as a starting point for working and integrating with Open-RMF.

You can also find a demonstration using `Nav2` and `MoveIt!` in the [Ionic Release Demo](https://github.com/gazebosim/ionic_demo).

[![Robotics Middleware Framework](../media/thumbnail.png?raw=true)](https://vimeo.com/405803151)

#### (Click to watch video)

---

# Project Structure

The repository is organized into the following main components:

| Directory | Description |
|----------|-------------|
| **rmf_demos_gz/** | Simulation launch files (`*.launch.xml`) for Gazebo/Ignition worlds. |
| **rmf_demos_maps/** | Maps created using *traffic_editor* for each demonstration world. |
| **rmf_demos_tasks/** | Task scripts for different behaviors such as delivery, loop, patrol, cleaning, etc. |
| **rmf_demos_assets/** | 3D models and assets used across the demo environments. |
| **rmf_demos_bridges/** | Interfaces such as MQTT bridges for integration with external systems. |
| **docs/** | Additional documentation, including secure world configurations and FAQs. |

---

# Demo Worlds

Below are the available simulation worlds included in this repository.  
**All original images and media references are preserved exactly as in the source README.**

---

## Hotel World

This hotel world consists of a lobby and 2 guest levels. The environment includes two lifts, multiple doors and three robot fleets. It demonstrates multi-level navigation and interoperability among heterogeneous robots.

![](../media/hotel_world.png)

Robots performing loop and clean tasks:

![](../media/hotel_scenarios.gif)

---

## Office World

An indoor office environment integrating Open-RMF features such as:

- beverage dispensing station  
- controllable doors  
- laneways and indoor navigation  

![](../media/delivery_request.gif?raw=true)

![](../media/loop_request.gif)

---

## Airport Terminal World

A large-scale terminal with many lanes, destinations and robot interactions.  
Includes Crowd Simulation and a manually operated `read_only` vehicle (Caddy).

![](../media/airport_terminal_traffic_editor_screenshot.png)
![](../media/airport_terminal_demo_screenshot.png)

Caddy vehicle demonstration:

![](../media/caddy.gif)

---

## Clinic World

A two-level clinic environment with two lifts and multiple robot fleets.  
Robots navigate between floors using lift coordination.

![](../media/clinic.png)

Robots using the lift:

![](../media/robot_taking_lift.gif)

Multi-fleet scenario:

![](../media/clinic.gif)

---

## Campus World

A large-scale outdoor world using GPS WGS84 coordinates for robot navigation.  
Demonstrates long-range delivery and fleet coordination across a campus.

![](../media/campus.gif)

---

## Manufacturing & Logistics World

A simulation showcasing interoperability in industrial workflows, including manipulators, conveyors and multiple AMR fleets.

<p align="center">
<a href="https://www.youtube.com/watch?v=oSVQrjx_4w4">
<img src="https://img.youtube.com/vi/oSVQrjx_4w4/0.jpg">
</a>
</p>

---

This README summarizes the core structure and demonstration worlds provided in the RMF Demos repository, preserving all contextual descriptions and original media references.
