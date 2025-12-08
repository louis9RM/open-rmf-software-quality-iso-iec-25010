# RMF Demos

The Open Robotics Middleware Framework (Open-RMF) enables interoperability among heterogeneous robot fleets while managing shared resources such as space, lifts, doors and other infrastructure. It also provides task allocation and conflict resolution across multiple robot fleets. This repository contains demonstration environments showcasing these capabilities.

---

# Project Structure

The repository is organized into the following directories:

| Directory | Description |
|----------|-------------|
| **rmf_demos_gz/** | Simulation launch configurations for the various demonstration worlds. |
| **rmf_demos_maps/** | Maps created using *traffic_editor* for each demo scenario. |
| **rmf_demos_tasks/** | Scripts that define high-level task types such as delivery, loop, patrol and cleaning. |
| **rmf_demos_assets/** | Models and resources used within the environments. |
| **rmf_demos_bridges/** | Bridges for integration, such as MQTT interfaces for external systems. |
| **docs/** | Additional documentation including FAQs and secure-mode configurations. |

---

# Demo Worlds

Below is a concise description of each demonstration world included in RMF Demos.

---

## Hotel World

A multi-level hotel environment with a lobby and guest floors. It demonstrates coordination of multiple robot fleets using lifts and indoor navigation.

---

## Office World

An indoor office layout that includes controllable doors, laneways and stations for simple logistics tasks. It demonstrates navigation, resource sharing and multi-robot coordination.

---

## Airport Terminal World

A large-scale terminal with numerous lanes and destinations. It includes optional crowd simulation and a manually operated vehicle treated as a non-autonomous actor, showcasing dynamic avoidance and replanning.

---

## Clinic World

A two-floor clinical facility equipped with lifts and multiple fleets. This world highlights coordinated navigation using vertical transport systems and cross-fleet interactions.

---

## Campus World

A large outdoor environment annotated using GPS WGS84 coordinates. It demonstrates long-range navigation, fleet tracking and operation across wide areas.

---

## Manufacturing & Logistics World

An industrial setting combining mobile robots, conveyors and fixed manipulators, illustrating workflow interoperability in automated logistics environments.
