# Particle Collision Simulation

## Overview
This project is a **C++ simulation of 2D circular particles** bouncing elastically inside a box.  
It demonstrates the use of **stacks and queues**:

- **Priority Queue (min-heap):** Manages future collision events (particle–wall and particle–particle).  
- **Stack (rollback/undo):** Supports restoring previous simulation states for debugging or speculative execution.  

The simulator is inspired by **event scheduling systems** used in physics engines, GPU task scheduling, and NVIDIA’s **Omniverse** physics/digital twin simulations.

---

## Features
- Deterministic.  
- Handles **elastic particle–particle collisions** and **particle–wall collisions**.  
- **Numerical safeguards**: invalidates stale events using collision counters.  
- Configurable simulation box size, time horizon, and number of particles.  
- Clean separation of simulation logic (`Simulator`) and vector math (`Vec2`).  


---

## Technologies Used
- **Language:** C++17 (works with GCC, Clang, or MSVC).  
- **Data Structures:** `std::priority_queue` (for events), `std::vector`, `std::stack` (for rollback).  
- **Math/Physics:** basic vector algebra, elastic collision equations.  

---
