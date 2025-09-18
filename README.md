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

## 🚀 Getting Started

### 1. Clone & build
```bash
git clone https://github.com/YOUR_USERNAME/physics-simulator.git
cd physics-simulator
g++ -std=c++17 -O2 -Wall physics.cpp -o physics
```
### 2. Run
```bash
./physics
```
### 2. Sample Output:
Final Time: 12.0000
P0 r=(3.0000,7.8000) v=(-1.2000,-0.8000) collisions=4
P1 r=(6.1000,1.5000) v=(0.9000,0.6000) collisions=4
P2 r=(3.0000,2.6000) v=(-0.4000,-1.1000) collisions=1
