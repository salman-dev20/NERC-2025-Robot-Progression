# NERC 2025 Indigenous Category - Robot Algorithm Progression

This repository documents the evolution of our autonomous robot's pathfinding logic for the National Engineering Robotics Contest 2025. We iterated through three major algorithm designs to optimize for the fastest completion time.

## 📂 Project Structure

### 1. [v1_Basic_BFS](./v1_Basic_BFS) (Proof of Concept)
- **Algorithm:** Standard Breadth-First Search (BFS).
- **Goal:** Find the shortest path by distance (number of grid blocks).
- **Result:** The robot successfully found paths, but it ignored the cost of turning. It would often make zig-zag movements that were physically short but time-consuming due to mechanical turning delays.

### 2. [v2_Min_Turns_BFS](./v2_Min_Turns_BFS) (Mechanical Optimization)
- **Algorithm:** BFS modified to track turns.
- **Goal:** Minimize the number of turns to reduce mechanical delay.
- **Result:** This solved the zig-zagging issue. The robot took longer physical paths that were straighter. However, it sometimes took massive detours just to avoid a single turn, which ended up taking more time overall.

  ### 3. [v3_STPO_Optimized](./v3_STPO_Optimized) (Final Solution)
- **Algorithm:** Short-Time Path Optimization (STPO).
- **Goal:** Shortest **Time** pathing using Dijkstra on 3D States (x, y, direction).
- **Logic:** Tracks accumulated time, including:
    - Segment Time: ~1.0s (Time to travel 1 block)
    - Turn Penalty: ~2.5s (Time to turn 90°)
- **Result:** Calculates the mathematically fastest route based on exact robot physics.

## 🛠 Hardware
- **Controller:** Arduino Mega 2560
- **Sensors:** QTR-8A Line Array, IR Obstacle Sensors, TCS34725 Color Sensor
- **Actuators:** Custom H-Bridge Motor Drivers, Servo Gripper
