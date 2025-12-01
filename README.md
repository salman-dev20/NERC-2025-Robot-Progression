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

## 🛠 Hardware
- **Controller:** Arduino Mega 2560
- **Sensors:** QTR-8A Line Array, IR Obstacle Sensors, TCS34725 Color Sensor
- **Actuators:** Custom H-Bridge Motor Drivers, Servo Gripper
