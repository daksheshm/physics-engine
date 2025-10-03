
# Python 2D Physics Engine

A 2D rigid body physics engine built from scratch in Python, utilizing **Pygame** for rendering and **NumPy** for vector operations. This engine simulates physical interactions between rigid bodies, including collision detection, impulse-based resolution, and friction.

The purpose of this engine was to allow me to use realistic physics to create an AngryBirds like game as a part of my Course Project for CS104 (Software Systems Lab)

## Features

-   **Rigid Body Dynamics**: Simulates the motion of objects based on physical properties such as mass, moment of inertia, linear velocity, and angular velocity.
-   **Collision Detection**:
    -   Implements the **Separating Axis Theorem (SAT)** for robust collision detection between rotated rectangles (Oriented Bounding Boxes).
    -   Supports multiple shape combinations:
        -   Rectangle vs. Rectangle
        -   Circle vs. Circle
        -   Circle vs. Rectangle
-   **Collision Response**:
    -   Utilizes an **iterative impulse-based solver** for stable and realistic collision resolution.
    -   Accurately models rotational effects by applying impulses at the calculated points of contact.
    -   Includes a coefficient of restitution (bounciness) to control post-collision energy.
-   **Friction Model**: Implements both **static and dynamic friction** to simulate resistance between surfaces.
-   **Static Bodies**: Supports immovable objects with infinite mass and inertia, suitable for creating boundaries and static platforms.

## Getting Started

### Prerequisites

-   Python 3.x
-   Pygame
-   NumPy

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/your-repository-name.git
    cd your-repository-name
    ```

2.  **Install dependencies:**
    ```bash
    pip install pygame numpy
    ```

### How to Run

Execute the `game.py` script to start the simulation:

```bash
python game.py
```

## Controls

-   **Left Mouse Click**: Spawn a dynamic **rectangle** at the cursor's position.
-   **Right Mouse Click**: Spawn a dynamic **circle** at the cursor's position.
-   **Arrow Keys**: Apply continuous linear force to the first spawned dynamic object.
-   **'U' Key**: Apply continuous positive torque (rotation) to the first spawned dynamic object.

## Code Structure

The engine's architecture is modular, with responsibilities separated into distinct files:

-   `game.py`
    -   **Purpose**: Main application entry point.
    -   **Responsibilities**: Handles Pygame initialization, the main game loop, user input processing, and rendering of all bodies.

-   `environment.py`
    -   **Purpose**: The core physics world that manages all simulation objects and steps.
    -   **Responsibilities**:
        -   Maintains a list of all `Body` objects.
        -   Executes the main physics update loop (`processPhysics`), which includes broad-phase and narrow-phase collision detection.
        -   Orchestrates collision response by calculating and applying positional correction, impulses, and friction.
        -   Applies global forces like gravity.

-   `body.py`
    -   **Purpose**: Defines the `Body` class, representing a single rigid body.
    -   **Responsibilities**:
        -   Stores physical properties: `mass`, `inertia`, `center` (position), `angle`, `linearVelocity`, `angularVelocity`, etc.
        -   Manages state for static vs. dynamic bodies.
        -   Provides utility methods like `getVertices()` to compute world-space coordinates for a rectangle's vertices.

-   `collision.py`
    -   **Purpose**: Contains all algorithms for collision detection and contact point generation.
    -   **Responsibilities**: Implements geometric tests (e.g., SAT) to determine if two bodies are intersecting. Calculates the collision normal and penetration depth, and finds the precise contact points needed for the solver.

-   `contact.py`
    -   **Purpose**: Defines a simple data structure for storing collision information.
    -   **Responsibilities**: Aggregates all relevant data for a single collision event (the two bodies, normal, depth, and contact points) into a single object for clean and organized data transfer between modules.

## Future Improvements

-   **Performance Optimization**: Add AABB (axis aligned bounding boxes) to improve speed of the physics engine (the current implementation becomes laggy for a large number of objects)
-   **Advanced Shapes**: Add support for complex convex polygons, capsules, and other geometric primitives.
