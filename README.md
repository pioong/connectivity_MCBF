# Connectivity Maintenance Matrix Control Barrier Functions

This repository illustrates how matrix control barrier functions (MCBFs) can be used to handle **semidefinite matrix constraints**, using a connectivity maintenance problem in multi-agent systems as a case study.

We implement a **CBF-SDP** controller to maintain connectivity in a dynamic graph, where each agent is modeled as a 2D **single integrator**. The approach ensures the graph Laplacian (ignoring the 1 subspace) remains a positive definite matrix, using a semidefinite constraint.

## Features

- MCBF-based connectivity maintenance
- Formulation of MCBF semidefinite constraints
- Simple single-integrator dynamics
- Python-based simulation with clear structure for extension

## Getting Started
Create and activate a virtual environment
```
python -m venv venv
source venv/bin/activate
```
Install dependencies via:

```bash
pip install -r requirements.txt
```
## Using the code

To run the game:

```
python run_game.py
```
To run the simulation:

```
python simulation.py
```

## Game Overview

The game runs a real-time interactive visualization using **pygame**, where:

- Each agent has a fixed **desired destination**.
- The **nominal controller** is a **proportional controller** to the agent’s destination.
- **CBFs** are used to:
  - Maintain **connectivity** of the dynamic graph (via SDP formulation).
  - Prevent **collisions** between agents.
- An additional input constraint enforces the **prioritized agent**  to use its desired control, ensuring progress towards its destination.

### Interactivity

- 🖱️ **Click anywhere** to update the destination of the **closest agent**.
- This agent becomes the **prioritized agent**, marked in **red**.
- Non-prioritized agents are shown in blue.

### ⚙️ Performance

The speed of the game depends on how fast your computer can solve the underlying optimization problems in real time. If the simulation feels too slow or sluggish, you can adjust the time step `dt` in `drone.py` to reduce the frequency of updates.

> **Note:** Increasing `dt` too much can cause **chattering** in the control inputs — particularly when both **connectivity** and **collision avoidance** constraints are active. This occurs due to **sample-and-hold effects** in the discrete-time controller.

With reasonable values of `dt`, despite possible chattering, the system should still:
- **Maintain connectivity**
- **Avoid collisions**

This makes the simulation behaviorally accurate even under less-than-ideal timing conditions.

## 🧪 Simulation Overview

The simulation models **5 drones**:

- **1 center drone** starts at the origin and moves back and forth between the **origin** and the **bottom-right corner**.
- **4 corner drones** are initialized in each corner and use a **proportional controller** to return to their original positions.

All drones are subject to a **CBF-SDP controller** that:
- Prevents **collisions**
- Maintains **connectivity**

As a result, the drones may temporarily move away from their nominal targets to satisfy safety constraints.

After the animation finishes, the simulation generates plots showing:
- 📍 **Position** of each drone over time
- 🌀 **Velocity** of each drone
- 📈 **Eigenvalues** of the graph Laplacian, to illustrate connectivity maintenance

## Research Context

This code supports our exploration of:
- Enforcing matrix inequality constraints via MCBFs.
- Integrating CBF-SDP with semidefinite constraints.


## 📄 Related Paper

This repository complements the following paper:

**Matrix Control Barrier Functions**  
Pio Ong, Yicheng Xu, Ryan M. Bena, Faryar Jabbari, Aaron D. Ames.  
Submitted to **IEEE Transactions on Automatic Control (TAC)**, 2025.

> If you use this code or build on it, please consider citing the paper.


## 🙏 Acknowledgments

This work was supported in part by the Technology Innovation Institute (TII).  
We would like to thank Carlos Llamas of the Technology Innovation Institute (TII) for his contributions to the development of the Pygame environment.
