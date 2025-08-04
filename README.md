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
## Running the Simulation

To run the simulation:

```
python simulation.py
```

## Simulation Overview

The simulation runs a real-time interactive visualization using **pygame**, where:

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

## Research Context

This code supports our exploration of:
- Enforcing matrix inequality constraints via MCBFs.
- Integrating CBF-SDP with semidefinite constraints.
