# Referential Communication in Evolved Agents

Code for the paper:

> **Analysis of Evolved Agents Performing Referential Communication**
> Santosh Manicka
> *Artificial Life XIII (ALIFE 2012)*

## Overview

This project evolves pairs of embodied agents (a Sender and a Receiver) that communicate referentially. The Sender observes a target location and must communicate its position to the Receiver through movement-based signals. The Receiver must then navigate to the target location based on the Sender's behavior.

### Agent Architecture

Each agent is controlled by a **continuous-time recurrent neural network (CTRNN)** with:
- **Distance sensors**: detect proximity to the other agent (left/right or CW/CCW)
- **Bearing sensors**: provide positional information (home vector for Receiver, target vector for Sender)
- **Interneurons**: recurrently connected hidden layer (configurable size, default 5)
- **Motor neurons**: two output neurons controlling movement in opposing directions

### World Configurations

The simulation supports two world geometries:
- **LINE**: Agents move on a 1D line with periodic boundary conditions (wrap-around)
- **CIRCLE**: Agents move on a circular arena

### Evolutionary Search

Agent pairs are co-evolved using a hill-climbing search algorithm (with optional genetic algorithm mode) to maximize the Receiver's accuracy in reaching the target location. Key features:
- Multi-threaded fitness evaluation (16 threads)
- Trials across multiple sender positions, agent separations, and target locations
- Sender can be spatially constrained to force referential (vs. direct guidance) communication
- Clamped-receiver mode: Receiver is stationary during information transmission, then navigates alone

## Building

```bash
make          # builds the evolution executable (Comm)
make Analysis # builds the analysis executable
make clean    # removes build artifacts
```

Requires: g++, pthreads

## Running

### Evolutionary Search

```bash
# Run an evolutionary search with a given random seed
./Comm <random_seed>

# Or use the convenience script (creates a run directory)
bash Run.sh <run_number> <random_seed>
```

The search outputs:
- `bestSender.ns` and `bestReceiver.ns`: serialized neural network parameters for the best evolved agents
- Periodic checkpoint files for resuming interrupted searches

### Analysis

The Analysis executable loads evolved agent parameters and records detailed trial-by-trial data (sensor values, neuron outputs, positions, velocities) for post-hoc analysis:

```bash
# Edit runNum in AnalysisMain.cpp to point to the desired run directory
./Analysis
```

Output CSV files contain per-timestep recordings of all agent variables for each trial.

## Configuration

Key parameters are defined in `src/globals.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `NUMINTS` | 5 | Number of interneurons per agent |
| `WorldLength` | 10.0 | Size of the linear arena |
| `MaxSensorDist` | WorldLength/16 | Maximum sensor detection range |
| `MaxLinearVelocity` | WorldLength/64 | Maximum movement speed |
| `PopulationSize` | 450 | Evolutionary population size |
| `MaxGenerations` | 20000 | Maximum evolutionary generations |
| `CONSTRAIN_SENDER` | true | Whether to restrict sender movement |
| `CLAMP_RECEIVER` | true | Whether receiver is stationary during info transmission |
| `TARGET_ADDRESS_ONLY` | true | Whether sender gets static target info |
| `DISTINCT_AGENTS` | true | Whether sender and receiver have separate neural networks |

## Code Structure

```
src/
  main.cpp              - Evolution main: sets up search, evaluates agent pairs
  AnalysisMain.cpp      - Analysis main: loads evolved agents, records trial data
  CommAgent1D.h/.cpp    - Agent classes (CommAgent1DCircle, CommAgent1DLine)
  SensorCTRNN.h/.cpp    - CTRNN with weighted sensor inputs
  CTRNN.h/.cpp          - Base continuous-time recurrent neural network
  SensorLoadCircuit.h   - Utilities for mapping search vectors to CTRNN parameters
  Search.h/.cpp         - Evolutionary search engine (hill-climbing, GA)
  VectorMatrix.h        - Template vector and matrix classes
  random.h/.cpp         - Random number generation (Numerical Recipes)
  globals.h             - Global simulation parameters and configuration
  tree.hh               - Tree data structure for lineage tracking
Run.sh                  - Convenience script for launching runs
Makefile                - Build configuration
```

## Citation

```bibtex
@inproceedings{manicka2012analysis,
  title={Analysis of Evolved Agents Performing Referential Communication},
  author={Manicka, Santosh},
  booktitle={Artificial Life XIII: Proceedings of the Thirteenth International Conference on the Simulation and Synthesis of Living Systems},
  year={2012}
}
```
