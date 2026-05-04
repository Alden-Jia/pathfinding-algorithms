# Pathfinding Algorithms — D* Lite & LPA*

C++ implementation of two incremental heuristic search algorithms for robot path planning in dynamic grid environments, developed as part of the **Studies in Intelligent Systems (159.740)** course at Massey University.

## Algorithms

### D* Lite
An incremental replanning algorithm for autonomous robot navigation. The robot plans an initial path and continuously replans as it moves and discovers unknown obstacles within sensor range (1-cell radius).

### LPA* (Lifelong Planning A*)
An incremental version of A* that efficiently reuses previous search results when the environment changes, supporting both initial planning and full replanning after obstacle revelation.

## Features
- 8-connected grid world (N, NE, E, SE, S, SW, W, NW)
- Two heuristic functions: **Euclidean** and **Chebyshev** distance
- Real-time graphical visualisation using **SDL2**
- Displays g-values, rhs-values, and expanded nodes during search
- STL-based priority queue (min-heap) for the OPEN set
- Sensor simulation: robot detects unknown cells within 1-cell radius during navigation
- Performance tracking: state expansions, vertex accesses, max queue length, path length, running time
- 6 test grid worlds included

## Project Structure
```
pathfinding-algorithms/
├── DStar_lite/          # D* Lite implementation
│   ├── main.cpp
│   ├── dstar_lite.cpp / .h
│   ├── gridworld.cpp / .h
│   ├── graphics.cpp / .h
│   └── grids/           # Test map files
└── LPAStar/             # LPA* implementation
    ├── main.cpp
    ├── lpastar.cpp / .h
    ├── gridworld.cpp / .h
    ├── graphics.cpp / .h
    └── grids/           # Test map files
```
## Dependencies
- g++ 15.1 or later
- SDL2
- SDL_bgi

**macOS / Linux:** Install SDL2 and SDL_bgi before compiling (see installation notes in each folder).  
**Windows:** MinGW with SDL2 support required.

## Build & Run
```bash
cd DStar_lite
make
./search <gridworld_file> <heuristic>
```

## Author
Aowei Jia — Massey University, Auckland, New Zealand
