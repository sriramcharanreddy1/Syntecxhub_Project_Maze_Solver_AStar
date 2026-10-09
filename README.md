# Maze Solver using A* Search

**Internship Task 1 — Syntecxhub**

## Overview
This project finds a shortest route through a 2D maze using the A* pathfinding algorithm. It models the maze as a grid, treats walls as blocked cells, uses Manhattan distance as the heuristic, and reports when the goal cannot be reached.

## Requirements
- Python 3.8 or later
- Optional: `matplotlib` for graphical visualization

## Run the project
```bash
python main.py
```

To open the graphical visualization:
```bash
pip install matplotlib
python main.py --visualize
```

## Maze symbols
- `S` — start cell
- `G` — goal cell
- `#` — wall
- `*` — shortest path
- `+` — explored cell
- `.` — open cell that was not explored

## How A* works
A* ranks each candidate cell using:

`f(n) = g(n) + h(n)`

- `g(n)` is the distance travelled from the start to the current cell.
- `h(n)` is the estimated distance from the current cell to the goal.
- This project uses Manhattan distance: `abs(row1 - row2) + abs(col1 - col2)`.

Movement is limited to up, down, left, and right, and every move has a cost of 1. With this heuristic, A* finds a shortest path when one exists.

## Customization
Edit `MAZE`, `START`, and `GOAL` near the top of `main.py`.
- `0` means an open cell.
- `1` means a wall.
- Coordinates use `(row, column)`, starting from zero.

Ensure start and goal are inside the grid and are not walls.

## Suggested GitHub repository name
`Syntecxhub_Project_Maze_Solver_AStar`

## Suggested internship submission checklist
- [ ] Run `python main.py` and confirm a path is found.
- [ ] Try a maze where the goal is blocked off to check the unreachable case.
- [ ] If desired, run `python main.py --visualize`.
- [ ] Upload `main.py` and `README.md` to the repository.
- [ ] Share the repository link in the internship submission form.
- [ ] Share your internship status on LinkedIn and mention **@Syntecxhub**, as instructed by the internship task.

## Complexity
Let `V` be the number of traversable cells and `E` the valid neighbor connections. A* uses a priority queue; typical graph-search complexity is `O((V + E) log V)` with this implementation's heap-based frontier, and memory use is `O(V)`.
