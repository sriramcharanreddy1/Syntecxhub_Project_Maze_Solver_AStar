"""
Maze Solver using A* Search
Syntecxhub Internship - Task 1

Run:
    python main.py

Optional visualization:
    pip install matplotlib
    python main.py --visualize
"""

import argparse
import heapq
from itertools import count

# 0 = open cell, 1 = wall
MAZE = [
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [1, 1, 0, 0, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 1, 1, 1, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    [0, 1, 1, 1, 1, 0, 0, 1, 0, 1],
    [0, 0, 0, 0, 1, 0, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
]

START = (0, 0)
GOAL = (9, 9)


def manhattan_distance(a, b):
    """Estimate distance between two cells when movement is up/down/left/right."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(maze, cell):
    """Return valid open neighbors in four directions."""
    rows, cols = len(maze), len(maze[0])
    row, col = cell
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = row + dr, col + dc
        if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] == 0:
            yield (nr, nc)


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(maze, start, goal):
    """
    A* search using f(n) = g(n) + h(n).
    Returns (path, explored), where path is a list of cells or [] if unreachable.
    """
    if not maze or not maze[0]:
        return [], set()

    rows, cols = len(maze), len(maze[0])
    for name, point in (("start", start), ("goal", goal)):
        r, c = point
        if not (0 <= r < rows and 0 <= c < cols):
            raise ValueError(f"{name.capitalize()} position {point} is outside the maze.")
        if maze[r][c] == 1:
            raise ValueError(f"{name.capitalize()} position {point} is a wall.")

    sequence = count()
    open_heap = [(manhattan_distance(start, goal), 0, next(sequence), start)]
    came_from = {}
    g_score = {start: 0}
    explored = set()

    while open_heap:
        _, current_g, _, current = heapq.heappop(open_heap)

        # Ignore outdated heap entries.
        if current_g != g_score.get(current):
            continue

        if current == goal:
            return reconstruct_path(came_from, current), explored

        explored.add(current)

        for neighbor in get_neighbors(maze, current):
            tentative_g = current_g + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan_distance(neighbor, goal)
                heapq.heappush(
                    open_heap, (f_score, tentative_g, next(sequence), neighbor)
                )

    return [], explored


def print_maze(maze, start, goal, path, explored):
    """Print a readable text map of the maze."""
    path_set = set(path)
    for r, row in enumerate(maze):
        symbols = []
        for c, value in enumerate(row):
            cell = (r, c)
            if cell == start:
                symbol = "S"
            elif cell == goal:
                symbol = "G"
            elif cell in path_set:
                symbol = "*"
            elif value == 1:
                symbol = "#"
            elif cell in explored:
                symbol = "+"
            else:
                symbol = "."
            symbols.append(symbol)
        print(" ".join(symbols))
    print("\nLegend: S = Start | G = Goal | # = Wall | * = Shortest path")
    print("        + = Explored cell | . = Open cell")


def visualize_maze(maze, start, goal, path, explored):
    """Optional matplotlib visualization."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError:
        print("\nVisualization requires matplotlib. Install it with: pip install matplotlib")
        return

    rows, cols = len(maze), len(maze[0])
    # 0 open, 1 wall, 2 explored, 3 path, 4 start, 5 goal
    display = [[1 if maze[r][c] else 0 for c in range(cols)] for r in range(rows)]
    for r, c in explored:
        if display[r][c] == 0:
            display[r][c] = 2
    for r, c in path:
        display[r][c] = 3
    display[start[0]][start[1]] = 4
    display[goal[0]][goal[1]] = 5

    colors = ["white", "black", "lightskyblue", "gold", "limegreen", "tomato"]
    cmap = ListedColormap(colors)
    plt.figure(figsize=(8, 7))
    plt.imshow(display, cmap=cmap, vmin=0, vmax=5)
    plt.xticks(range(cols))
    plt.yticks(range(rows))
    plt.grid(which="major", color="gray", linewidth=0.5)
    plt.title("Maze Solver using A* Search")
    plt.xlabel("Column")
    plt.ylabel("Row")
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Solve a grid maze using A* Search.")
    parser.add_argument(
        "--visualize", action="store_true",
        help="show a graphical maze visualization (requires matplotlib)"
    )
    args = parser.parse_args()

    path, explored = astar(MAZE, START, GOAL)
    print("Maze Solver using A* Search\n")
    print_maze(MAZE, START, GOAL, path, explored)

    if path:
        print(f"\nPath found! Number of moves: {len(path) - 1}")
        print(f"Cells in path: {path}")
    else:
        print("\nNo path exists between the start and goal.")

    print(f"Explored cells: {len(explored)}")
    if args.visualize:
        visualize_maze(MAZE, START, GOAL, path, explored)


if __name__ == "__main__":
    main()
