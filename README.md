PathAPI – A* Grid Path Planning

PathAPI provides a simple interface for running A* pathfinding on a 2D grid of size MAP_W × MAP_H.
{
0 = walkable cell
1 = obstacle
}

The module uses the general A* implementation in AStar.c / AStar.h.

==================================================================================================

Features:

2D grid-based pathfinding

4-directional movement (up/down/left/right)

Manhattan-distance heuristic

Configurable and lightweight

Simple ASCII grid visualization for debugging

===================================================================================================

Data Types
Position

Integer grid coordinate:

typedef struct {
    int x;   // column index  [0 .. MAP_W-1]
    int y;   // row index     [0 .. MAP_H-1]
} Position;

====================================================================================================
Basic Usage:


PathAPI_Init();
PathAPI_ClearMap();

// Mark obstacles
PathAPI_AddObstacle(3, 4);
PathAPI_AddObstacle(3, 5);

Position start = {0, 0};
Position end   = {10, 10};

if (PathAPI_FindPath(start, end)) {
char buf[128];
PathAPI_GetPathString(buf, sizeof(buf));
printf("path=%s\n", buf);
}

PathAPI_PrintPathAsGrid();


=======================================================================================================


API Reference:

void PathAPI_Init(void);
Initializes the module.
Currently, clears the map; reserved for future extensions such as loading a saved map.

void PathAPI_ClearMap(void);
Clears the internal grid:
Sets all cells to walkable (0)
Clears the cached path
Use when resetting the environment or starting a new search.

int PathAPI_AddObstacle(int x, int y);
Adds a blocked cell.
Returns 1 on success
Returns 0 if (x, y) is out of bounds
Internally sets g_map[y][x] = 1.

int PathAPI_FindPath(Position start, Position end);
Runs A* from start to end.
Returns 1 if a path is found
Returns 0 if no valid path exists
The search uses:

Manhattan heuristic
Four neighbors per node
Comparator: first y, then x
Results are stored in an internal g_pathNodes[] array.

int PathAPI_GetPathString(char *outBuf, size_t outBufSize);
Writes the most recent path into the provided buffer.
Format example:
(0,0)->(1,0)->(2,0)

returns 1 if a valid path exists
Returns 0 and writes "NO PATH" otherwise.


void PathAPI_PrintPathAsGrid(void)
Prints the map and the most recent path as text.

Example output:

Grid (W=20, H=20):
S=start, E=end, O=path, #=obstacle, .=free
y=0: S . # . .
y=1: O . # . .
...
0 1 2 3 4  <- x

Useful for debugging via terminal or microcontroller serial output.