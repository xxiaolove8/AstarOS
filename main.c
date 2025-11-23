#include "PathAPI.h"
#include <stdio.h>

int main(void)
{
    PathAPI_Init();
    PathAPI_ClearMap();

    PathAPI_AddObstacle(1, 1);
    PathAPI_AddObstacle(0, 2);
    PathAPI_AddObstacle(4, 1);

    Position start = {0, 0};
    Position end   = {4, 4};

    if (PathAPI_FindPath(start, end)) {
        char buf[256];
        PathAPI_GetPathString(buf, sizeof(buf));
        printf("Path: %s\r\n\r\n", buf);



        PathAPI_PrintPathAsGrid();
    } else {
        printf("No path found!\r\n");
        PathAPI_PrintPathAsGrid();
    }

    while (1) {

    }
}