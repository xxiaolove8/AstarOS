#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "PathAPI.h"

static int obstacleMask[MAP_H][MAP_W];

static Position visited[256];
static int visitedCount = 0;

void RecordVisitedStep(Position p)
{
    if (visitedCount < 256)
        visited[visitedCount++] = p;
}


void PrintVisitedMap(void)
{
    char map[MAP_H][MAP_W];

    // init as '.'
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            map[y][x] = '.';

    // mark obstacles
    for (int y = 0; y < MAP_H; y++)
        for (int x = 0; x < MAP_W; x++)
            if (obstacleMask[y][x])
                map[y][x] = '#';

    // mark visited path
    for (int i = 0; i < visitedCount; i++)
        map[visited[i].y][visited[i].x] = 'V';

    // start & end
    map[0][0] = 'S';
    map[MAP_H - 1][MAP_W - 1] = 'E';

    printf("\n=== VISITED PATH MAP (S=start, E=end, V=visited, #=obstacle) ===\n");

    for (int y = 0; y < MAP_H; y++) {
        printf("y=%d: ", y);
        for (int x = 0; x < MAP_W; x++) {
            printf("%c ", map[y][x]);
        }
        printf("\n");
    }

    printf("    ");
    for (int x = 0; x < MAP_W; x++)
        printf("%d ", x);
    printf(" <- x\n\n");
}



// Parse the next step from: (x0,y0)->(x1,y1)->...
int GetNextStep(const char *pathStr, Position *out)
{
    int x0, y0, x1, y1;
    if (sscanf(pathStr, "(%d,%d)->(%d,%d)", &x0,&y0,&x1,&y1) == 4) {
        out->x = x1;
        out->y = y1;
        return 1;
    }
    return 0;
}



int main(void)
{
    srand((unsigned)time(NULL));

    PathAPI_Init();
    memset(obstacleMask, 0, sizeof(obstacleMask));

    Position cur = {0, 0};
    RecordVisitedStep(cur);
    Position end = {MAP_W - 1, MAP_H - 1};
    char buf[256];

    int added = 0;

    while (1) {

        // Try to find a path
        if (!PathAPI_FindPath(cur, end)) {
            printf("\n=== PATH FAILED ===\n");
            printf("Failed at position (%d,%d)\n", cur.x, cur.y);
            printf("\nCurrent map:\n");
            PathAPI_PrintPathAsGrid();
            return 0;
        }

        PathAPI_GetPathString(buf, sizeof(buf));

        // Reached goal
        if (cur.x == end.x && cur.y == end.y) {
            printf("\n=== SUCCESS ===\n");
            printf("Final path: %s\n\n", buf);
            PathAPI_PrintPathAsGrid();
            return 0;
        }

        // Move to next cell
        Position next;
        if (!GetNextStep(buf, &next)) {
            printf("\nError: Could not parse next step.\n");
            return 0;
        }
        cur = next;

        RecordVisitedStep(cur);

        // Add one random obstacle per step (max 10)
        if (added < 10) {
            Position obs;
            int ok = 0;

            for (int t = 0; t < 200; t++) {
                obs.x = rand() % MAP_W;
                obs.y = rand() % MAP_H;

                if (obstacleMask[obs.y][obs.x]) continue;
                if (obs.x == cur.x && obs.y == cur.y) continue;
                if (obs.x == end.x && obs.y == end.y) continue;

                ok = 1;
                break;
            }

            if (ok) {
                obstacleMask[obs.y][obs.x] = 1;
                PathAPI_AddObstacle(obs.x, obs.y);
                added++;
            }
        }

        if (added >= 10)
            break;
    }

    // Final result after 10 obstacles
    PathAPI_FindPath(cur, end);
    PathAPI_GetPathString(buf, sizeof(buf));

    printf("\n=== FINAL RESULT ===\n");
    printf("Position: (%d,%d)\n", cur.x, cur.y);
    printf("Final Path: %s\n\n", buf);
    PathAPI_PrintPathAsGrid();

    PrintVisitedMap();

    return 0;
}