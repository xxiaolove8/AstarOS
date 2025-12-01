//
// Created by 29465 on 2025/11/23.
//

#include "PathAPI.h"
#include "AStar.h"
#include <stdio.h>   // printf / snprintf
#include <string.h>  // memset, memcpy

// =================== 地图与路径数据 ===================

// 0 = 空地, 1 = 障碍
static int g_map[MAP_H][MAP_W] = {0};

static Position g_start;
static Position g_end;

// 保存最近一次规划出的路径
static Position g_pathNodes[MAP_W * MAP_H];
static size_t   g_pathCount = 0;

// =================== A* 所需回调 ===================

static int Map_IsFree(int x, int y)
{
    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) return 0;
    return g_map[y][x] == 0;
}

// 邻居（4方向）
static void GridNodeNeighbors(ASNeighborList neighbors, void *node, void *context)
{
    Position *cur = (Position *)node;

    static const int dx[4] = { 1, -1, 0, 0 };
    static const int dy[4] = { 0, 0, 1,-1 };

    for (int i = 0; i < 4; i++) {
        int nx = cur->x + dx[i];
        int ny = cur->y + dy[i];

        if (Map_IsFree(nx, ny)) {
            Position nb = { nx, ny };
            ASNeighborListAdd(neighbors, &nb, 1.0f);  // 每步代价=1
        }
    }
}

// 曼哈顿启发
static float GridPathCostHeuristic(void *fromNode, void *toNode, void *context)
{
    Position *a = (Position *)fromNode;
    Position *b = (Position *)toNode;

    int dx = (a->x > b->x) ? (a->x - b->x) : (b->x - a->x);
    int dy = (a->y > b->y) ? (a->y - b->y) : (b->y - a->y);
    return (float)(dx + dy);
}

// 节点比较（按 y 再按 x）
static int GridNodeComparator(void *node1, void *node2, void *context)
{
    Position *a = (Position *)node1;
    Position *b = (Position *)node2;

    if (a->y < b->y) return -1;
    if (a->y > b->y) return  1;
    if (a->x < b->x) return -1;
    if (a->x > b->x) return  1;
    return 0;
}

// =================== 对外 API 实现 ===================

void PathAPI_Init(void)
{
    PathAPI_ClearMap();
}

void PathAPI_ClearMap(void)
{
    memset(g_map, 0, sizeof(g_map));
    g_pathCount = 0;
}

int PathAPI_AddObstacle(int x, int y)
{
    if (x < 0 || x >= MAP_W || y < 0 || y >= MAP_H) {
        return 0;
    }
    g_map[y][x] = 1;
    return 1;
}

int PathAPI_FindPath(Position start, Position end)
{
    g_start = start;
    g_end   = end;

    ASPathNodeSource source;
    source.nodeSize          = sizeof(Position);
    source.nodeNeighbors     = GridNodeNeighbors;
    source.pathCostHeuristic = GridPathCostHeuristic;
    source.earlyExit         = NULL;
    source.nodeComparator    = GridNodeComparator;

    ASPath path = ASPathCreate(&source, NULL, &start, &end);
    size_t count = ASPathGetCount(path);

    if (count == 0) {
        // 无路可走
        ASPathDestroy(path);
        g_pathCount = 0;
        return 0;
    }

    // 把节点拷贝到本地数组，方便后续打印和转换为字符串
    if (count > MAP_W * MAP_H) count = MAP_W * MAP_H;

    for (size_t i = 0; i < count; i++) {
        Position *p = (Position *)ASPathGetNode(path, i);
        g_pathNodes[i] = *p;
    }
    g_pathCount = count;

    ASPathDestroy(path);
    return 1;
}

int PathAPI_GetPathString(char *outBuf, size_t outBufSize)
{
    if (outBuf == NULL || outBufSize == 0) return 0;

    if (g_pathCount == 0) {
        snprintf(outBuf, outBufSize, "NO PATH");
        return 0;
    }

    size_t offset = 0;
    outBuf[0] = '\0';

    for (size_t i = 0; i < g_pathCount; i++) {
        Position *p = &g_pathNodes[i];

        int n = snprintf(outBuf + offset,
                         (offset < outBufSize) ? (outBufSize - offset) : 0,
                         "(%d,%d)", p->x, p->y);
        if (n < 0) break;
        offset += (size_t)n;

        if (i != g_pathCount - 1) {
            n = snprintf(outBuf + offset,
                         (offset < outBufSize) ? (outBufSize - offset) : 0,
                         "->");
            if (n < 0) break;
            offset += (size_t)n;
        }

        if (offset >= outBufSize) {
            outBuf[outBufSize - 1] = '\0';
            break;
        }
    }

    return 1;
}

static int IsOnPath(int x, int y)
{
    for (size_t i = 0; i < g_pathCount; i++) {
        if (g_pathNodes[i].x == x && g_pathNodes[i].y == y) {
            return 1;
        }
    }
    return 0;
}

void PathAPI_PrintPathAsGrid(void)
{




    printf("Grid (W=%d, H=%d):\r\n", MAP_W, MAP_H);
    printf("S=start, E=end, O=path, #=obstacle, .=free\r\n");

    for (int y = 0; y < MAP_H; y++) {
        printf("y=%d: ", y);
        for (int x = 0; x < MAP_W; x++) {

            char c = '.';

            if (g_map[y][x] == 1) {
                c = '#';              // 障碍
            }

            if (IsOnPath(x, y)) {
                c = 'O';              // 在路径上
            }

            if (x == g_start.x && y == g_start.y) {
                c = 'S';              // 起点
            } else if (x == g_end.x && y == g_end.y) {
                c = 'E';              // 终点
            }

            printf("%c ", c);
        }
        printf("\r\n");
    }


    printf("     ");
    for (int x = 0; x < MAP_W; x++) {
        printf("%d ", x);
    }
    printf("<- x\r\n");
}

// =================== 路径访问辅助函数（多车调度用） ===================

// 返回最近一次 PathAPI_FindPath 计算出的路径节点数
size_t PathAPI_GetPathCount(void)
{
    return g_pathCount;
}

// 返回最近一次路径中的第 index 个节点（0 = 起点）
// 若 index 越界，则返回 (-1, -1)
Position PathAPI_GetPathNode(size_t index)
{
    Position invalid;
    invalid.x = -1;
    invalid.y = -1;

    if (index >= g_pathCount) {
        return invalid;
    }
    return g_pathNodes[index];
}