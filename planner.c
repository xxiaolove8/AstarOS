//
// Created by XxiaoLove8 on 2025/12/7.
//

#include "planner.h"
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include "SerialWin.h"
#endif

#define ACK_TIMEOUT_MS 5000
// 理论上可以改成任意值：地图由 MAP_W / MAP_H 控制，车由 MAX_ROBOTS 控制
#define MAX_ROBOTS 3

typedef enum {
    DIR_UP = 0,    // y--
    DIR_RIGHT = 1, // x++
    DIR_DOWN = 2,  // y++
    DIR_LEFT = 3   // x--
} Heading;

typedef struct {
    int id;
    Position pos;
    Position goal;
    int arrived;
    Heading heading;    // 当前朝向

#ifdef _WIN32
    SerialPort port;    // 对应蓝牙串口
    const char *comName;
    int hasSerial;      // 1=串口已成功打开, 0=只仿真
#endif

} Robot;

static Robot g_robots[MAX_ROBOTS];
static int   g_robotCount = 3;

// 保存静态障碍（地图的墙）
static int g_staticObstacles[MAP_H][MAP_W];


static void Init_MapAndRobots(void);
static void Print_Grid_ASCII(void);
static void BuildDynamicMap(int selfIndex, int reserved[MAP_H][MAP_W]);
static Heading GetTargetHeading(Position from, Position to);
static void SendAction(Robot *rb, char action);
static void PlanOneStep(int index, int reserved[MAP_H][MAP_W]);
static void ApplyStaticObstaclesToPathAPI(void);

static int WaitForAckDone(Robot *rb)
{
#ifdef _WIN32
    if (!rb->hasSerial || !rb->port.h || rb->port.h == INVALID_HANDLE_VALUE) {
        // 没有串口，就直接当作收到 ACK（仿真用）
        return 1;
    }

    char ch = 0;
    if (!Serial_RecvByte(&rb->port, &ch, ACK_TIMEOUT_MS)) {
        printf("[WARN] Robot %d wait ACK timeout (no 'D' received)\n", rb->id);
        return 0;
    }

    if (ch != 'D') {
        printf("[WARN] Robot %d got unexpected byte '%c' (0x%02X) instead of 'D'\n",
               rb->id, ch, (unsigned char)ch);
        return 0;
    }

    // 正常收到 'D'
    return 1;
#else
    (void)rb;   // 非 Windows 平台，直接视为成功，防止未使用参数 warning
    return 1;
#endif
}



void Planner_Init(void)
{
    PathAPI_Init();
    Init_MapAndRobots();

#ifdef _WIN32
    // 打开串口
    for (int i = 0; i < g_robotCount; ++i) {
        Robot *rb = &g_robots[i];
        rb->hasSerial = 0;

        if (!Serial_Open(&rb->port, rb->comName, 9600)) {
            rb->hasSerial = 0;
            printf("Robot %d: FAILED to open %s, will only simulate.\n",
                   rb->id, rb->comName);
        } else {
            rb->hasSerial = 1;
            printf("Robot %d: opened %s (hasSerial=1)\n",
                   rb->id, rb->comName);
        }
    }
#endif
}

void Planner_PrintGrid(void)
{
    Print_Grid_ASCII();
}

int Planner_AllArrived(void)
{
    for (int i = 0; i < g_robotCount; ++i) {
        if (!g_robots[i].arrived) return 0;
    }
    return 1;
}

void Planner_StepOnce(void)
{
    int reserved[MAP_H][MAP_W] = {0};

    for (int i = 0; i < g_robotCount; ++i) {
        PlanOneStep(i, reserved);
    }
}

void Planner_Deinit(void)
{
#ifdef _WIN32
    for (int i = 0; i < g_robotCount; ++i) {
        Serial_Close(&g_robots[i].port);
    }
#endif
}





// ========== 初始化地图 + 机器人（你要求的方格网 + 位置） ==========
static void Init_MapAndRobots(void)
{
    PathAPI_Init();
    PathAPI_ClearMap();
    memset(g_staticObstacles, 0, sizeof(g_staticObstacles));

    // 中央障碍照旧...
    // ---------- 静态障碍：中央 2x2 方块 ----------
    // (3,3), (3,4), (4,3), (4,4)
    int obsX[] = {3, 6, 4, 4, 5, 3};
    int obsY[] = {3, 4, 3, 4, 5, 6};
    int obsCount = 6;
    for (int i = 0; i < obsCount; ++i) {
        int x = obsX[i];
        int y = obsY[i];
        g_staticObstacles[y][x] = 1;
        PathAPI_AddObstacle(x, y);
    }


    // Robot 0
    g_robots[0].id      = 0;
    g_robots[0].pos.x   = 0;
    g_robots[0].pos.y   = 0;
    g_robots[0].goal.x  = 7;
    g_robots[0].goal.y  = 7;
    g_robots[0].arrived = 0;
    g_robots[0].heading = DIR_RIGHT;
#ifdef _WIN32
    g_robots[0].hasSerial = 0;
    g_robots[0].comName = "COM4";   // 这里改成你蓝牙的串口号
#endif

    // Robot 1
    g_robots[1].id      = 1;
    g_robots[1].pos.x   = 7;
    g_robots[1].pos.y   = 0;
    g_robots[1].goal.x  = 0;
    g_robots[1].goal.y  = 6;
    g_robots[1].arrived = 0;
    g_robots[1].heading = DIR_LEFT;
#ifdef _WIN32
    g_robots[1].hasSerial = 0;
    g_robots[1].comName = "COM3";
#endif

    // Robot 2
    g_robots[2].id      = 2;
    g_robots[2].pos.x   = 0;
    g_robots[2].pos.y   = 7;
    g_robots[2].goal.x  = 6;
    g_robots[2].goal.y  = 0;
    g_robots[2].arrived = 0;
    g_robots[2].heading = DIR_UP;
#ifdef _WIN32
    g_robots[2].hasSerial = 0;
    g_robots[2].comName = "COM5";
#endif
}
static Heading GetTargetHeading(Position from, Position to)
{
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (dx == 1 && dy == 0)  return DIR_RIGHT;
    if (dx == -1 && dy == 0) return DIR_LEFT;
    if (dx == 0 && dy == 1)  return DIR_DOWN;
    if (dx == 0 && dy == -1) return DIR_UP;

    // 非相邻，异常情况：打印一条警告，方便你调试地图/路径
    fprintf(stderr,
            "[WARN] GetTargetHeading: from (%d,%d) to (%d,%d) not adjacent! "
            "dx=%d dy=%d\n",
            from.x, from.y, to.x, to.y, dx, dy);

    // 兜底：随便返回一个（不会导致崩溃，但你会在日志里看到问题）
    return DIR_UP;
}
// ========== ASCII 输出整张网格（地图 + 障碍 + 机器人） ==========
static void Print_Grid_ASCII(void)
{
    printf("Grid (%d x %d):\n", MAP_W, MAP_H);

    // 打印列号
    printf("   ");
    for (int x = 0; x < MAP_W; ++x) {
        printf("%d ", x);
    }
    printf("\n");

    for (int y = 0; y < MAP_H; ++y) {
        printf("%d: ", y);

        for (int x = 0; x < MAP_W; ++x) {
            char c = '.';

            // 静态障碍
            if (g_staticObstacles[y][x]) {
                c = '#';
            }

            // 机器人（覆盖障碍显示）
            for (int r = 0; r < g_robotCount; ++r) {
                if (!g_robots[r].arrived &&
                    g_robots[r].pos.x == x &&
                    g_robots[r].pos.y == y) {
                    c = (char)('0' + g_robots[r].id); // 0 / 1 / 2 ...
                    break;
                }
            }

            printf("%c ", c);
        }
        printf("\n");
    }
    printf("\n");
}
static void ApplyStaticObstaclesToPathAPI(void)
{
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            if (g_staticObstacles[y][x]) {
                PathAPI_AddObstacle(x, y);
            }
        }
    }
}
// ========== 构建“动态地图”：其他车 + 预定格子都视为障碍 ==========
static void BuildDynamicMap(int selfIndex, int reserved[MAP_H][MAP_W])
{
    // 先清空 PathAPI 内部地图
    PathAPI_ClearMap();

    // 1) 固定不变的静态障碍：墙、禁止通行区
    ApplyStaticObstaclesToPathAPI();

    // 2) 动态障碍：其他车当前所处的位置
    for (int i = 0; i < g_robotCount; ++i) {
        if (i == selfIndex) continue;
        if (g_robots[i].arrived) continue;

        int ox = g_robots[i].pos.x;
        int oy = g_robots[i].pos.y;

        // 特例：如果其他车刚好站在“我的终点”，最后一步要允许我走过去
        if (ox == g_robots[selfIndex].goal.x &&
            oy == g_robots[selfIndex].goal.y) {
            continue;
            }

        PathAPI_AddObstacle(ox, oy);
    }

    // 3) 本时间步已被“预定”的格子（避免两车同时冲同一格）
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            if (reserved[y][x]) {
                PathAPI_AddObstacle(x, y);
            }
        }
    }
}


// 对一个机器人，发送一个动作（可以扩展成发送序列）
static void SendAction(Robot *rb, char action)
{
#ifdef _WIN32
    if (rb->hasSerial && rb->port.h && rb->port.h != INVALID_HANDLE_VALUE) {
        printf("Send to robot %d [COM=%s]: %c\n", rb->id, rb->comName, action);
        Serial_SendByte(&rb->port, action);

        // 如果这一类动作需要等待 ACK（F/L/R），就在这里等 'D'
        if (action == 'F' || action == 'L' || action == 'R') {
            if (!WaitForAckDone(rb)) {   // 最长等 5 秒
                printf("[WARN] Robot %d: no ACK for action '%c'\n", rb->id, action);
            }
        }
    } else {
        // 只仿真，不发串口
        printf("Send to robot %d [SIM ONLY]: %c\n", rb->id, action);
    }
#else
    // 非 Windows 平台：纯仿真
    printf("Send to robot %d [SIM ONLY, no serial]: %c\n", rb->id, action);
#endif
}



// ========== 为某一辆车规划“一步” ==========
static void PlanOneStep(int index, int reserved[MAP_H][MAP_W])
{
    Robot *rb = &g_robots[index];

    // 已经到达，不再规划
    if (rb->arrived) return;

    // 如果已经在目标格，标记到达
    if (rb->pos.x == rb->goal.x && rb->pos.y == rb->goal.y) {
        rb->arrived = 1;
        printf("Robot %d already at goal (%d,%d)\n",
               rb->id, rb->goal.x, rb->goal.y);
        return;
    }

    // 1. 构建带动态障碍的地图
    BuildDynamicMap(index, reserved);

    // 2. 用 A* 求从当前位置到目标的整条路径
    if (!PathAPI_FindPath(rb->pos, rb->goal)) {
        // 找不到路，本周期等待
        printf("Robot %d: no path -> WAIT at (%d,%d)\n",
               rb->id, rb->pos.x, rb->pos.y);
        return;
    }

    size_t count = PathAPI_GetPathCount();
    if (count < 2) {
        // 起点 = 终点 或 异常
        printf("Robot %d: path too short -> WAIT\n", rb->id);
        return;
    }

    // 3. 路径中第 0 个是当前点，第 1 个是下一步要去的格子
    Position next = PathAPI_GetPathNode(1);

    // 越界防呆
    if (next.x < 0 || next.x >= MAP_W || next.y < 0 || next.y >= MAP_H) {
        printf("Robot %d: invalid next (%d,%d) -> WAIT\n",
               rb->id, next.x, next.y);
        return;
    }

    // 如果这个格子已经被其他机器人“预定”，本周期就等待
    if (reserved[next.y][next.x]) {
        printf("Robot %d: next (%d,%d) already reserved -> WAIT\n",
               rb->id, next.x, next.y);
        return;
    }

    // 4. 真正移动前，先算出动作
    Heading targetHeading = GetTargetHeading(rb->pos, next);
    int turn = ((int)targetHeading - (int)rb->heading + 4) % 4;

    // 先根据转向发送 L / R
    if (turn == 1) {          // 右转 90°
        SendAction(rb, 'R');
        rb->heading = (Heading)((rb->heading + 1) % 4);
    } else if (turn == 3) {   // 左转 90°
        SendAction(rb, 'L');
        rb->heading = (Heading)((rb->heading + 3) % 4);
    } else if (turn == 2) {   // 掉头
        SendAction(rb, 'R');
        SendAction(rb, 'R');
        rb->heading = (Heading)((rb->heading + 2) % 4);
    }
    // 然后前进一格
    SendAction(rb, 'F');

    // 仿真中直接把坐标跳到 next
    printf("Robot %d: (%d,%d) -> (%d,%d)\n",
           rb->id, rb->pos.x, rb->pos.y, next.x, next.y);
    rb->pos = next;
    reserved[next.y][next.x] = 1;

    // 5. 再次检查是否到达目标
    if (rb->pos.x == rb->goal.x && rb->pos.y == rb->goal.y) {
        rb->arrived = 1;
        printf("Robot %d reached goal!\n", rb->id);
    }
}