/******************************************************************************
 *
 *        	     /\     佛祖保佑           |\_/|
 *              /  \    代码无BUG          |^_^|
 *       	   	 /|||\    阿弥陀佛          /     \
 *      	    _|||||_   法力无边        _/       \_
 *
 *                       _oo0oo_
 *                      o8888888o
 *                      88" . "88
 *                      (| -_- |)
 *                      0\  =  /0
 *                    ___/`---'\___
 *                  .' \\|     |// '.
 *                 / \\|||  :  |||// \
 *                / _||||| -:- |||||- \
 *               |   | \\\  -  /// |   |
 *               | \_|  ''\---/''  |_/ |
 *               \  .-\__  '-'  ___/-. /
 *             ___'. .'  /--.--\  `. .'___
 *          ."" '<  `.___\_<|>_/___.' >' "".
 *         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *         \  \ `_.   \_ __\ /__ _/   .-` /  /
 *     =====`-.____`.___ \_____/___.-`___.-'=====
 *                       `=---='
 *
 *     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *         如来保佑     永无BUG        法力无边
 *
 *****************************************************************************/


#include "PathAPI.h"
#include <stdio.h>


#include <string.h>
#include "PathAPI.h"
#ifdef _WIN32
#include "SerialWin.h"
#endif
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
    g_robots[0].comName = "COM3";   // 这里改成你蓝牙的串口号
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
    g_robots[0].hasSerial = 0;
    g_robots[1].comName = "COM4";
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
    g_robots[0].hasSerial = 0;
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
    } else {
        // 串口没打开的情况：只在仿真里动，但也打印出来方便你看
        printf("Send to robot %d [SIM ONLY, no COM]: %c\n", rb->id, action);
    }
#else
    // 非 Windows 平台：纯仿真
    printf("Send to robot %d [SIM ONLY, no serial]: %c\n", rb->id, action);
#endif
}

#ifdef _WIN32
// 简单扫描 COM1..COM20，看看哪些端口能打开（调试用）
static void Debug_ScanCOMPorts(void)
{
    printf("=== Debug: scanning COM1..COM20 ===\n");
    for (int i = 1; i <= 20; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "COM%d", i);

        SerialPort p;
        p.h = NULL;

        if (Serial_Open(&p, name, 9600)) {
            printf("[PORT OK]   %s can be opened.\n", name);
            Serial_Close(&p);
        } else {
            printf("[PORT FAIL] %s cannot be opened.\n", name);
        }
    }
    printf("===================================\n\n");
}
#endif


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


// ========== 主仿真入口 ==========
int main(void)
{
#ifdef _WIN32
    Debug_ScanCOMPorts();  // 先看看当前机器上哪些 COM 能用
#endif
    Init_MapAndRobots();
#ifdef _WIN32
    for (int i = 0; i < g_robotCount; ++i) {
        if (!Serial_Open(&g_robots[i].port, g_robots[i].comName, 9600)) {
            printf("Robot %d: failed to open %s, will only simulate.\n",
                   g_robots[i].id, g_robots[i].comName);
        } else {
            printf("Robot %d: opened %s\n", g_robots[i].id, g_robots[i].comName);
        }
    }
#endif
    printf("Multi-robot A* demo on %dx%d grid, robots=%d\n\n",
           MAP_W, MAP_H, g_robotCount);

    int step = 0;

    while (1) {
        printf("======= STEP %d =======\n", step);
        Print_Grid_ASCII();

        // 检查是否所有车都已到达
        int allDone = 1;
        for (int i = 0; i < g_robotCount; ++i) {
            if (!g_robots[i].arrived) {
                allDone = 0;
                break;
            }
        }
        if (allDone) {
            printf("All robots reached their goals.\n");
            break;
        }

        // 每个时间步的“预定表”，防止多车抢同一个格子
        int reserved[MAP_H][MAP_W];
        memset(reserved, 0, sizeof(reserved));

        // 按 id 顺序，一个个为每辆车规划“一步”
        for (int i = 0; i < g_robotCount; ++i) {
            PlanOneStep(i, reserved);
        }

        printf("\n");
        step++;

        // 安全退出条件，防止意外死循环
        if (step > 50) {
            printf("Too many steps, stop simulation.\n");
            break;
        }
    }
    #ifdef _WIN32
    for (int i = 0; i < g_robotCount; ++i) {
        Serial_Close(&g_robots[i].port);
    }
    #endif

    return 0;
}
