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

    // Robot 0
    g_robots[0].id      = 0;
    g_robots[0].pos.x   = 0;
    g_robots[0].pos.y   = 0;
    g_robots[0].goal.x  = 7;
    g_robots[0].goal.y  = 7;
    g_robots[0].arrived = 0;
    g_robots[0].heading = DIR_RIGHT;
#ifdef _WIN32
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
    g_robots[2].comName = "COM5";
#endif
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

// ========== 构建“动态地图”：其他车 + 预定格子都视为障碍 ==========
static void BuildDynamicMap(int selfIndex, int reserved[MAP_H][MAP_W])
{
    PathAPI_ClearMap();

    // 1. 静态障碍
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            if (g_staticObstacles[y][x]) {
                PathAPI_AddObstacle(x, y);
            }
        }
    }

    // 2. 其他机器人
    for (int i = 0; i < g_robotCount; ++i) {
        if (i == selfIndex) continue;
        if (g_robots[i].arrived) continue;

        int ox = g_robots[i].pos.x;
        int oy = g_robots[i].pos.y;

        // 🚩 如果这个位置刚好是当前机器人 self 的目标，就暂时不当成障碍
        if (ox == g_robots[selfIndex].goal.x &&
            oy == g_robots[selfIndex].goal.y) {
            continue;
            }

        PathAPI_AddObstacle(ox, oy);
    }

    // 3. reserved
    for (int y = 0; y < MAP_H; ++y) {
        for (int x = 0; x < MAP_W; ++x) {
            if (reserved[y][x]) {
                PathAPI_AddObstacle(x, y);
            }
        }
    }
}


// 根据 pos -> next，推导目标朝向
static Heading GetTargetHeading(Position from, Position to)
{
    int dx = to.x - from.x;
    int dy = to.y - from.y;

    if (dx == 1 && dy == 0)  return DIR_RIGHT;
    if (dx == -1 && dy == 0) return DIR_LEFT;
    if (dx == 0 && dy == 1)  return DIR_DOWN;
    if (dx == 0 && dy == -1) return DIR_UP;

    // 非相邻，异常
    return DIR_UP;
}

// 对一个机器人，发送一个动作（可以扩展成发送序列）
static void SendAction(Robot *rb, char action)
{
    printf("Send to robot %d: %c\n", rb->id, action);

#ifdef _WIN32
    // 如果串口打开成功，就实际发出去
    if (rb->port.h && rb->port.h != INVALID_HANDLE_VALUE) {
        Serial_SendByte(&rb->port, action);
    }
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


// ========== 主仿真入口 ==========
int main(void)
{
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
