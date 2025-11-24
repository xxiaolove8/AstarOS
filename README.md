Public APIs and descriptions are shown here

/**
* @file PathAPI.h
* @brief 基于 A* 算法的二维栅格路径规划接口
*
* 本模块在一个 MAP_W x MAP_H 的二维网格上进行 A* 自动寻路。
* - 0 表示可行走格子
* - 1 表示障碍
*
* 使用流程：
*  1. 调用 PathAPI_Init() 初始化模块；
*  2. 调用 PathAPI_ClearMap() 清空地图；
*  3. 通过 PathAPI_AddObstacle(x, y) 标记障碍；
*  4. 调用 PathAPI_FindPath(start, end) 计算从起点到终点的最优路径；
*  5. 可通过 PathAPI_GetPathString() 获得形如 "(0,0)->(1,0)->..." 的路径字符串；
*  6. 调试时可调用 PathAPI_PrintPathAsGrid() 在串口/终端以字符画的方式显示地图和路径。
*
* 本模块内部使用 AStar.c / AStar.h 中提供的通用 A* 实现进行寻路。
  */

* @brief 栅格坐标（整数网格点）
*
* 坐标范围：x ∈ [0, MAP_W-1], y ∈ [0, MAP_H-1]
* - x：水平方向坐标（列索引）
* - y：竖直方向坐标（行索引）
    */
    typedef struct {
    int x;
    int y;
    } Position;



/**
* @brief 初始化路径规划模块
*
* 当前实现主要是调用 PathAPI_ClearMap() 清空地图，
* 预留将来需要的初始化逻辑（如从 Flash 读取地图等）。
*
* 调用时机：
*   - 系统上电后
*   - 地图需要重新初始化时
      */
      void PathAPI_Init(void);


/**
* @brief 清空地图，将所有格子设置为可行走（0）
*
* 会：
*   - 把内部 g_map[y][x] 全部置为 0
*   - 清空上一条路径的缓存
*
* 使用场景：
*   - 切换到全新环境或重新规划整个地图前
      */
      void PathAPI_ClearMap(void);


/**
* @brief 在地图中添加一个障碍格
*
* @param x  障碍格 x 坐标 [0, MAP_W-1]
* @param y  障碍格 y 坐标 [0, MAP_H-1]
* @return   1 表示添加成功，0 表示坐标越界（未修改地图）
*
* 说明：
*   - 内部将 g_map[y][x] 置为 1，表示该格子为障碍，不可通行。
*   - 可多次调用以添加多个障碍。
      */
      int PathAPI_AddObstacle(int x, int y);



/**
* @brief 使用 A* 算法计算从 start 到 end 的最优路径
*
* @param start 起点栅格坐标
* @param end   终点栅格坐标
* @return      1 表示找到路径并缓存到内部数组；0 表示无可行路径
*
* 说明：
*   - 使用 MAP_W x MAP_H 的全局地图和障碍信息进行寻路。
*   - 内部构造 ASPathNodeSource，设置：
*        - nodeSize = sizeof(Position)
*        - nodeNeighbors = 基于上下左右四邻居、忽略障碍的回调
*        - pathCostHeuristic = 曼哈顿距离启发函数
*        - nodeComparator = 先比较 y，再比较 x
*   - 通过 ASPathCreate() 调用 A* 库进行寻路，将结果节点复制到内部 g_pathNodes[] 数组。
*   - 路径最长不会超过 MAP_W * MAP_H。
*
* 注意：
*   - 调用前请确保 start / end 在地图范围内，且非障碍格。
*   - 若返回 0，可调用 PathAPI_PrintPathAsGrid() 检查地图是否被障碍完全堵死。
      */
      int PathAPI_FindPath(Position start, Position end);


/**
* @brief 以字符串形式获取最新一次规划出的路径
*
* 路径格式类似：
*   "(x0,y0)->(x1,y1)->(x2,y2)"
*
* @param outBuf      调用方提供的输出缓冲区
* @param outBufSize  缓冲区大小（单位：字节）
* @return            1 = 有有效路径并成功写入字符串；0 = 当前无路径（将返回 "NO PATH"）
*
* 使用示例：
*   char buf[128];
*   if (PathAPI_GetPathString(buf, sizeof(buf))) {
*       printf("path=%s\r\n", buf);
*   }
    */
    int PathAPI_GetPathString(char *outBuf, size_t outBufSize);
    /**
* @brief 以字符画的方式打印当前地图及路径（调试用）
*
* 输出示例（通过 printf 到串口）：
*   Grid (W=20, H=20):
*   S=start, E=end, O=path, #=obstacle, .=free
*   y=0: S . # . .
*   y=1: O . # . .
*   ...
*        0 1 2 3 4 <- x
*
* 图例：
*   'S' = 起点
*   'E' = 终点
*   'O' = 路径经过的格子
*   '#' = 障碍
*   '.' = 可行走空地
*
* 典型用法：
*   - 在上位机调试（printf 到控制台）
*   - 在单片机上通过串口查看地图 / 路径是否正确
      */
      void PathAPI_PrintPathAsGrid(void);