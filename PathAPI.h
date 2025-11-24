//
// Created by 29465 on 2025/11/23.
//

#ifndef UNTITLED_PATHAPI_H
#define UNTITLED_PATHAPI_H

#include <stddef.h>

#define MAP_W 8
#define MAP_H 8

// 坐标结构（0~4）
typedef struct {
    int x;
    int y;
} Position;

/**
 * 初始化路径规划模块（目前主要是预留接口）
 */
void PathAPI_Init(void);

/**
 * 清空地图（全部变为可走）
 */
void PathAPI_ClearMap(void);

/**
 * 添加一个障碍到地图中
 * @param x, y   障碍坐标 (0~4)
 * @return       1 = 添加成功, 0 = 坐标越界
 */
int PathAPI_AddObstacle(int x, int y);

/**
 * 计算从 start 到 end 的最优路径
 * @param start, end 起点 / 终点
 * @return           1 = 找到路径, 0 = 无路可走
 */
int PathAPI_FindPath(Position start, Position end);

/**
 * 以字符串形式获取路径，例如: "(0,0)->(0,1)->(1,1)"
 * @param outBuf     输出缓冲区
 * @param outBufSize 缓冲区大小
 * @return           1 = 有路径并已写入, 0 = 当前没有有效路径
 */
int PathAPI_GetPathString(char *outBuf, size_t outBufSize);

/**
 * 以「图形方式」打印 5x5 网格和路径
 * 建议用 printf 输出到串口/终端
 *
 * 图例：
 *   'S' = 起点
 *   'E' = 终点
 *   'O' = 路径
 *   '#' = 障碍
 *   '.' = 空地
 */
void PathAPI_PrintPathAsGrid(void);


#endif //UNTITLED_PATHAPI_H