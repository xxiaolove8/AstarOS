//
// Created by XxiaoLove8 on 2025/12/7.
//

#ifndef ASTAROS_PLANNER_H
#define ASTAROS_PLANNER_H
#include "PathAPI.h"  // 里面有 Position、MAP_W/MAP_H 的定义
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

    // 初始化：地图 + 机器人 + 串口（等价于你现在的 Init_MapAndRobots + 串口打开那段）
    void Planner_Init(void);

    // 打印当前网格（相当于 Print_Grid_ASCII）
    void Planner_PrintGrid(void);

    // 判断是不是所有机器人都到达了目标
    int Planner_AllArrived(void);   // 返回 1 = 全部到达，0 = 还有车在路上

    // 执行“一步规划”：为每辆车各规划一步，并发送 L/R/F
    void Planner_StepOnce(void);

    // 可选：结束时关闭串口
    void Planner_Deinit(void);

#ifdef __cplusplus
}
#endif
#endif //ASTAROS_PLANNER_H