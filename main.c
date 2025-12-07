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

#ifdef _WIN32
#include "SerialWin.h"
#endif

#include <stdio.h>
#include "PathAPI.h"
#include "planner.h"

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


int main(void)
{
    Debug_ScanCOMPorts();

    Planner_Init();

    int step = 0;
    printf("Multi-robot A* demo on %dx%d grid\n", MAP_W, MAP_H);

    while (1) {
        printf("======= STEP %d =======\n", step);
        Planner_PrintGrid();

        if (Planner_AllArrived()) {
            printf("All robots reached their goals.\n");
            break;
        }

        Planner_StepOnce();
        printf("\n");
        step++;

        if (step > 50) {
            printf("Too many steps, stop simulation.\n");
            break;
        }
    }

    Planner_Deinit();
    return 0;
}

