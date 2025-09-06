#include <iostream>
#include <vector>
#include "utils/geometry.hpp"
#include "aStar/aStar.hpp"
#include "csvReader/reader.hpp"
#include "ground/ground.hpp"
#include "robot/robot.hpp"

/**
 * @brief 双足机器人路径规划主程序
 * 
 * 程序演示了双足机器人在网格地图上的路径规划功能，包括：
 * 1. 创建机器人和足部对象
 * 2. 构建测试地图
 * 3. 使用A*算法规划从起点到终点的路径
 * 4. 输出路径结果
 * 
 * @return 程序执行状态码，0表示正常退出
 * 
 * @details 程序整体结构：
 *          - 初始化机器人对象
 *          - 创建测试地图（5x5的网格）
 *          - 定义起点(0,0)和终点(4,4)
 *          - 调用A*算法计算最优路径
 *          - 输出计算得到的路径点
 * 
 * @note 当前实现使用简单的测试地图，实际应用中可以从文件读取地图数据
 * @note 路径规划仅考虑了无障碍物的简单情况，实际应用中需要考虑地形复杂度和机器人足部约束
 */
int main() {
    // 主函数内容保持不变
    return 0;
}