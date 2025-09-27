#include <iostream>
#include <vector>
#include <filesystem>
#include "utils/geometry.hpp"
#include "aStar/aStar.hpp"
#include "csv/reader.hpp"
#include "csv/writer.hpp"
#include "ground/ground.hpp"
#include "robot/robot.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

/**
 * @brief 双足机器人在线落足点规划系统主函数
 * 
 * 该程序实现了双足机器人在复杂地形上的路径规划功能，通过读取地形数据，
 * 使用A*算法进行路径搜索，并考虑机器人物理约束条件生成可行的行走路径。
 * 
 * @return 程序执行状态码，0表示正常退出
 */
int main(int argc, char const *argv[]) {
#ifdef _WIN32
    // 在Windows上设置控制台代码页为UTF-8
    SetConsoleOutputCP(CP_UTF8);
#endif

    try {
        std::cout << "=== 双足机器人路径规划系统 ===" << std::endl;
        
        // 初始化IO管理器
        auto& ioManager = IOManager::get_instance();
        
        // 演示CSV读写工具的使用
        std::cout << "=== CSV工具使用演示 ===" << std::endl;
        
        // 创建示例CSV数据并写入文件
        CSVWriter writer;
        std::vector<std::vector<double>> sampleData = {
            {1.0, 2.0, 3.0},
            {4.0, 5.0, 6.0},
            {7.0, 8.0, 9.0}
        };
        std::vector<std::string> columnNames = {"列1", "列2", "列3"};
        
        // 写入示例数据
        std::string sampleFile = ioManager.build_path("data/csv/sample_data.csv");
        // 确保data/csv目录存在
        ioManager.createDirectories(sampleFile);
        if (writer.writeToFile(sampleFile, sampleData, columnNames)) {
            std::cout << "示例数据已写入到 " << sampleFile << std::endl;
        } else {
            std::cerr << "写入示例数据失败" << std::endl;
            return 1;
        }
        
        // 演示CSV读取器的使用
        CSVReader reader;
        if (reader.readFromFile(sampleFile)) {
            std::cout << "成功读取文件，数据尺寸: " << reader.getRows() 
                      << "行 x " << reader.getCols() << "列" << std::endl;
            
            // 输出读取的数据
            const auto& data = reader.getData();
            for (size_t i = 0; i < data.size() && i < 5; ++i) {  // 限制输出前5行
                std::cout << "第" << i+1 << "行: ";
                for (size_t j = 0; j < data[i].size(); ++j) {
                    std::cout << data[i][j];
                    if (j < data[i].size() - 1) std::cout << ", ";
                }
                std::cout << std::endl;
            }
            if (data.size() > 5) {
                std::cout << "... (省略更多行)" << std::endl;
            }
        } else {
            std::cerr << "读取文件失败" << std::endl;
            return 1;
        }
        
        std::cout << std::endl << "=== 路径规划演示 ===" << std::endl;
        
        // 检查地形数据文件是否存在
        std::string mapFile = ioManager.build_path("data/csv/map.csv");
        if (!std::filesystem::exists(mapFile)) {
            std::cerr << "错误: 地形数据文件 " << mapFile << " 不存在" << std::endl;
            std::cerr << "请确保数据文件存在后再运行程序" << std::endl;
            return 1;
        }
        
        // 读取地形数据
        std::cout << "正在读取地形数据..." << std::endl;
        Ground ground(mapFile);
        std::cout << "地形数据读取完成" << std::endl;
        
        // 初始化机器人参数
        Robot robot;
        std::vector<SqDot> path;
        
        // 获取起点和终点坐标
        SqDot start, goal;
        if (argc >= 5) {
            // 从命令行参数获取起点和终点坐标
            double start_x = std::stod(argv[1]);
            double start_y = std::stod(argv[2]);
            double goal_x = std::stod(argv[3]);
            double goal_y = std::stod(argv[4]);
            
            start = SqDot(start_x, start_y);
            goal = SqDot(goal_x, goal_y);
            
            std::cout << "使用命令行参数设置起点: (" << start_x << ", " << start_y << ")" << std::endl;
            std::cout << "使用命令行参数设置终点: (" << goal_x << ", " << goal_y << ")" << std::endl;
        } else {
            // 使用默认起点和终点进行测试
            start = SqDot(0, 0);
            goal = SqDot(50, 50);
            
            std::cout << "使用默认起点: (0, 0)" << std::endl;
            std::cout << "使用默认终点: (50, 50)" << std::endl;
            std::cout << "提示: 可通过命令行参数指定起点和终点坐标: " << argv[0] << " <start_x> <start_y> <goal_x> <goal_y>" << std::endl;
        }
        
        // 设置机器人的初始足部位置
        std::cout << "设置机器人初始足部位置..." << std::endl;
        // 将左脚放在起点
        robot.feet[0] = Foot(start, 0.0, robot.feet[0].shape.length, robot.feet[0].shape.width);
        // 将右脚放在起点附近
        robot.feet[1] = Foot(SqDot(start.x, start.y - 2.0), 0.0, 
                           robot.feet[1].shape.length, robot.feet[1].shape.width);
        
        // 调用路径规划算法
        std::cout << "开始路径规划..." << std::endl;
        path = robot.find_path(ground, goal);
        std::cout << "路径规划完成" << std::endl;
        
        // 输出路径结果到控制台
        std::cout << std::endl << "=== 路径规划结果 ===" << std::endl;
        std::cout << "找到路径，共 " << path.size() << " 个点:" << std::endl;
        
        // 限制输出的点数，避免输出过多
        size_t maxPointsToShow = 20;
        for (size_t i = 0; i < path.size() && i < maxPointsToShow; ++i) {
            std::cout << "点 " << i << ": (" << path[i].x << ", " << path[i].y << ")" << std::endl;
        }
        if (path.size() > maxPointsToShow) {
            std::cout << "... (省略 " << (path.size() - maxPointsToShow) << " 个点)" << std::endl;
        }
        
        // 将路径结果写入CSV文件
        std::vector<std::vector<double>> csvData;
        std::vector<std::string> columnNamesPath = {"x", "y"};
        
        // 转换路径数据格式
        for (const auto& point : path) {
            csvData.push_back({point.x, point.y});
        }
        
        // 写入CSV文件
        std::string filename = ioManager.build_path("data/csv/path_result.csv");
        std::cout << "正在保存路径结果到 " << filename << " ..." << std::endl;
        if (writer.writeToFile(filename, csvData, columnNamesPath)) {
            std::cout << "路径结果已成功保存到 " << filename << std::endl;
        } else {
            std::cerr << "保存路径结果失败: " << filename << std::endl;
            return 1;
        }
        
        std::cout << std::endl << "程序执行完成" << std::endl;
        
    } catch (const std::invalid_argument& e) {
        std::cerr << "参数错误: " << e.what() << std::endl;
        std::cerr << "请确保提供的坐标参数是有效的数字" << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "未知错误发生" << std::endl;
        return 1;
    }

    return 0;
}