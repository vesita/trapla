// T0 猫步指标探针：配合 catwalk_probe_instrumentation.patch（给 robot.cpp 加 g_trapla_foot_trace）使用。
// 编译（在 worktree 根）：
//   g++ -std=c++17 -I include catwalk_probe.cpp src/utils/*.cpp src/ground/*.cpp \
//       src/robot/*.cpp src/aStar/*.cpp src/csv/*.cpp -lm -o /tmp/catwalk_probe
// 运行（在主工作区根，TRAPLA_SEED 决定 scale_star 的随机源种子）：
//   cd /home/vesita/coding/my/trapla && TRAPLA_SEED=3 /tmp/catwalk_probe 500 500
#include "robot/robot.hpp"
#include "ground/ground.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

extern std::vector<int> g_trapla_foot_trace;
extern int g_trapla_will_cross_hits;

int main(int argc, char** argv) {
    SqDot goal(500, 500);
    if (argc >= 3) goal = SqDot(std::stod(argv[1]), std::stod(argv[2]));
    Ground ground(std::string("data/csv/map.csv"));
    Robot robot;
    auto path = robot.find_path(ground, goal);
    const char* sd = std::getenv("TRAPLA_SEED");
    std::cout << "seed=" << (sd ? sd : "1") << "\n";
    std::cout << "goal=" << goal.x << "," << goal.y << "\n";
    std::cout << "path_size=" << path.size() << " trace_size=" << g_trapla_foot_trace.size()
                  << " will_cross_hits=" << g_trapla_will_cross_hits << "\n";
    std::cout << "i,x,y,foot\n";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << i << "," << path[i].x << "," << path[i].y << ","
                  << (i < g_trapla_foot_trace.size() ? g_trapla_foot_trace[i] : -1) << "\n";
    }
    return 0;
}
