// T0 C++ 函数级 golden 导出器
//
// 目的：把与 RNG 无关的纯函数在 C++ 原实现下的 输入->输出 冻结成 CSV，供 Rust 侧逐条对齐。
// 只调用 include/ 下的公开接口；不改任何被测代码；不依赖 scale_star 的随机源。
//
// 编译（在仓库根）：
//   g++ -std=c++17 -I include baseline/harness/golden_dump.cpp \
//       src/utils/*.cpp src/ground/*.cpp src/robot/*.cpp src/aStar/*.cpp src/csv/*.cpp \
//       -lm -o /tmp/golden_dump
// 运行（在仓库根，输出目录必须存在）：
//   /tmp/golden_dump baseline/golden
//
// 约定：
//  - 所有浮点用 setprecision(17) 输出，保证可 round-trip。
//  - Ground 的每个格子都显式导出到 grounds.csv；Rust 侧读同一份，不要试图复现高度公式。
//  - Foot::cover 返回 unordered_set 的迭代顺序未定义，这里按 (x,y) 升序导出，Rust 必须按集合比较。
//  - slide 的结果枚举按声明顺序：Modified=0, NoModification=1, NotApplicable=2。

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "aStar/aStar.hpp"
#include "ground/ground.hpp"
#include "robot/foot.hpp"
#include "robot/robot.hpp"
#include "utils/geometry.hpp"
#include "utils/index.hpp"
#include "utils/scale.hpp"
#include "utils/sequence.hpp"
#include "aStar/direction.hpp"

static std::string N(double v) {
    if (std::isnan(v)) return "nan";
    if (std::isinf(v)) return v > 0 ? "inf" : "-inf";
    std::ostringstream ss;
    ss << std::setprecision(17) << v;
    return ss.str();
}
static std::string NI(long long v) { return std::to_string(v); }

struct Csv {
    std::ofstream f;
    long long rows = 0;
    explicit Csv(const std::string& path) : f(path) {}
    void row(const std::vector<std::string>& cells) {
        for (size_t i = 0; i < cells.size(); ++i) {
            if (i) f << ",";
            f << cells[i];
        }
        f << "\n";
        ++rows;
    }
};

// 确定性伪随机（不用 std::random_*，避免平台差异；Rust 侧不需要复现，只需读导出的输入）
struct Lcg {
    uint64_t s;
    explicit Lcg(uint64_t seed) : s(seed) {}
    uint32_t next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<uint32_t>(s >> 33);
    }
    double uni(double a, double b) { return a + (b - a) * (next() % 1000000) / 1000000.0; }
    int irand(int a, int b) { return a + static_cast<int>(next() % static_cast<uint32_t>(b - a + 1)); }
};

static const char* slide_name(SlideResult r) {
    switch (r) {
        case SlideResult::Modified: return "Modified";
        case SlideResult::NoModification: return "NoModification";
        case SlideResult::NotApplicable: return "NotApplicable";
    }
    return "?";
}
static int slide_int(SlideResult r) {
    switch (r) {
        case SlideResult::Modified: return 0;
        case SlideResult::NoModification: return 1;
        case SlideResult::NotApplicable: return 2;
    }
    return -1;
}

static std::vector<Ground> build_grounds() {
    std::vector<Ground> gs;
    auto put = [](Ground& g, int x, int y, double v) { g.map[x][y] = v; };

    {   // g0 5x7 全 0
        Ground g(5, 7);
        for (int x = 0; x < 5; ++x) for (int y = 0; y < 7; ++y) put(g, x, y, 0.0);
        gs.push_back(g);
    }
    {   // g1 8x8 线性斜面
        Ground g(8, 8);
        for (int x = 0; x < 8; ++x) for (int y = 0; y < 8; ++y) put(g, x, y, 0.5 * x + 0.25 * y);
        gs.push_back(g);
    }
    {   // g2 6x9 起伏
        Ground g(6, 9);
        for (int x = 0; x < 6; ++x) for (int y = 0; y < 9; ++y)
            put(g, x, y, 10.0 * std::sin(0.7 * x) + 5.0 * std::cos(0.5 * y));
        gs.push_back(g);
    }
    {   // g3 7x7 x==3 一列 infinity
        Ground g(7, 7);
        for (int x = 0; x < 7; ++x) for (int y = 0; y < 7; ++y)
            put(g, x, y, (x == 3) ? std::numeric_limits<double>::infinity() : 1.0 * x);
        gs.push_back(g);
    }
    {   // g4 6x10 LCG 噪声
        Ground g(6, 10);
        Lcg rng(12345);
        for (int x = 0; x < 6; ++x) for (int y = 0; y < 10; ++y) put(g, x, y, rng.uni(0.0, 20.0));
        gs.push_back(g);
    }
    {   // g5 10x10 斜面 + 4 个负值障碍格
        Ground g(10, 10);
        for (int x = 0; x < 10; ++x) for (int y = 0; y < 10; ++y) put(g, x, y, 2.0 * x - 0.5 * y + 3.0);
        put(g, 2, 2, -1.0); put(g, 2, 3, -1.0); put(g, 7, 7, -1.0); put(g, 0, 9, -1.0);
        gs.push_back(g);
    }
    {   // g6 5x5 走廊：只有 x==2 或 y==2 有限
        Ground g(5, 5);
        for (int x = 0; x < 5; ++x) for (int y = 0; y < 5; ++y)
            put(g, x, y, (x == 2 || y == 2) ? 5.0 + x : std::numeric_limits<double>::infinity());
        gs.push_back(g);
    }
    return gs;
}

static void dump_grounds(const std::vector<Ground>& gs, const std::string& dir) {
    Csv c(dir + "/grounds.csv");
    c.row({"ground_id", "x", "y", "height"});
    for (size_t i = 0; i < gs.size(); ++i) {
        auto sh = gs[i].shape();
        for (int x = 0; x < sh[0]; ++x)
            for (int y = 0; y < sh[1]; ++y)
                c.row({NI((long long)i), NI(x), NI(y), N(gs[i].map[x][y])});
    }
    std::cout << "grounds.csv rows=" << c.rows - 1 << "\n";
}

int main(int argc, char** argv) {
    std::string dir = (argc > 1) ? argv[1] : "baseline/golden";
    std::vector<Ground> gs = build_grounds();
    dump_grounds(gs, dir);
    Lcg rng(20251016);

    // ---------- normal_angle ----------
    {
        Csv c(dir + "/normal_angle.csv");
        c.row({"case", "p1x", "p1y", "p1z", "p2x", "p2y", "p2z", "p3x", "p3y", "p3z",
               "A", "B", "C", "D", "angle"});
        for (int k = 0; k < 45; ++k) {
            CuDot p1, p2, p3;
            if (k < 3) {  // 水平面 -> angle 应为 0
                p1 = CuDot(0, 0, 5); p2 = CuDot(1, 0, 5); p3 = CuDot(0, 1, 5);
            } else if (k < 6) {  // 共线/退化
                p1 = CuDot(1, 1, 1); p2 = CuDot(2, 2, 2); p3 = CuDot(3, 3, 3);
            } else if (k < 9) {  // 近 90 度竖直面
                p1 = CuDot(0, 0, 0); p2 = CuDot(0, 1, 0); p3 = CuDot(0, 0, 1);
            } else {
                p1 = CuDot(rng.uni(-5, 5), rng.uni(-5, 5), rng.uni(-5, 5));
                p2 = CuDot(rng.uni(-5, 5), rng.uni(-5, 5), rng.uni(-5, 5));
                p3 = CuDot(rng.uni(-5, 5), rng.uni(-5, 5), rng.uni(-5, 5));
            }
            std::array<CuDot, 3> three = {p1, p2, p3};
            CuPlane pl(three);
            c.row({NI(k), N(p1.x), N(p1.y), N(p1.z), N(p2.x), N(p2.y), N(p2.z),
                   N(p3.x), N(p3.y), N(p3.z), N(pl.A), N(pl.B), N(pl.C), N(pl.D),
                   N(pl.normal_angle())});
        }
        std::cout << "normal_angle.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- up_fit ----------
    {
        Csv c(dir + "/up_fit.csv");
        c.row({"case", "p1x", "p1y", "p1z", "p2x", "p2y", "p2z", "p3x", "p3y", "p3z",
               "qx", "qy", "qz",
               "outA", "outB", "outC", "outD",
               "m0x", "m0y", "m0z", "m1x", "m1y", "m1z", "m2x", "m2y", "m2z"});
        for (int k = 0; k < 45; ++k) {
            std::array<CuDot, 3> three = {
                CuDot(rng.uni(-4, 4), rng.uni(-4, 4), rng.uni(-4, 4)),
                CuDot(rng.uni(-4, 4), rng.uni(-4, 4), rng.uni(-4, 4)),
                CuDot(rng.uni(-4, 4), rng.uni(-4, 4), rng.uni(-4, 4)),
            };
            if (k < 5) {  // 强制一个"在平面上方"的查询点
                three[0] = CuDot(0, 0, 0); three[1] = CuDot(1, 0, 0); three[2] = CuDot(0, 1, 0);
            }
            CuDot q = (k < 5) ? CuDot(0.25, 0.25, 3.0)
                              : CuDot(rng.uni(-4, 4), rng.uni(-4, 4), rng.uni(-4, 4));
            CuPlane pl = up_fit(q, three);
            c.row({NI(k),
                   N(three[0].x), N(three[0].y), N(three[0].z),
                   N(three[1].x), N(three[1].y), N(three[1].z),
                   N(three[2].x), N(three[2].y), N(three[2].z),
                   N(q.x), N(q.y), N(q.z),
                   N(pl.A), N(pl.B), N(pl.C), N(pl.D),
                   N(three[0].x), N(three[0].y), N(three[0].z),
                   N(three[1].x), N(three[1].y), N(three[1].z),
                   N(three[2].x), N(three[2].y), N(three[2].z)});
        }
        std::cout << "up_fit.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- ground.trip ----------
    {
        Csv c(dir + "/trip.csv");
        c.row({"case", "ground_id", "area_n",
               "x0", "y0", "x1", "y1", "x2", "y2", "x3", "y3", "x4", "y4", "x5", "y5",
               "A", "B", "C", "D", "define_extend", "normal_angle"});
        for (int k = 0; k < 45; ++k) {
            int gid = (k % 5);  // 只用有限高度地面 g0,g1,g2,g4,g5
            if (gid == 3) gid = 4;
            auto sh = gs[gid].shape();
            int n = 3 + (k % 4);  // 3..6
            std::vector<SqDot> area;
            std::set<std::pair<int, int>> used;
            while (static_cast<int>(area.size()) < n) {
                int x = rng.irand(0, sh[0] - 1);
                int y = rng.irand(0, sh[1] - 1);
                if (used.count({x, y})) continue;
                used.insert({x, y});
                area.emplace_back(SqDot(x, y));
            }
            CuPlane pl = gs[gid].trip(area);
            std::vector<std::string> cells = {NI(k), NI(gid), NI(n)};
            for (int i = 0; i < 6; ++i) {
                if (i < n) { cells.push_back(N(area[i].x)); cells.push_back(N(area[i].y)); }
                else { cells.push_back(""); cells.push_back(""); }
            }
            cells.push_back(N(pl.A)); cells.push_back(N(pl.B));
            cells.push_back(N(pl.C)); cells.push_back(N(pl.D));
            cells.push_back(NI(pl.define_extend));
            cells.push_back(N(pl.normal_angle()));
            c.row(cells);
        }
        std::cout << "trip.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- Foot::cover / Foot::corner ----------
    {
        Csv c(dir + "/cover.csv");
        c.row({"case", "x", "y", "rz", "length", "width", "pt_idx", "px", "py"});
        long long cases = 0;
        for (int k = 0; k < 40; ++k) {
            double x = rng.uni(-3, 30), y = rng.uni(-3, 30), rz = rng.uni(-M_PI, M_PI);
            double len = rng.uni(1, 8), wid = rng.uni(1, 8);
            if (k < 4) { x = 0; y = 0; rz = k * M_PI / 2; len = 5; wid = 3; }
            Foot f(SqDot(x, y), rz, len, wid);
            std::set<std::pair<double, double>> pts;
            for (const auto& p : f.cover()) pts.insert({p.x, p.y});
            long long idx = 0;
            for (const auto& p : pts) {
                c.row({NI(k), N(x), N(y), N(rz), N(len), N(wid), NI(idx), N(p.first), N(p.second)});
                ++idx;
            }
            ++cases;
        }
        std::cout << "cover.csv cases=" << cases << " rows=" << c.rows - 1 << "\n";
    }
    {
        Csv c(dir + "/corner.csv");
        c.row({"case", "x", "y", "rz", "length", "width",
               "c0x", "c0y", "c1x", "c1y", "c2x", "c2y", "c3x", "c3y"});
        for (int k = 0; k < 40; ++k) {
            double x = rng.uni(-3, 30), y = rng.uni(-3, 30), rz = rng.uni(-M_PI, M_PI);
            double len = rng.uni(1, 8), wid = rng.uni(1, 8);
            if (k < 4) { x = 1; y = 1; rz = k * M_PI / 2; len = 5; wid = 3; }
            Foot f(SqDot(x, y), rz, len, wid);
            auto cs = f.corner();
            c.row({NI(k), N(x), N(y), N(rz), N(len), N(wid),
                   N(cs[0].x), N(cs[0].y), N(cs[1].x), N(cs[1].y),
                   N(cs[2].x), N(cs[2].y), N(cs[3].x), N(cs[3].y)});
        }
        std::cout << "corner.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- Foot::standable ----------
    {
        Csv c(dir + "/standable.csv");
        c.row({"case", "ground_id", "x", "y", "rz", "length", "width",
               "covered_count", "all_inside", "normal_angle", "standable"});
        for (int k = 0; k < 45; ++k) {
            int gid = k % 5;
            if (gid == 3) gid = 5;  // 只用有限高度地面（inf 会让 trip/slide 进入 nan）
            auto sh = gs[gid].shape();
            double x = rng.uni(-3, sh[0] + 2), y = rng.uni(-3, sh[1] + 2);
            double rz = rng.uni(-M_PI, M_PI), len = 5, wid = 3;
            if (k < 5) { x = 0; y = 0; rz = 0; }  // 站在角落：覆盖点越界
            Foot f(SqDot(x, y), rz, len, wid);
            auto pts = f.cover();
            bool all_in = true;
            for (const auto& p : pts) if (!gs[gid].is_valid(p)) { all_in = false; break; }
            CuPlane pl = gs[gid].trip(pts);
            bool st = f.standable(gs[gid]);
            c.row({NI(k), NI(gid), N(x), N(y), N(rz), N(len), N(wid),
                   NI((long long)pts.size()), all_in ? "1" : "0",
                   N(pl.normal_angle()), st ? "1" : "0"});
        }
        std::cout << "standable.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- FootShape::slide ----------
    {
        Csv c(dir + "/slide.csv");
        c.row({"case", "ground_id", "x", "y", "rz", "length", "width",
               "in0x", "in0y", "in1x", "in1y", "in2x", "in2y", "in3x", "in3y",
               "result_int", "result_name",
               "out0x", "out0y", "out1x", "out1y", "out2x", "out2y", "out3x", "out3y"});
        for (int k = 0; k < 45; ++k) {
            int gid = k % 5;
            if (gid == 3) gid = 5;  // 只用有限高度地面（inf 会让 normal/slide 进入 nan）
            auto sh = gs[gid].shape();
            double x = rng.uni(2, sh[0] - 3), y = rng.uni(2, sh[1] - 3);
            double rz = rng.uni(-M_PI, M_PI), len = 5, wid = 3;
            if (k < 5) { x = 2; y = 2; rz = 0; }
            if (sh[0] < 6) { x = 1; y = 1; }
            Foot f(SqDot(x, y), rz, len, wid);
            std::vector<SqDot> area = f.corner();
            std::vector<SqDot> in_area = area;
            FootShape shape(len, wid);
            SlideResult r = shape.slide(area, gs[gid]);
            std::vector<std::string> cells = {NI(k), NI(gid), N(x), N(y), N(rz), N(len), N(wid)};
            for (const auto& p : in_area) { cells.push_back(N(p.x)); cells.push_back(N(p.y)); }
            cells.push_back(NI(slide_int(r)));
            cells.push_back(slide_name(r));
            for (size_t i = 0; i < 4; ++i) {
                if (i < area.size()) { cells.push_back(N(area[i].x)); cells.push_back(N(area[i].y)); }
                else { cells.push_back(""); cells.push_back(""); }
            }
            c.row(cells);
        }
        std::cout << "slide.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- satisfy_spacing ----------
    {
        Csv c(dir + "/satisfy_spacing.csv");
        c.row({"case", "l0x", "l0y", "l0rz", "r0x", "r0y", "r0rz", "now_which",
               "newx", "newy", "min_sep", "max_sep", "spacing", "result"});
        // 先原样喂 constraints_test.cpp:14-26 的 11 组，再补随机与边界
        std::vector<SqDot> cs = {SqDot(50, 55), SqDot(50, 58), SqDot(50, 52), SqDot(50, 60),
                                 SqDot(50, 51), SqDot(44, 40), SqDot(65, 59), SqDot(75, 40),
                                 SqDot(72, 46), SqDot(62, 50), SqDot(58, 57)};
        long long k = 0;
        auto emit = [&](double l0x, double l0y, double l0rz, double r0x, double r0y, double r0rz,
                        int now, SqDot np, double mn, double mx) {
            Robot robot(40.0, M_PI * 75.0 / 180.0, mx, mn, 5.0, 3.0);
            robot.feet[0].position = SqDot(l0x, l0y); robot.feet[0].rz = l0rz;
            robot.feet[1].position = SqDot(r0x, r0y); robot.feet[1].rz = r0rz;
            robot.now_which_foot_to_move = (now == 0) ? WhichFoot::Left : WhichFoot::Right;
            auto sw = robot.get_swing_foot();
            auto sp = robot.get_support_foot();
            auto nf = robot.next(np);
            auto pts = nf.corner();
            SqLine line(sp.position, sp.rz);
            double spacing = std::numeric_limits<double>::infinity();
            for (const auto& p : pts) spacing = std::min(spacing, line.distance(p));
            spacing = std::max(0.0, spacing - sp.shape.width / 2.0);
            bool r = robot.satisfy_spacing(np);
            c.row({NI(k), N(l0x), N(l0y), N(l0rz), N(r0x), N(r0y), N(r0rz), NI(now),
                   N(np.x), N(np.y), N(mn), N(mx), N(spacing), r ? "1" : "0"});
            ++k;
        };
        for (const auto& p : cs) emit(50, 50, 0, 50, 45, 0, 1, p, 2.0, 10.0);
        for (int i = 0; i < 35; ++i) {
            double l0x = rng.uni(0, 80), l0y = rng.uni(0, 80), l0rz = rng.uni(-M_PI, M_PI);
            double r0x = rng.uni(0, 80), r0y = rng.uni(0, 80), r0rz = rng.uni(-M_PI, M_PI);
            int now = rng.irand(0, 1);
            SqDot np(rng.uni(0, 80), rng.uni(0, 80));
            emit(l0x, l0y, l0rz, r0x, r0y, r0rz, now, np, 2.0, 10.0);
        }
        emit(0, 0, 0, 0, 0, 0, 0, SqDot(0, 0), 2.0, 10.0);      // 两脚重合
        emit(10, 10, 0, 10, 12, 0, 1, SqDot(10, 15), 2.0, 10.0); // 间距恰好 = 2（support y=10, 不给 foot 转向）
        emit(10, 10, 0, 10, 12, 0, 1, SqDot(10, 23), 2.0, 10.0); // 间距恰好 = 10
        // epsilon=1e-3 容差探针。(50,50)/(50,45) 配置下：
        //   stride < active_distance(16) 时 foot 不转向，spacing = newy-53；
        //   newy>=61 时 foot 转向 90°，spacing = newy-54。
        emit(50, 50, 0, 50, 45, 0, 1, SqDot(50.0, 54.999), 2.0, 10.0);   // spacing=1.999   -> 代码判 true
        emit(50, 50, 0, 50, 45, 0, 1, SqDot(50.0, 54.9985), 2.0, 10.0);  // spacing=1.9985  -> false
        emit(50, 50, 0, 50, 45, 0, 1, SqDot(50.0, 54.998), 2.0, 10.0);   // spacing=1.998   -> false
        emit(50, 50, 0, 50, 45, 0, 1, SqDot(50.0, 64.0005), 2.0, 10.0);  // spacing=10.0005 -> true
        emit(50, 50, 0, 50, 45, 0, 1, SqDot(50.0, 64.001), 2.0, 10.0);   // spacing=10.001  -> true（恰好上界）
        emit(50, 50, 0, 50, 45, 0, 1, SqDot(50.0, 64.0011), 2.0, 10.0);  // spacing=10.0011 -> false
        std::cout << "satisfy_spacing.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- satisfy_stride ----------
    {
        Csv c(dir + "/satisfy_stride.csv");
        c.row({"case", "swx", "swy", "swrz", "supx", "supy", "suprz", "max_stride",
               "newx", "newy", "distance", "result"});
        long long k = 0;
        auto emit = [&](double swx, double swy, double swrz, double supx, double supy, double suprz,
                        double ms, SqDot np) {
            Robot robot(ms, M_PI * 75.0 / 180.0, 10.0, 2.0, 5.0, 3.0);
            robot.feet[0].position = SqDot(swx, swy); robot.feet[0].rz = swrz;
            robot.feet[1].position = SqDot(supx, supy); robot.feet[1].rz = suprz;
            robot.now_which_foot_to_move = WhichFoot::Left;
            double d = robot.distance(np);
            bool r = robot.satisfy_stride(np);
            c.row({NI(k), N(swx), N(swy), N(swrz), N(supx), N(supy), N(suprz), N(ms),
                   N(np.x), N(np.y), N(d), r ? "1" : "0"});
            ++k;
        };
        emit(0, 0, 0, 0, 5, 0, 40.0, SqDot(39.999999, 0.0));  // 刚好小于
        emit(0, 0, 0, 0, 5, 0, 40.0, SqDot(40.0, 0.0));       // 恰好等于 -> false（严格小于）
        emit(0, 0, 0, 0, 5, 0, 40.0, SqDot(40.000001, 0.0));  // 略大 -> false
        emit(0, 0, 0, 0, 5, 0, 40.0, SqDot(0, 0));          // 原地
        emit(0, 0, 0, 0, 5, 0, 40.0, SqDot(40.0 / std::sqrt(2.0), 40.0 / std::sqrt(2.0)));  // 对角 40
        for (int i = 0; i < 35; ++i) {
            double swx = rng.uni(0, 100), swy = rng.uni(0, 100), swrz = rng.uni(-M_PI, M_PI);
            double supx = rng.uni(0, 100), supy = rng.uni(0, 100), suprz = rng.uni(-M_PI, M_PI);
            double ms = rng.uni(5, 60);
            SqDot np(rng.uni(0, 100), rng.uni(0, 100));
            emit(swx, swy, swrz, supx, supy, suprz, ms, np);
        }
        std::cout << "satisfy_stride.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- satisfy_turn ----------
    {
        Csv c(dir + "/satisfy_turn.csv");
        c.row({"case", "swx", "swy", "swrz", "supx", "supy", "suprz", "max_turn",
               "newx", "newy", "angle", "result"});
        long long k = 0;
        auto emit = [&](double swx, double swy, double swrz, double supx, double supy, double suprz,
                        double mt, SqDot np) {
            Robot robot(40.0, mt, 10.0, 2.0, 5.0, 3.0);
            robot.feet[0].position = SqDot(swx, swy); robot.feet[0].rz = swrz;
            robot.feet[1].position = SqDot(supx, supy); robot.feet[1].rz = suprz;
            robot.now_which_foot_to_move = WhichFoot::Left;
            double ang = robot.get_swing_foot().position.angle(np) - robot.get_support_foot().rz;
            bool r = robot.satisfy_turn(np);
            c.row({NI(k), N(swx), N(swy), N(swrz), N(supx), N(supy), N(suprz), N(mt),
                   N(np.x), N(np.y), N(ang), r ? "1" : "0"});
            ++k;
        };
        double mt = M_PI * 75.0 / 180.0;
        emit(0, 0, 0, 0, 5, 0, mt, SqDot(std::cos(mt - 0.001), std::sin(mt - 0.001)));
        emit(0, 0, 0, 0, 5, 0, mt, SqDot(std::cos(mt), std::sin(mt)));
        emit(0, 0, 0, 0, 5, 0, mt, SqDot(std::cos(mt + 0.001), std::sin(mt + 0.001)));
        emit(0, 0, 0, 0, 5, 0, mt, SqDot(1, 0));                    // 0 角
        emit(0, 0, 0, 0, 5, 0, mt, SqDot(std::cos(-mt + 0.001), std::sin(-mt + 0.001)));
        emit(0, 0, 0, 0, 5, M_PI - 0.1, mt, SqDot(1, 0));           // 角度回绕：angle ~ -2.9
        emit(0, 0, 0, 0, 5, -M_PI + 0.1, mt, SqDot(1, 0));          // 角度回绕：angle ~ 2.9
        for (int i = 0; i < 35; ++i) {
            double swx = rng.uni(0, 100), swy = rng.uni(0, 100), swrz = rng.uni(-M_PI, M_PI);
            double supx = rng.uni(0, 100), supy = rng.uni(0, 100), suprz = rng.uni(-M_PI, M_PI);
            SqDot np(rng.uni(0, 100), rng.uni(0, 100));
            emit(swx, swy, swrz, supx, supy, suprz, mt, np);
        }
        std::cout << "satisfy_turn.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- restore / restore_dot / central_restore ----------
    {
        Csv c(dir + "/restore.csv");
        c.row({"case", "mode", "blockx", "blocky", "scale", "outax", "outay", "outbx", "outby"});
        long long k = 0;
        for (int i = 0; i < 20; ++i) {
            int bx = rng.irand(-3, 30), by = rng.irand(-3, 30);
            double sc = (i % 4 == 0) ? 0.025 : (i % 4 == 1 ? 0.5 : (i % 4 == 2 ? 1.0 : 1.0 / 40.0));
            Intex bi(bx, by);
            auto pr = gs[0].map.restore(bi, sc);
            c.row({NI(k++), "int", NI(bx), NI(by), N(sc),
                   NI(pr.first.x), NI(pr.first.y), NI(pr.second.x), NI(pr.second.y)});
            SqDot bd(bx, by);
            auto pd = gs[0].map.restore(bd, sc);
            c.row({NI(k++), "dot", N(bx), N(by), N(sc),
                   N(pd.first.x), N(pd.first.y), N(pd.second.x), N(pd.second.y)});
        }
        std::cout << "restore.csv rows=" << c.rows - 1 << "\n";
    }
    {
        Csv c(dir + "/restore_dot.csv");
        c.row({"case", "mode", "inx", "iny", "scale", "outx", "outy"});
        long long k = 0;
        for (int i = 0; i < 30; ++i) {
            int x = rng.irand(-5, 60), y = rng.irand(-5, 60);
            double sc = (i % 3 == 0) ? 0.025 : (i % 3 == 1 ? 0.5 : 2.0);
            Intex di(x, y);
            Intex oi = gs[0].map.restore_dot(di, sc);
            c.row({NI(k++), "int", NI(x), NI(y), N(sc), NI(oi.x), NI(oi.y)});
            SqDot dd(x, y);
            SqDot od = gs[0].map.restore_dot(dd, sc);
            c.row({NI(k++), "dot", N(x), N(y), N(sc), N(od.x), N(od.y)});
        }
        std::cout << "restore_dot.csv rows=" << c.rows - 1 << "\n";
    }
    {
        Csv c(dir + "/central_restore.csv");
        c.row({"case", "mode", "inx", "iny", "scale", "outx", "outy"});
        long long k = 0;
        for (int i = 0; i < 35; ++i) {
            int x = rng.irand(-10, 80), y = rng.irand(-10, 80);
            double sc = (i % 4 == 0) ? 0.025 : (i % 4 == 1 ? 0.5 : (i % 4 == 2 ? 1.0 : 3.0));
            Intex di(x, y);
            Intex oi = di.central_restore(sc);
            c.row({NI(k++), "int", NI(x), NI(y), N(sc), NI(oi.x), NI(oi.y)});
            SqDot dd(x, y);
            SqDot od = dd.central_restore(sc);
            c.row({NI(k++), "dot", N(x), N(y), N(sc), N(od.x), N(od.y)});
        }
        std::cout << "central_restore.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- index_scale / scale ----------
    {
        Csv c(dir + "/index_scale.csv");
        c.row({"case", "fn", "index_in", "scale", "result"});
        long long k = 0;
        for (int i = 0; i < 40; ++i) {
            double sc = (i % 3 == 0) ? 0.025 : (i % 3 == 1 ? 0.5 : 1.7);
            int ii = rng.irand(-20, 120);
            c.row({NI(k++), "index_scale_int", NI(ii), N(sc), NI(index_scale(ii, sc))});
            double dd = (i % 2 == 0) ? ii : ii + 0.25;
            c.row({NI(k++), "index_scale_double", N(dd), N(sc), NI(index_scale(dd, sc))});
            c.row({NI(k++), "scale_double", N(dd), N(sc), N(::scale(dd, sc))});
            Intex ix(ii, ii * 2);
            c.row({NI(k++), "intex_sx", NI(ii), N(sc), NI(ix.sx(sc))});
            c.row({NI(k++), "intex_sy", NI(ii * 2), N(sc), NI(ix.sy(sc))});
        }
        // 边界：0、负、整数倍
        for (double sc : {0.025, 0.5, 1.0, 2.0}) {
            for (int v : {0, 1, -1, 40, 1000}) {
                c.row({NI(k++), "index_scale_int", NI(v), N(sc), NI(index_scale(v, sc))});
            }
        }
        std::cout << "index_scale.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- scale_graph ----------
    {
        Csv c(dir + "/scale_graph.csv");
        Csv cc(dir + "/scale_graph_cells.csv");
        c.row({"case", "ground_id", "scale", "out_rows", "out_cols"});
        cc.row({"case", "ground_id", "scale", "x", "y", "value"});
        std::vector<double> scales = {0.1, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0};
        std::vector<int> gids = {0, 1, 2, 4, 5};
        long long k = 0;
        for (int gid : gids) {
            for (double sc : scales) {
                SqPlane out = gs[gid].map.scale_graph(sc);
                c.row({NI(k), NI(gid), N(sc), NI(out.rows()), NI(out.cols())});
                for (int x = 0; x < out.rows(); ++x)
                    for (int y = 0; y < out.cols(); ++y)
                        cc.row({NI(k), NI(gid), N(sc), NI(x), NI(y), N(out[x][y])});
                ++k;
            }
        }
        std::cout << "scale_graph.csv rows=" << c.rows - 1 << " cells_rows=" << cc.rows - 1 << "\n";
    }

    // ---------- steep_extend ----------
    {
        Csv c(dir + "/steep_extend.csv");
        c.row({"case", "ground_id", "x0", "y0", "x1", "y1", "result"});
        long long k = 0;
        auto emit = [&](int gid, Intex a, Intex b) {
            c.row({NI(k++), NI(gid), NI(a.x), NI(a.y), NI(b.x), NI(b.y),
                   N(steep_extend(gs[gid].map, a, b))});
        };
        emit(0, Intex(0, 0), Intex(4, 6));        // 全平 -> 0
        emit(0, Intex(0, 0), Intex(0, 0));        // 单点 -> 0
        emit(1, Intex(0, 0), Intex(7, 7));        // 斜面
        emit(2, Intex(0, 0), Intex(5, 8));        // 起伏
        emit(3, Intex(0, 0), Intex(6, 6));        // 含整列 infinity
        emit(6, Intex(0, 0), Intex(4, 4));        // 走廊：障碍占比高
        emit(6, Intex(2, 2), Intex(2, 2));        // 走廊单格
        emit(4, Intex(1, 1), Intex(4, 8));        // LCG 噪声
        emit(5, Intex(0, 0), Intex(9, 9));        // 斜面 + 负值障碍
        emit(0, Intex(-5, -5), Intex(-1, -1));    // 完全在界外 -> -1
        emit(0, Intex(100, 100), Intex(200, 200));// 完全在界外 -> -1
        for (int i = 0; i < 35; ++i) {
            int gid = rng.irand(0, 6);
            auto sh = gs[gid].shape();
            int x0 = rng.irand(-3, sh[0] + 2), x1 = rng.irand(-3, sh[0] + 2);
            int y0 = rng.irand(-3, sh[1] + 2), y1 = rng.irand(-3, sh[1] + 2);
            emit(gid, Intex(x0, y0), Intex(x1, y1));
        }
        std::cout << "steep_extend.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- geometric_decay ----------
    {
        Csv c(dir + "/geometric_decay.csv");
        std::vector<std::string> hdr = {"case", "fn", "alpha", "start", "n"};
        for (int i = 0; i < 10; ++i) hdr.push_back("c" + std::to_string(i));
        c.row(hdr);
        long long k = 0;
        auto emit = [&](const std::string& fn, double alpha, double start, int n) {
            std::vector<double> coef = (fn == "geometric_decay_start")
                                           ? geometric_decay(start, alpha, n)
                                           : geometric_decay(alpha, n);
            std::vector<std::string> cells = {NI(k++), fn, N(alpha), N(start), NI(n)};
            for (int i = 0; i < 10; ++i)
                cells.push_back(i < static_cast<int>(coef.size()) ? N(coef[i]) : "");
            c.row(cells);
        };
        emit("geometric_decay", std::sqrt(M_PI), 1.0, 1);
        emit("geometric_decay", std::sqrt(M_PI), 1.0, 27);   // main_test 实际用到的规模
        emit("geometric_decay", 1.0, 1.0, 5);                // alpha=1
        emit("geometric_decay", 3.0, 1.0, 5);                // direction_test 用到的
        emit("geometric_decay", 0.0, 1.0, 4);                // alpha=0
        emit("geometric_decay", std::sqrt(M_PI), 1.0, 0);    // n=0
        emit("geometric_decay_start", std::sqrt(M_PI), 2.5, 6);
        emit("geometric_decay_start", 1.0, 2.5, 6);
        for (int i = 0; i < 34; ++i) {
            double a = rng.uni(0.1, 4.0), st = rng.uni(0.5, 5.0);
            int n = rng.irand(1, 8);
            emit("geometric_decay", a, 1.0, n);
            emit("geometric_decay_start", a, st, n);
        }
        std::cout << "geometric_decay.csv rows=" << c.rows - 1 << "\n";
    }

    // ---------- direction_determine ----------
    {
        Csv c(dir + "/direction_determine.csv");
        Csv cg(dir + "/direction_guides.csv");
        c.row({"case", "mode", "atx", "aty", "near_end", "n", "outx", "outy"});
        cg.row({"case", "mode", "idx", "x", "y"});
        long long k = 0;
        auto emit = [&](const std::string& mode, SqDot at, const std::vector<SqDot>& guides,
                        bool near_end) {
            std::vector<SqDot> gd = guides;
            std::vector<Intex> gi;
            for (const auto& p : gd)
                gi.emplace_back(static_cast<int>(std::lround(p.x)),
                                static_cast<int>(std::lround(p.y)));
            for (size_t j = 0; j < gd.size(); ++j) {
                if (mode == "dot") cg.row({NI(k), mode, NI((long long)j), N(gd[j].x), N(gd[j].y)});
                else cg.row({NI(k), mode, NI((long long)j), NI(gi[j].x), NI(gi[j].y)});
            }
            SqDot out = (mode == "dot") ? direction_determine(at, gd, near_end)
                                        : direction_determine(at, gi, near_end);
            c.row({NI(k), mode, N(at.x), N(at.y), near_end ? "1" : "0", NI((long long)guides.size()),
                   N(out.x), N(out.y)});
            ++k;
        };
        emit("dot", SqDot(0, 0), {}, false);
        emit("dot", SqDot(0, 0), {}, true);
        emit("dot", SqDot(0, 0), {SqDot(10, 0)}, false);
        emit("dot", SqDot(0, 0), {SqDot(10, 0), SqDot(20, 0), SqDot(30, 0)}, false);
        emit("dot", SqDot(0, 0), {SqDot(10, 0), SqDot(20, 0), SqDot(30, 0)}, true);
        emit("dot", SqDot(5, 5), {SqDot(10, 10), SqDot(20, 15), SqDot(30, 20)}, false);
        for (int i = 0; i < 34; ++i) {
            int n = rng.irand(0, 6);
            std::vector<SqDot> gd;
            for (int j = 0; j < n; ++j) gd.emplace_back(SqDot(rng.uni(-20, 40), rng.uni(-20, 40)));
            SqDot at(rng.uni(-5, 5), rng.uni(-5, 5));
            bool ne = (i % 2 == 0);
            emit(i % 3 == 0 ? "int" : "dot", at, gd, ne);
        }
        std::cout << "direction_determine.csv rows=" << c.rows - 1
                  << " guides_rows=" << cg.rows - 1 << "\n";
    }

    std::cout << "done\n";
    return 0;
}
