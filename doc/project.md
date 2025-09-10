# 双足机器人在线落足点规划项目说明

## 1. 项目概述

双足机器人在不平坦的路面上运动时，其双足交替移动会受到多种因素的制约，包括步长、足平面法向量方向与重力方向夹角、观测范围、实际落足点与规划落足点的偏差等。本项目旨在解决给定起点和终点位置的情况下，如何在线规划落足点位置的问题。

## 2. 系统需求

### 2.1 输入条件

- **三维地形图**：提供机器人运动环境的三维地形信息
- **足尺寸**：3×5（单位：cm）
- **步长限制**：最大值40（单位：cm）
- **双足间距**：大于2且小于10（单位：cm）
- **观测范围**：运动方向前方200×200区域（单位：cm）
- **最大转向角**：小于±75°
- **足平面法向量与重力方向夹角**：小于±20°
- **起点和终点的投影坐标**：在二维平面上的起始和目标位置

### 2.2 输出要求

1. **双足落足点序列**：
   - 足中心位置坐标
   - 足平面法向量方向与重力方向夹角
   - 支持可视化展示

2. **轨迹长度**：
   - 计算并输出整个运动轨迹的总长度

3. **转向角曲线**：
   - 绘制机器人在运动过程中的转向角变化曲线

4. **足平面法向量方向与重力方向夹角曲线**：
   - 绘制足平面法向量方向与重力方向夹角的变化曲线

5. **实验数据分析和安全性分析**：
   - 对实验数据进行统计分析
   - 评估路径的安全性，包括稳定性、可行性等指标

## 3. 系统设计

### 3.1 对象特征及约束

#### 3.1.1 双足机器人 (Bipedal Robot)

**属性**：

- 足尺寸：3×5 cm
- 步长：最大值40 cm
- 双足间距：大于2 cm且小于10 cm
- 观测范围：运动方向前方200×200 cm区域
- 最大转向角：小于±75°
- 足平面法向量与重力方向夹角：小于±20°

**约束条件**：

- 步长不能超过40 cm
- 双足间距必须在2-10 cm范围内
- 转向角不能超过±75°
- 足平面法向量与重力方向夹角不能超过±20°

#### 3.1.2 环境地形 (Terrain)

**属性**：

- 三维地形图：提供机器人运动环境的三维地形信息
- 起点和终点的投影坐标：在二维平面上的起始和目标位置

**约束条件**：

- 机器人只能在给定的三维地形图范围内移动
- 必须从指定起点移动到指定终点

#### 3.1.3 路径规划 (Path Planning)

**属性**：

- 双足落足点序列：包含足中心位置坐标和足平面法向量方向与重力方向夹角
- 轨迹长度：整个运动轨迹的总长度
- 转向角曲线：机器人在运动过程中的转向角变化
- 足平面法向量方向与重力方向夹角曲线：足平面法向量方向与重力方向夹角的变化

**约束条件**：

- 落足点必须在机器人的观测范围内
- 落足点必须满足机器人物理限制（步长、角度等）
- 路径必须保证机器人稳定性和可行性

### 3.2 算法设计

#### 3.2.1 分层路径规划

本项目采用分层路径规划策略：

1. **高层规划**：在缩放地图上使用A*算法快速找到粗略路径
2. **中层规划**：将高层路径点映射回原始地图坐标
3. **底层规划**：根据机器人物理约束生成具体的足部落点序列

#### 3.2.2 A*搜索算法

使用改进的A*算法进行路径搜索，考虑地形代价和机器人约束：

- **启发式函数**：使用欧几里得距离作为启发式函数
- **代价函数**：综合考虑地形复杂度和移动代价
- **节点扩展**：在满足约束条件下扩展可行节点

#### 3.2.3 足部姿态计算

通过三点迭代算法计算足部与地面接触的最佳拟合平面：

- **点云采样**：获取足部覆盖区域内的地面点云数据
- **平面拟合**：使用三点迭代算法计算最佳接触平面
- **姿态评估**：计算足部法向量与重力方向的夹角

#### 3.2.4 双足间距约束检查

双足间距约束检查采用几何计算方法：

- **近侧外形线**：定义支撑脚靠近摆动脚一侧的长边所在的直线
- **距离计算**：计算摆动脚四角到近侧外形线的垂直距离
- **最小距离**：选取四角中到近侧外形线的最小距离
- **实际间距**：最小距离减去支撑脚半宽得到实际双足间距
- **约束验证**：验证计算出的间距是否在2-10单位允许范围内

## 4. 技术实现

### 4.1 核心数据结构

- **SqDot**：二维整数坐标点，用于表示地图网格位置
- **CuDot**：三维浮点坐标点，用于表示空间位置
- **SqPlain**：二维地图结构，包含地形代价信息
- **CuPlain**：三维平面结构，用于表示接触平面

### 4.2 核心算法模块

- **A*搜索模块**：实现路径规划算法
- **地形处理模块**：处理三维地形数据
- **足部姿态模块**：计算足部与地面的接触姿态
- **约束检查模块**：验证机器人运动约束

### 4.3 测试框架

项目包含完整的测试框架，确保各模块功能正确：

- **单元测试**：针对各个模块进行独立测试
- **集成测试**：测试模块间的协同工作
- **性能测试**：评估算法性能和效率

## 5. 代码结构说明

### 5.1 源代码目录结构

```
src/
├── aStar/              # A*算法实现
│   └── aStar.cpp       # A*算法核心实现
├── csvReader/          # CSV文件读取器
│   └── reader.cpp      # CSV数据读取实现
├── ground/             # 地面处理模块
│   └── ground.cpp      # 地面数据处理实现
├── robot/              # 机器人相关模块
│   ├── foot.cpp        # 足部相关实现
│   └── robot.cpp       # 机器人行为实现
├── utils/              # 工具模块
│   ├── geometry.cpp    # 几何计算实现
│   ├── fast_flatness.cpp # 快速平整度评估实现
│   └── scale.cpp       # 缩放功能实现
└── main.cpp            # 主程序入口

```

### 5.2 头文件目录结构

```
include/
├── aStar/
│   └── aStar.hpp       # A*算法头文件
├── csvReader/
│   └── reader.hpp      # CSV读取器头文件
├── ground/
│   └── ground.hpp      # 地面处理头文件
├── robot/
│   ├── foot.hpp        # 足部相关头文件
│   └── robot.hpp       # 机器人相关头文件
├── utils/
│   ├── geometry.hpp    # 几何计算头文件
│   ├── fast_flatness.hpp # 快速平整度评估头文件
│   ├── scale.hpp       # 缩放功能头文件
│   ├── test_framework.hpp # 测试框架头文件
│   └── io.hpp          # IO管理头文件
└── prepare.hpp         # 预处理头文件
```

### 5.3 测试目录结构

```
tests/
├── aStar_test.cpp      # A*算法测试
├── ground_test.cpp     # 地面处理测试
├── utils_test.cpp      # 工具模块测试
├── comparison_test.cpp # 对比测试
└── run_tests.py        # 测试运行脚本

```

## 6. 编译和构建

项目使用CMake作为构建系统，支持跨平台构建。

### 6.1 构建步骤

6.1.1. 创建构建目录：

   ```bash
   mkdir build
   cd build
   ```

6.1.2. 配置项目：

   ```bash
   cmake ..
   ```

6.1.3. 编译项目：

   ```bash
   make
   ```

### 6.2 生成的可执行文件

构建完成后会生成以下可执行文件：

- `trapla`：主程序
- `aStar_test`：A*算法测试程序
- `ground_test`：地面处理测试程序
- `utils_test`：工具模块测试程序
- `foot_test`：足部相关测试程序
- `comparison_test`：对比测试程序
- `constraints_test`：约束条件测试程序

## 7. 运行和测试

### 7.1 运行主程序

```bash
./trapla
```

### 7.2 运行测试

在项目根目录下执行：

```bash
python tests/run_tests.py
```

或者在Windows系统上：

```cmd
scripts\platform\windows\run_tests.bat
```

### 7.3 测试框架使用说明

项目采用统一的测试框架，具有以下特点：

1. 使用[TestFramework](../../include/utils/test_framework.hpp)类管理测试用例执行
2. 使用[TEST](../../include/utils/test_framework.hpp)宏定义测试用例
3. 集成测试失败数据收集和CSV输出功能
4. 自动将失败数据写入CSV文件，便于分析
5. 统一的测试报告格式
6. 支持详细日志记录

使用示例：

```cpp
TEST(example_test) {
    // 定义测试用例
    std::vector<std::tuple<InputType, ExpectedType>> test_cases = {
        {input1, expected1},
        {input2, expected2}
    };

    // 获取测试框架实例
    auto& framework = TestFramework::getInstance();
    const std::string testName = "测试名称";
    
    // 执行测试
    for (const auto& test_case : test_cases) {
        // 执行测试逻辑
        auto result = function_to_test(input);
        
        // 检查结果
        if (result != expected) {
            // 收集失败数据
            framework.addFailure(testName, {input, expected, result});
        }
    }
    
    // 定义列名
    std::vector<std::string> columnNames = {"input", "expected", "result"};
    
    // 写入失败数据到CSV文件
    framework.writeFailures(testName, "failure_data.csv", columnNames);
    
    // 如果有失败的测试用例，则抛出异常
    framework.throwIfFailed(testName, "测试失败");
}

```

所有测试都遵循统一的模式：

1. 定义测试用例集合
2. 获取[TestFramework](../../include/utils/test_framework.hpp)实例
3. 定义测试名称
4. 执行测试逻辑并收集失败用例
5. 定义CSV列名
6. 调用[writeFailures](../../include/utils/test_framework.hpp)方法将失败数据写入CSV文件
7. 调用[throwIfFailed](../../include/utils/test_framework.hpp)方法在有失败用例时抛出异常

这种统一的测试风格可以：

- 更好地收集和分析失败的测试用例
- 自动生成CSV报告便于调试
- 提供一致的测试接口和错误处理机制
- 提高测试代码的可维护性

### 7.4 测试框架组件详解

#### 7.4.1 TestFramework类

[TestFramework](../../include/utils/test_framework.hpp)是测试框架的核心类，负责管理测试用例的注册和执行。

主要方法：

- [getInstance()](../../include/utils/test_framework.hpp) - 获取单例实例
- [setWorkingDirectory()](../../include/utils/test_framework.hpp) - 设置工作目录
- [setLogFile()](../../include/utils/test_framework.hpp) - 设置日志文件
- [addTest()](../../include/utils/test_framework.hpp) - 添加测试用例
- [log()](../../include/utils/test_framework.hpp) - 记录日志信息
- [addFailure()](../../include/utils/test_framework.hpp) - 添加失败数据
- [hasFailures()](../../include/utils/test_framework.hpp) - 检查是否有失败数据
- [failureCount()](../../include/utils/test_framework.hpp) - 获取失败数据数量
- [writeFailures()](../../include/utils/test_framework.hpp) - 将失败数据写入CSV文件
- [throwIfFailed()](../../include/utils/test_framework.hpp) - 如果有失败数据则抛出异常
- [clearFailures()](../../include/utils/test_framework.hpp) - 清除指定测试的失败数据
- [runTests()](../../include/utils/test_framework.hpp) - 运行所有测试用例

#### 7.4.2 TEST宏

[TEST](../../include/utils/test_framework.hpp)宏用于定义测试用例，自动注册到[TestFramework](../../include/utils/test_framework.hpp)中。

使用方法：

```cpp
TEST(test_name) {
    // 测试代码
}
```

#### 7.4.3 TestFailureCollector类（已集成到TestFramework）

TestFailureCollector类的功能已集成到TestFramework类中，提供更简洁的接口用于失败数据收集和报告生成。

### 7.5 测试运行方式

在项目根目录下执行：

```bash
python tests/run_tests.py
```

或者在Windows系统上：

```cmd
scripts\platform\windows\run_tests.bat
```
