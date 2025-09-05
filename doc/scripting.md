# 脚本系统说明

## 概述

本项目包含多个脚本，用于自动化数据生成、格式转换、构建、测试和部署等任务。脚本分为Python脚本和平台特定脚本（如PowerShell、Batch），以提供跨平台支持和良好的用户体验。

## 脚本目录结构

```
scripts/
├── data_generator/     # 数据生成脚本
│   ├── gen_map.m       # MATLAB版本的地图生成脚本
│   └── gen_map.py      # Python版本的地图生成脚本
├── transor/            # 数据格式转换脚本
│   └── jpg2csv.py      # JPEG到CSV格式转换脚本
├── sync/               # 同步脚本
│   ├── get-sync.py     # 获取同步脚本
│   └── put-sync.py     # 推送同步脚本
├── platform/           # 平台特定脚本
│   └── windows/        # Windows平台脚本
│       └── run_tests.bat # Windows批处理测试运行脚本
├── run_tests.py        # 跨平台测试运行脚本
└── deploy.py           # 跨平台部署脚本
```

## 脚本详细说明

### 1. 数据生成脚本

#### gen_map.py

位置：[scripts/data_generator/gen_map.py](../scripts/data_generator/gen_map.py)

功能：

- 生成模拟地形图
- 创建具有不同高度区域的地图
- 添加噪声以模拟真实地形
- 输出为JPEG格式文件

使用方法：

```bash
python scripts/data_generator/gen_map.py
```

输出：

- 输入文件：`data/input/map.jpg`

#### gen_map.m

位置：[scripts/data_generator/gen_map.m](../scripts/data_generator/gen_map.m)

功能：

- MATLAB版本的地图生成脚本
- 功能与Python版本相同

### 2. 数据格式转换脚本

#### jpg2csv.py

位置：[scripts/transor/jpg2csv.py](../scripts/transor/jpg2csv.py)

功能：

- 将JPEG格式的地图转换为CSV格式
- 支持反向转换以验证数据完整性
- 输出地图的基本信息（尺寸、数值范围等）

使用方法：

```bash
python scripts/transor/jpg2csv.py
```

输入：

- `data/input/map.jpg` - 输入的JPEG地图文件

输出：

- `data/csv/map.csv` - 输出的CSV文件
- `data/output/map_from_csv.jpg` - 验证用的反向转换JPEG文件

### 3. 同步脚本

#### get-sync.py 和 put-sync.py

位置：

- [scripts/sync/get-sync.py](../scripts/sync/get-sync.py)
- [scripts/sync/put-sync.py](../scripts/sync/put-sync.py)

功能：

- 用于与外部系统同步数据
- get-sync.py 用于获取外部数据
- put-sync.py 用于推送本地数据到外部系统

### 4. 测试运行脚本

#### run_tests.py

位置：[scripts/run_tests.py](../scripts/run_tests.py)

功能：

- 跨平台的测试运行脚本
- 自动化整个测试流程
- 适用于Linux、macOS和Windows (WSL)

流程：

1. 生成测试地图数据
2. 将JPEG转换为CSV格式
3. 配置项目（CMake）
4. 构建项目
5. 运行测试

使用方法：

```bash
python scripts/run_tests.py
```

## 脚本详细说明

### run_tests.bat

位置：[scripts/platform/windows/run_tests.bat](../scripts/platform/windows/run_tests.bat)

功能：

- Windows批处理版本的测试运行脚本
- 简单直接，兼容性好
- 自动处理控制台编码问题

使用方法：

```batch
scripts\platform\windows\run_tests.bat
```

## 不同类型脚本的特点和优势

### 1. Python脚本 (.py)

优势：

- 跨平台兼容性最好
- 丰富的库支持
- 易于维护和扩展
- 适合复杂逻辑处理

适用场景：

- 数据处理和转换
- 跨平台自动化流程
- 复杂的业务逻辑

### 2. 批处理脚本 (.bat/.cmd)

优势：

- 兼容性极好，所有Windows版本都支持
- 启动速度快
- 语法简单直观
- 不需要额外的运行环境

适用场景：

- 简单的命令执行序列
- 基础的自动化任务
- 需要最大兼容性

## 脚本开发规范

### Python脚本规范

1. 使用UTF-8编码
2. 包含shebang行（`#!/usr/bin/env python3`）
3. 添加编码声明（`# -*- coding: utf-8 -*-`）
4. 使用相对路径处理文件
5. 包含错误处理和日志输出
6. 遵循PEP 8代码风格

### 批处理脚本规范

1. 使用UTF-8编码或代码页处理中文
2. 包含清晰的注释
3. 正确处理错误和返回码
4. 使用相对路径
5. 提供用户友好的输出信息

## 跨平台兼容性

### 脚本选择指南

1. **首选Python脚本**：
   - 需要跨平台运行
   - 逻辑复杂
   - 需要丰富的库支持

2. **使用批处理脚本**：
   - 简单的命令执行
   - 需要最大兼容性
   - 不想依赖额外运行环境

### 编码处理

所有脚本都应正确处理中文字符编码：

1. Python脚本中设置编码：

```python
import sys
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')
```

2. 批处理脚本中设置编码：

```batch
chcp 65001
```

## 最佳实践

### 1. 脚本调用

推荐使用以下方式调用脚本：

Windows:

```batch
# 使用批处理（推荐）
scripts\platform\windows\run_tests.bat

# 或使用Python
python scripts\run_tests.py
```

Linux/macOS:

```bash
# 测试
python3 scripts/run_tests.py
```

### 2. 错误处理

脚本应包含完善的错误处理机制：

1. 检查命令执行结果
2. 提供清晰的错误信息
3. 在出错时使用非零退出码
4. 记录关键操作日志

## 常见问题

### 1. 控制台乱码问题

问题：在命令行中运行脚本时出现中文乱码

解决方案：

- 使用批处理脚本并运行`chcp 65001`切换到UTF-8编码
- 或在Python脚本中正确设置编码

### 2. 路径问题

问题：在不同平台下路径分隔符不一致

解决方案：

- Python脚本使用`os.path.join()`或`pathlib`
- 批处理脚本使用`%~dp0`获取脚本目录

### 3. 权限问题

问题：脚本无法执行

解决方案：

- 确保脚本具有执行权限
