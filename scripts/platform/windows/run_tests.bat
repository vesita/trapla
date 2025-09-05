@echo off
REM Windows批处理脚本用于运行测试
REM 该脚本执行完整的测试流程，包括：
REM 1. 生成测试数据
REM 2. 转换数据格式
REM 3. 配置和构建项目
REM 4. 运行单元测试

REM 设置控制台编码为UTF-8以支持中文显示
chcp 65001 >nul

REM 获取项目根目录
REM %~dp0 表示当前脚本所在的目录路径
set PROJECT_ROOT=%~dp0..\..
set BUILD_DIR=%PROJECT_ROOT%\build

echo 项目根目录: %PROJECT_ROOT%
echo 构建目录: %BUILD_DIR%

REM 确保构建目录存在，如果不存在则创建
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

REM 生成地图数据
echo.
echo === 生成地图数据 ===
echo 正在执行地图数据生成脚本...
python "%PROJECT_ROOT%\scripts\data_generator\gen_map.py"
if %errorlevel% neq 0 (
    echo [错误] 生成地图数据失败，错误代码: %errorlevel%
    echo 请检查以下内容：
    echo - Python是否已正确安装
    echo - 脚本路径是否正确
    echo - 脚本是否有执行权限
    pause
    exit /b 1
)

REM 转换JPEG到CSV
echo.
echo === 转换地图数据格式 ===
echo 正在将JPEG格式的地图数据转换为CSV格式...
python "%PROJECT_ROOT%\scripts\transor\jpg2csv.py"
if %errorlevel% neq 0 (
    echo [错误] 转换地图数据格式失败，错误代码: %errorlevel%
    echo 请检查：
    echo - 输入文件是否存在
    echo - 输入文件格式是否正确
    echo - 脚本是否完整
    pause
    exit /b 1
)

REM 配置项目
echo.
echo === 配置项目 ===
echo 正在使用CMake配置项目...
cd /d "%BUILD_DIR%"
cmake ..
if %errorlevel% neq 0 (
    echo [错误] CMake配置失败，错误代码: %errorlevel%
    echo 请检查：
    echo - CMake是否已正确安装
    echo - CMakeLists.txt是否完整
    echo - 是否缺少必要的依赖项
    pause
    exit /b 1
)

REM 构建项目
echo.
echo === 构建项目 ===
echo 正在构建项目...
cmake --build .
if %errorlevel% neq 0 (
    echo [错误] 项目构建失败，错误代码: %errorlevel%
    echo 请检查：
    echo - 源代码是否有编译错误
    echo - 是否缺少必要的库文件
    echo - 编译器配置是否正确
    pause
    exit /b 1
)

REM 运行测试
echo.
echo === 运行测试 ===
echo 正在查找测试可执行文件...
set TEST_EXE=%BUILD_DIR%\foot_test.exe
if exist "%TEST_EXE%" (
    echo 测试可执行文件已找到: %TEST_EXE%
    echo 正在运行测试...
    "%TEST_EXE%"
    if %errorlevel% neq 0 (
        echo [错误] 测试运行失败，错误代码: %errorlevel%
        echo 请检查：
        echo - 测试代码是否有错误
        echo - 测试依赖是否完整
        echo - 测试环境是否正确配置
        pause
        exit /b 1
    )
) else (
    echo [错误] 测试可执行文件不存在: %TEST_EXE%
    echo 请检查：
    echo - 测试是否已正确编译
    echo - 构建目录是否正确
    echo - 是否缺少必要的构建步骤
    pause
    exit /b 1
)

echo.
echo === 所有测试通过 ===
echo 测试流程已完成，所有测试通过。
echo 当前状态：✅ 测试成功
pause