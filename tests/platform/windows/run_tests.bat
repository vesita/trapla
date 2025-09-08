@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

cd /d "%~dp0\..\..\.."

echo 正在运行测试套件...

if "%1"=="" (
    python tests\run_tests.py
) else (
    python tests\run_tests.py %*
)

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo 测试执行失败，错误代码: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
) else (
    echo.
    echo 测试执行完成。
    pause
)
