#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import subprocess
import sys
import os

def run_command(command, cwd=None):
    """运行命令并检查返回码，返回 True 表示成功，False 表示失败"""
    print(f"▶▶▶ 正在执行命令: {command}")
    try:
        # 设置环境变量以支持 UTF-8 编码（特别是 Windows 上）
        env = os.environ.copy()
        env['PYTHONIOENCODING'] = 'utf-8'
        
        result = subprocess.run(
            command, 
            shell=True, 
            cwd=cwd, 
            check=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE, 
            text=True,
            encoding='utf-8',
            env=env  # 明确使用带编码设置的环境变量
        )
        if result.stdout:
            print("STDOUT:\n", result.stdout)
        if result.stderr:
            print("STDERR:\n", result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ 命令执行失败: {e}")
        if e.stdout:
            print("STDOUT:\n", e.stdout)
        if e.stderr:
            print("STDERR:\n", e.stderr)
        return False

def main():
    # 获取项目根目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    build_dir = os.path.join(project_root, "build")
    
    print(f"项目根目录: {project_root}")
    print(f"构建目录: {build_dir}")
    
    # 确保构建目录存在
    os.makedirs(build_dir, exist_ok=True)
    
    # 阶段 1: 生成地图数据
    print("\n=== 阶段 1: 生成地图数据 ===")
    gen_script = os.path.join(project_root, "scripts", "data_generator", "gen_map.py")
    if not os.path.exists(gen_script):
        print(f"❌ 生成地图脚本不存在: {gen_script}")
        return 1
        
    if not run_command(f"python \"{gen_script}\""):
        print("❌ 地图数据生成失败，停止流程")
        return 1
    
    # 阶段 2: 转换地图数据格式 (JPEG -> CSV)
    print("\n=== 阶段 2: 转换地图数据格式 (JPEG -> CSV) ===")
    transor_script = os.path.join(project_root, "scripts", "transor", "jpg2csv.py")
    if not os.path.exists(transor_script):
        print(f"❌ 转换脚本不存在: {transor_script}")
        return 1
        
    if not run_command(f"python \"{transor_script}\""):
        print("❌ 地图数据格式转换失败，停止流程")
        return 1
    
    # 阶段 3: 配置 CMake 项目
    print("\n=== 阶段 3: 配置 CMake 项目 ===")
    if not run_command("cmake ..", cwd=build_dir):
        print("❌ CMake 配置失败，停止流程")
        return 1
    
    # 阶段 4: 构建项目
    print("\n=== 阶段 4: 构建项目 ===")
    if not run_command("cmake --build .", cwd=build_dir):
        print("❌ 项目构建失败，停止流程")
        return 1
    
    # 阶段 5: 运行单元测试
    print("\n=== 阶段 5: 运行单元测试 ===")
    
    # 定义所有测试可执行文件
    test_executables = ["foot_test", "aStar_test", "ground_test"]
    
    all_tests_passed = True
    
    for test_name in test_executables:
        print(f"\n--- 运行 {test_name} 测试 ---")
        test_exe = os.path.join(build_dir, test_name)
        if sys.platform == "win32":
            test_exe += ".exe"
        
        if not os.path.exists(test_exe):
            print(f"❌ 测试可执行文件不存在: {test_exe}")
            all_tests_passed = False
            continue
        
        if not run_command(f"\"{test_exe}\""):
            print(f"❌ {test_name} 测试运行失败")
            all_tests_passed = False
        else:
            print(f"✅ {test_name} 测试通过")
    
    if all_tests_passed:
        print("\n=== 所有测试通过 ===")
        return 0
    else:
        print("\n=== 部分测试失败 ===")
        return 1

if __name__ == "__main__":
    # 设置标准输出和错误输出的编码
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8')
        
    # 在Windows上设置环境变量
    if sys.platform == "win32":
        os.environ['PYTHONIOENCODING'] = 'utf-8'
        
    sys.exit(main())