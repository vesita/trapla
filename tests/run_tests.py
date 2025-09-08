#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import shutil
from pathlib import Path

def clean_log_directory():
    """清理log目录"""
    log_dir = Path("log")
    if log_dir.exists():
        shutil.rmtree(log_dir)
    log_dir.mkdir(exist_ok=True)
    print("已清理并重新创建log目录")

def get_available_tests():
    """获取所有可用的测试文件"""
    test_files = list(Path("tests").glob("*_test.cpp"))
    return [f.stem for f in test_files]

def build_tests():
    """构建测试程序"""
    print("正在构建测试程序...")
    
    # 进入build目录
    if not Path("build").exists():
        Path("build").mkdir()
    
    os.chdir("build")
    
    # 配置项目
    cmake_config_result = subprocess.run(["cmake", ".."], 
                                         capture_output=True, 
                                         text=True,
                                         encoding='utf-8')
    if cmake_config_result.returncode != 0:
        print("CMake配置失败:")
        print(cmake_config_result.stderr)
        print("\n可能的原因:")
        print("1. 系统未安装C++编译器(Visual Studio或MinGW)")
        print("2. CMake未正确配置编译器")
        print("3. 未使用Visual Studio开发者命令提示运行")
        print("\n解决方案:")
        print("- 安装Visual Studio Community并选择'C++桌面开发'工作负载")
        print("- 或安装Visual Studio Build Tools")
        print("- 或使用'开发者命令提示'运行此脚本")
        os.chdir("..")
        return False
    
    # 构建项目
    cmake_build_result = subprocess.run(["cmake", "--build", "."], 
                                        capture_output=True, 
                                        text=True,
                                        encoding='utf-8')
    if cmake_build_result.returncode != 0:
        print("构建失败:")
        print(cmake_build_result.stderr)
        os.chdir("..")
        return False
    
    os.chdir("..")
    print("测试程序构建成功")
    return True

def run_single_test(test_name, working_dir):
    """运行单个测试"""
    print(f"正在运行测试: {test_name}")
    
    # 根据操作系统确定可执行文件扩展名
    exe_extension = ".exe" if sys.platform == "win32" else ""
    test_exe = f"build/{test_name}{exe_extension}"
    
    if not Path(test_exe).exists():
        print(f"测试可执行文件不存在: {test_exe}")
        return False
    
    # 运行测试，传递工作目录作为命令行参数
    result = subprocess.run([test_exe, working_dir], capture_output=True, text=True, encoding='utf-8')
    
    # 输出结果
    print(result.stdout)
    if result.stderr:
        print("错误输出:", result.stderr)
    
    return result.returncode == 0

def display_test_menu(available_tests):
    """显示测试选择菜单"""
    print("\n可用的测试:")
    for i, test_name in enumerate(available_tests, 1):
        print(f"{i}. {test_name}")
    print("all. 运行所有测试")
    print("q. 退出")

def main():
    """主函数"""
    print("欢迎使用测试运行器")
    
    # 获取项目根目录（当前脚本的上两级目录）
    script_path = Path(__file__).resolve()
    project_root = script_path.parent.parent
    
    # 清理log目录
    clean_log_directory()
    
    # 获取可用测试
    available_tests = get_available_tests()
    if not available_tests:
        print("未找到测试文件")
        return
    
    # 构建测试程序
    if not build_tests():
        print("构建测试程序失败")
        return
    
    # 检查命令行参数
    if len(sys.argv) > 1:
        selected_tests = sys.argv[1:]
        if "all" in selected_tests:
            selected_tests = available_tests
    else:
        # 交互式选择测试
        while True:
            display_test_menu(available_tests)
            choice = input("\n请选择要运行的测试 (输入序号或'all'或'q'退出): ").strip()
            
            if choice.lower() == 'q':
                print("退出测试程序")
                return
            
            if choice.lower() == 'all':
                selected_tests = available_tests
                break
            
            try:
                indices = [int(x.strip()) for x in choice.split()]
                selected_tests = [available_tests[i-1] for i in indices if 1 <= i <= len(available_tests)]
                if not selected_tests:
                    print("未选择有效测试，请重新选择")
                    continue
                break
            except (ValueError, IndexError):
                print("输入无效，请重新输入")
                continue
    
    # 运行选中的测试
    print(f"\n将运行以下测试: {', '.join(selected_tests)}")
    
    passed_tests = []
    failed_tests = []
    
    for test_name in selected_tests:
        try:
            if run_single_test(test_name, str(project_root)):
                passed_tests.append(test_name)
                print(f"✓ {test_name} 测试通过")
            else:
                failed_tests.append(test_name)
                print(f"✗ {test_name} 测试失败")
        except Exception as e:
            failed_tests.append(test_name)
            print(f"✗ {test_name} 运行时出错: {e}")
    
    # 输出总结
    print("\n" + "="*50)
    print("测试运行总结:")
    print(f"总测试数: {len(selected_tests)}")
    print(f"通过: {len(passed_tests)}")
    print(f"失败: {len(failed_tests)}")
    
    if passed_tests:
        print(f"通过的测试: {', '.join(passed_tests)}")
    
    if failed_tests:
        print(f"失败的测试: {', '.join(failed_tests)}")
    
    if failed_tests:
        sys.exit(1)
    else:
        print("所有测试均已通过!")

if __name__ == "__main__":
    main()