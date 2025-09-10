#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import shutil
from pathlib import Path
import argparse
import platform
import time
import multiprocessing
from concurrent.futures import ThreadPoolExecutor, as_completed

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

def build_tests(verbose=False):
    """构建测试程序"""
    print("正在构建测试程序...")
    
    # 进入build目录
    if not Path("build").exists():
        Path("build").mkdir()
    
    os.chdir("build")
    
    # 配置项目
    cmake_cmd = ["cmake", ".."]
    if verbose:
        cmake_config_result = subprocess.run(cmake_cmd)
    else:
        cmake_config_result = subprocess.run(cmake_cmd, 
                                             capture_output=True, 
                                             text=True,
                                             encoding='utf-8')
    if cmake_config_result.returncode != 0:
        print("CMake配置失败:")
        if verbose:
            print("请查看上面的输出以获取详细错误信息")
        else:
            print(cmake_config_result.stderr)
            print("\n可能的原因:")
            print("1. 系统未安装C++编译器(Visual Studio或MinGW)")
            print("2. CMake未正确配置编译器")
            if platform.system() == "Windows":
                print("3. 未使用Visual Studio开发者命令提示运行")
                print("\n解决方案:")
                print("- 安装Visual Studio Community并选择'C++桌面开发'工作负载")
                print("- 或安装Visual Studio Build Tools")
                print("- 或使用'开发者命令提示'运行此脚本")
            else:
                print("3. 确保已安装g++或clang++编译器")
        os.chdir("..")
        return False
    
    # 构建项目
    build_cmd = ["cmake", "--build", "."]
    if verbose:
        cmake_build_result = subprocess.run(build_cmd)
    else:
        cmake_build_result = subprocess.run(build_cmd, 
                                            capture_output=True, 
                                            text=True,
                                            encoding='utf-8')
    if cmake_build_result.returncode != 0:
        print("构建失败:")
        if verbose:
            print("请查看上面的输出以获取详细错误信息")
        else:
            print(cmake_build_result.stderr)
        os.chdir("..")
        return False
    
    os.chdir("..")
    print("测试程序构建成功")
    return True

def run_single_test(test_name, working_dir, verbose=False):
    """运行单个测试"""
    print(f"正在运行测试: {test_name}")
    
    # 根据操作系统确定可执行文件扩展名
    exe_extension = ".exe" if sys.platform == "win32" else ""
    test_exe = f"build/{test_name}{exe_extension}"
    
    if not Path(test_exe).exists():
        print(f"测试可执行文件不存在: {test_exe}")
        return False, "", 0
    
    # 运行测试，传递工作目录作为命令行参数
    start_time = time.time()
    if verbose:
        result = subprocess.run([test_exe, working_dir])
    else:
        result = subprocess.run([test_exe, working_dir], capture_output=True, text=True, encoding='utf-8')
    end_time = time.time()
    
    duration = end_time - start_time
    
    # 输出结果
    output = ""
    if verbose:
        # 在verbose模式下，输出已经在控制台显示
        pass
    else:
        output = result.stdout
        if result.stderr:
            output += "\n错误输出: " + result.stderr
    
    return result.returncode == 0, output, duration

def display_test_menu(available_tests):
    """显示测试选择菜单"""
    print("\n可用的测试:")
    for i, test_name in enumerate(available_tests, 1):
        print(f"{i}. {test_name}")
    print("q. 退出")

def run_tests_parallel(test_names, working_dir, max_workers=None, verbose=False):
    """并行运行测试"""
    if max_workers is None:
        max_workers = min(len(test_names), multiprocessing.cpu_count())
    
    print(f"\n将使用 {max_workers} 个线程并行运行 {len(test_names)} 个测试")
    
    passed_tests = []
    failed_tests = []
    test_durations = {}
    
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        # 提交所有测试任务
        future_to_test = {
            executor.submit(run_single_test, test_name, working_dir, verbose): test_name 
            for test_name in test_names
        }
        
        # 收集结果
        for future in as_completed(future_to_test):
            test_name = future_to_test[future]
            try:
                success, output, duration = future.result()
                test_durations[test_name] = duration
                if success:
                    passed_tests.append(test_name)
                    print(f"✓ {test_name} 测试通过 (耗时: {duration:.2f}s)")
                else:
                    failed_tests.append(test_name)
                    print(f"✗ {test_name} 测试失败 (耗时: {duration:.2f}s)")
                    if not verbose and output:
                        print(f"  输出:\n{output}")
            except Exception as e:
                failed_tests.append(test_name)
                print(f"✗ {test_name} 运行时出错: {e}")
    
    return passed_tests, failed_tests, test_durations

def main():
    """主函数"""
    # 创建参数解析器
    parser = argparse.ArgumentParser(description='测试运行器')
    parser.add_argument('tests', nargs='*', help='要运行的测试名称，使用"all"运行所有测试')
    parser.add_argument('-p', '--project-path', help='设置项目路径')
    parser.add_argument('-v', '--verbose', action='store_true', help='显示详细输出')
    parser.add_argument('-j', '--jobs', type=int, help='并行运行的测试数量')
    parser.add_argument('--filter', help='使用正则表达式过滤测试名称')
    # 解析参数
    args = parser.parse_args()
    
    print("欢迎使用测试运行器")
    
    # 确定项目根目录
    if args.project_path:
        project_root = Path(args.project_path).resolve()
        print(f"使用指定的项目路径: {project_root}")
    else:
        # 获取项目根目录（当前脚本的上两级目录）
        script_path = Path(__file__).resolve()
        project_root = script_path.parent.parent
        print(f"使用默认项目路径: {project_root}")
    
    # 清理log目录
    clean_log_directory()
    
    # 获取可用测试
    available_tests = get_available_tests()
    if not available_tests:
        print("未找到测试文件")
        return
    
    # 应用测试过滤器
    if args.filter:
        import re
        pattern = re.compile(args.filter)
        available_tests = [test for test in available_tests if pattern.search(test)]
        print(f"过滤后的测试: {', '.join(available_tests)}")
    
    # 构建测试程序
    if not build_tests(args.verbose):
        print("构建测试程序失败")
        return
    
    # 确定要运行的测试
    if args.tests:
        # 移除了对"all"的特殊处理，现在"all"会被当作普通测试名称处理（但不会匹配任何测试）
        selected_tests = [test for test in args.tests if test in available_tests]
        if not selected_tests:
            print("未找到指定的测试")
            return
    else:
        # 交互式选择测试
        while True:
            display_test_menu(available_tests)
            choice = input("\n请选择要运行的测试 (默认执行所有测试，输入序号，多个序号用空格分隔，或'q'退出): ").strip()
            
            if choice.lower() == 'q':
                print("退出测试程序")
                return
            
            # 如果输入为空或仅为空格，则运行所有测试
            if not choice or choice.strip() == '':
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
    
    start_time = time.time()
    if len(selected_tests) > 1:
        # 并行运行多个测试
        passed_tests, failed_tests, test_durations = run_tests_parallel(
            selected_tests, str(project_root), args.jobs, args.verbose
        )
    else:
        # 顺序运行单个测试
        passed_tests = []
        failed_tests = []
        test_durations = {}
        
        for test_name in selected_tests:
            try:
                success, output, duration = run_single_test(test_name, str(project_root), args.verbose)
                test_durations[test_name] = duration
                if success:
                    passed_tests.append(test_name)
                    print(f"✓ {test_name} 测试通过 (耗时: {duration:.2f}s)")
                else:
                    failed_tests.append(test_name)
                    print(f"✗ {test_name} 测试失败 (耗时: {duration:.2f}s)")
                    if not args.verbose and output:
                        print(f"  输出:\n{output}")
            except Exception as e:
                failed_tests.append(test_name)
                print(f"✗ {test_name} 运行时出错: {e}")
    
    end_time = time.time()
    total_duration = end_time - start_time
    
    # 输出总结
    print("\n" + "="*80)
    print("测试运行总结:")
    print(f"总测试数: {len(selected_tests)}")
    print(f"通过: {len(passed_tests)}")
    print(f"失败: {len(failed_tests)}")
    print(f"总耗时: {total_duration:.2f}秒")
    
    if passed_tests:
        print(f"\n通过的测试:")
        for test in passed_tests:
            print(f"  ✓ {test} ({test_durations[test]:.2f}s)")
    
    if failed_tests:
        print(f"\n失败的测试:")
        for test in failed_tests:
            print(f"  ✗ {test} ({test_durations[test]:.2f}s)")
    
    if failed_tests:
        sys.exit(1)
    else:
        print("\n所有测试均已通过!")

if __name__ == "__main__":
    main()