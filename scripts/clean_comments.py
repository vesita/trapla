#!/usr/bin/env python3
"""
清理C++源文件中的注释脚本
"""

import os
import re

def remove_comments_from_file(file_path):
    """
    从单个C++文件中移除注释
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 先处理多行注释 /* ... */
    # 使用非贪婪匹配以确保正确处理多个注释块
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    
    # 处理单行注释 //
    # 注意：需要避免误删如URL中的//部分
    lines = content.split('\n')
    new_lines = []
    
    for line in lines:
        # 查找不在字符串中的注释
        in_string = False
        quote_char = None
        i = 0
        comment_pos = -1
        
        while i < len(line):
            char = line[i]
            
            # 检查引号
            if char in ['"', "'"] and (i == 0 or line[i-1] != '\\'):
                if not in_string:
                    in_string = True
                    quote_char = char
                elif char == quote_char:
                    in_string = False
                    quote_char = None
            # 检查注释符号
            elif char == '/' and i + 1 < len(line) and line[i+1] == '/' and not in_string:
                comment_pos = i
                break
            
            i += 1
        
        # 如果找到了注释，则截断该行
        if comment_pos != -1:
            new_lines.append(line[:comment_pos].rstrip())
        else:
            new_lines.append(line)
    
    # 重新组合内容
    new_content = '\n'.join(new_lines)
    
    # 移除多余的空行（超过2个连续的空行）
    new_content = re.sub(r'\n{3,}', '\n\n', new_content)
    
    # 写回文件
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

def clean_all_cpp_files(root_dir):
    """
    清理项目中所有C++文件的注释
    """
    cpp_extensions = ['.cpp', '.hpp', '.h', '.cc', '.cxx']
    
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if any(file.endswith(ext) for ext in cpp_extensions):
                file_path = os.path.join(root, file)
                print(f"Cleaning comments from {file_path}")
                remove_comments_from_file(file_path)

if __name__ == "__main__":
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.join(script_dir, "..")
    
    # 定义需要清理的目录
    dirs_to_clean = [
        os.path.join(project_root, "src"),
        os.path.join(project_root, "include"),
        os.path.join(project_root, "tests")
    ]
    
    for directory in dirs_to_clean:
        if os.path.exists(directory):
            print(f"Cleaning directory: {directory}")
            clean_all_cpp_files(directory)
    
    print("Comment cleaning completed!")