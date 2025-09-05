#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from PIL import Image
import csv
import os
import sys

def jpg_to_csv(input_path=None, output_path=None):
    """
    将JPEG格式的地图转换为CSV格式，以便进一步处理或分析。
    
    功能:
    - 读取JPEG图像文件并将其转换为灰度图像。
    - 将图像数据转换为NumPy数组，并将其写入CSV文件。
    
    参数:
    input_path: 输入的JPEG地图文件路径（可选）
    output_path: 输出的CSV文件路径（可选）
    
    如果未提供路径，将使用默认路径：
    - 输入路径: ../data/input/map.jpg
    - 输出路径: ../data/csv/map.csv
    """
    # 获取脚本所在目录，然后构建默认路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    if input_path is None:
        input_path = os.path.join(script_dir, '..', '..', 'data', 'input', 'map.jpg')
        input_path = os.path.normpath(input_path)
        
    if output_path is None:
        output_path = os.path.join(script_dir, '..', '..', 'data', 'csv', 'map.csv')
        output_path = os.path.normpath(output_path)
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # 检查输入文件是否存在
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"输入文件不存在: {input_path}")
    
    print(f"正在读取输入文件: {input_path}")
    # 读取JPEG图像
    img = Image.open(input_path)
    
    # 转换为灰度图像（如果还不是）
    print("正在将图像转换为灰度图像...")
    img = img.convert('L')
    
    # 转换为numpy数组
    data = np.array(img)
    
    # 获取图像尺寸
    height, width = data.shape
    print(f"图像尺寸: {width} x {height}")
    print(f"数据类型: {data.dtype}")
    
    # 将数据写入CSV文件
    print(f"正在将数据写入CSV文件: {output_path}")
    with open(output_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        
        # 写入数据
        for row in data:
            writer.writerow(row)
    
    print(f"地图数据已转换并保存为 {output_path}")
    print(f"数据形状: {data.shape}")
    print(f"最小高度值: {data.min()}")
    print(f"最大高度值: {data.max()}")
    print("JPEG到CSV转换完成。")

def csv_to_jpg(input_path=None, output_path=None):
    """
    将CSV格式的地图转换回JPEG格式，用于验证转换的准确性。
    
    功能:
    - 读取CSV文件中的图像数据。
    - 将数据转换为NumPy数组并生成灰度图像。
    - 保存生成的图像为JPEG格式。
    
    参数:
    input_path: 输入的CSV文件路径（可选）
    output_path: 输出的JPEG文件路径（可选）
    
    如果未提供路径，将使用默认路径：
    - 输入路径: ../data/csv/map.csv
    - 输出路径: ../data/output/map_from_csv.jpg
    """
    # 获取脚本所在目录，然后构建默认路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    if input_path is None:
        input_path = os.path.join(script_dir, '..', '..', 'data', 'csv', 'map.csv')
        input_path = os.path.normpath(input_path)
        
    if output_path is None:
        output_path = os.path.join(script_dir, '..', '..', 'data', 'output', 'map_from_csv.jpg')
        output_path = os.path.normpath(output_path)
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # 检查输入文件是否存在
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"输入文件不存在: {input_path}")
    
    print(f"正在读取CSV文件: {input_path}")
    # 从CSV文件读取数据
    data = []
    with open(input_path, 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data.append([int(float(val)) for val in row])
    
    print("正在将数据转换为NumPy数组...")
    # 转换为numpy数组
    data = np.array(data, dtype=np.uint8)
    
    print("正在生成灰度图像...")
    # 转换为图像并保存
    img = Image.fromarray(data, mode='L')
    img.save(output_path, 'JPEG')
    
    print(f"CSV数据已转换并保存为 {output_path}")
    print("CSV到JPEG转换完成。")

if __name__ == "__main__":
    print("开始执行地图数据转换流程...")
    try:
        # 将JPEG转换为CSV
        print("执行JPEG到CSV转换...")
        jpg_to_csv()
        
        # 验证：将CSV转换回JPEG
        print("执行CSV到JPEG转换以进行验证...")
        csv_to_jpg()
        
        print("地图数据转换流程已完成。")
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        sys.exit(1)