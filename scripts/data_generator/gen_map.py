#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from PIL import Image
import os
import sys

def generate_terrain_map():
    """
    生成模拟地形图用于双足机器人路径规划测试
    地图包含七个不同高度的区域，模拟真实地形的复杂性
    """
    # 定义高度值 (单位: cm)
    h0 = 30
    h1 = h0 + h0      # 60
    h2 = h1 + h0      # 90
    h3 = h2 + h0      # 120
    h4 = h3 - h0      # 90
    h5 = h4 - h0      # 60
    h6 = h5 - h0      # 30
    
    # 创建1000x3000的矩阵，代表地形图
    I = np.ones((1000, 3000))
    
    # 设置不同区域的高度，模拟复杂地形
    I[:, 0:1000] = h0           # 第一区域: 高度30
    I[:, 1000:1060] = h1        # 第二区域: 高度60
    I[:, 1060:1120] = h2        # 第三区域: 高度90
    I[:, 1120:1300] = h3        # 第四区域: 高度120
    I[:, 1300:1500] = h4        # 第五区域: 高度90
    I[:, 1500:2000] = h5        # 第六区域: 高度60
    I[:, 2000:3000] = h6        # 第七区域: 高度30
    
    # 添加高斯白噪声以模拟真实地形的不规则性
    noise = np.random.randn(1000, 3000)
    I = I + noise
    
    # 转换为uint8类型并保存为JPEG图像
    # 确保值在0-255范围内
    I = np.clip(I, 0, 255)
    I = I.astype(np.uint8)
    
    # 获取脚本所在目录，然后构建输出路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, '..', '..', 'data', 'input')
    output_dir = os.path.normpath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    
    # 保存为JPEG图像
    output_path = os.path.join(output_dir, 'map.jpg')
    img = Image.fromarray(I)
    img.save(output_path, 'JPEG')
    
    print(f"地形图已生成并保存为 {output_path}")
    print(f"图像尺寸: {img.width} x {img.height}")
    print(f"高度范围: {I.min()} - {I.max()}")

if __name__ == "__main__":
    try:
        generate_terrain_map()
    except Exception as e:
        print(f"生成地图时发生错误: {e}")
        sys.exit(1)