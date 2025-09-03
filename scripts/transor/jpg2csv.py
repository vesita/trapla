import numpy as np
from PIL import Image
import csv
import os

def jpg_to_csv(input_path='../../data/input/map.jpg', output_path='../../data/output/map.csv'):
    """
    将JPEG格式的地图转换为CSV格式
    
    参数:
    input_path: 输入的JPEG地图文件路径
    output_path: 输出的CSV文件路径
    """
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # 读取JPEG图像
    img = Image.open(input_path)
    
    # 转换为灰度图像（如果还不是）
    img = img.convert('L')
    
    # 转换为numpy数组
    data = np.array(img)
    
    # 获取图像尺寸
    height, width = data.shape
    print(f"图像尺寸: {width} x {height}")
    
    # 将数据写入CSV文件
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # 写入数据
        for row in data:
            writer.writerow(row)
    
    print(f"地图数据已转换并保存为 {output_path}")
    print(f"数据形状: {data.shape}")
    print(f"最小高度值: {data.min()}")
    print(f"最大高度值: {data.max()}")

def csv_to_jpg(input_path='../../data/output/map.csv', output_path='../../data/output/map_from_csv.jpg'):
    """
    将CSV格式的地图转换回JPEG格式（用于验证）
    
    参数:
    input_path: 输入的CSV文件路径
    output_path: 输出的JPEG文件路径
    """
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # 从CSV文件读取数据
    data = []
    with open(input_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data.append([int(float(val)) for val in row])
    
    # 转换为numpy数组
    data = np.array(data, dtype=np.uint8)
    
    # 转换为图像并保存
    img = Image.fromarray(data, mode='L')
    img.save(output_path, 'JPEG')
    
    print(f"CSV数据已转换并保存为 {output_path}")

if __name__ == "__main__":
    # 将JPEG转换为CSV
    jpg_to_csv()
    
    # 验证：将CSV转换回JPEG
    csv_to_jpg()