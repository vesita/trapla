import matplotlib.pyplot as plt
import csv
import os

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'FangSong', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

def read_csv_points(filename):
    """读取CSV文件中的点坐标"""
    points = []
    try:
        if os.path.exists(filename):
            with open(filename, 'r', encoding='utf-8') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    points.append((float(row['x']), float(row['y'])))
    except Exception as e:
        print(f"读取文件时出错: {e}")
        return []
    return points

def plot_path():
    """绘制机器人行走路径，区分连续足迹"""
    # 读取路径点数据
    path_points = read_csv_points('data/csv/path_result.csv')
    
    if not path_points:
        print("未找到路径数据文件或文件为空")
        return
    
    # 分离连续足迹点（每一步的落足点）
    even_points = path_points[::2]   # 偶数索引点
    odd_points = path_points[1::2]   # 奇数索引点
    
    # 提取坐标
    even_x = [point[0] for point in even_points]
    even_y = [point[1] for point in even_points]
    
    odd_x = [point[0] for point in odd_points]
    odd_y = [point[1] for point in odd_points]
    
    # 创建图形
    plt.figure(figsize=(15, 10))
    
    # 绘制连续足迹
    plt.plot(even_x, even_y, 'b-o', linewidth=2, markersize=6, label='足迹序列1', alpha=0.7)
    plt.plot(odd_x, odd_y, 'r-s', linewidth=2, markersize=6, label='足迹序列2', alpha=0.7)
    
    # 标记起点
    if path_points:
        plt.scatter(path_points[0][0], path_points[0][1], color='green', s=150, label='起始点', zorder=5)
    
    # 标记终点
    if path_points:
        plt.scatter(path_points[-1][0], path_points[-1][1], color='black', s=150, label='终点', zorder=5)
    
    # 连接相邻足迹点形成步态
    for i in range(len(path_points) - 1):
        plt.plot([path_points[i][0], path_points[i+1][0]], 
                 [path_points[i][1], path_points[i+1][1]], 
                 'g-', alpha=0.3, linewidth=1)
    
    # 添加网格
    plt.grid(True, alpha=0.3)
    
    # 设置坐标轴标签和标题
    plt.xlabel('X 坐标', fontsize=12)
    plt.ylabel('Y 坐标', fontsize=12)
    plt.title('双足机器人行走路径可视化', fontsize=14)
    
    # 设置图例
    plt.legend(fontsize=10)
    
    # 保证坐标轴比例一致
    plt.axis('equal')
    
    # 自动调整布局
    plt.tight_layout()
    
    # 创建log目录（如果不存在）
    os.makedirs('log', exist_ok=True)
    
    # 保存图像
    plt.savefig('log/path_visualization.png', dpi=300, bbox_inches='tight')
    
    # 显示图形
    plt.show()
    
    # 打印统计信息
    print(f"路径点总数: {len(path_points)}")
    print(f"足迹序列1点数: {len(even_points)}")
    print(f"足迹序列2点数: {len(odd_points)}")
    
    if path_points:
        print(f"起始点坐标: ({path_points[0][0]:.2f}, {path_points[0][1]:.2f})")
        print(f"终点坐标: ({path_points[-1][0]:.2f}, {path_points[-1][1]:.2f})")
    
    print("路径可视化图像已保存至: log/path_visualization.png")

if __name__ == "__main__":
    plot_path()