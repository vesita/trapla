import matplotlib.pyplot as plt
import csv
import os

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'FangSong', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

def read_csv_points(filename):
    """读取CSV文件中的点坐标"""
    points = []
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                points.append((float(row['x']), float(row['y'])))
    return points

def plot_path():
    """绘制机器人行走路径，区分左右脚足迹"""
    # 读取路径点数据
    path_points = read_csv_points('data/csv/path_result.csv')
    
    if not path_points:
        print("未找到路径数据文件或文件为空")
        return
    
    # 分离左右脚足迹
    left_foot_points = path_points[::2]   # 偶数索引为左脚
    right_foot_points = path_points[1::2] # 奇数索引为右脚
    
    # 提取坐标
    left_x = [point[0] for point in left_foot_points]
    left_y = [point[1] for point in left_foot_points]
    
    right_x = [point[0] for point in right_foot_points]
    right_y = [point[1] for point in right_foot_points]
    
    # 创建图形
    plt.figure(figsize=(15, 8))
    
    # 绘制左右脚足迹
    plt.plot(left_x, left_y, 'b-o', linewidth=2, markersize=6, label='左脚轨迹')
    plt.plot(right_x, right_y, 'r-s', linewidth=2, markersize=6, label='右脚轨迹')
    
    # 标记起点（左脚起点）
    if left_foot_points:
        plt.scatter(left_x[0], left_y[0], color='green', s=150, label='起始点', zorder=5)
    
    # 标记终点（根据最后一步判断哪只脚是最后一步）
    if path_points:
        plt.scatter(path_points[-1][0], path_points[-1][1], color='black', s=150, label='终点', zorder=5)
    
    # 连接左右脚足迹形成步态
    for i in range(min(len(left_foot_points), len(right_foot_points))):
        plt.plot([left_x[i], right_x[i]], [left_y[i], right_y[i]], 'g-', alpha=0.3, linewidth=1)
    
    # 添加网格
    plt.grid(True, alpha=0.3)
    
    # 设置坐标轴标签和标题
    plt.xlabel('X 坐标')
    plt.ylabel('Y 坐标')
    plt.title('双足机器人行走路径可视化')
    
    # 设置图例
    plt.legend()
    
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
    print(f"左脚足迹数: {len(left_foot_points)}")
    print(f"右脚足迹数: {len(right_foot_points)}")
    
    if path_points:
        print(f"起始点坐标: ({path_points[0][0]:.2f}, {path_points[0][1]:.2f})")
        print(f"终点坐标: ({path_points[-1][0]:.2f}, {path_points[-1][1]:.2f})")
    
    print("路径可视化图像已保存至: log/path_visualization.png")

if __name__ == "__main__":
    plot_path()