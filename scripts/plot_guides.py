import matplotlib.pyplot as plt
import csv
import os

def read_csv_points(filename):
    """读取CSV文件中的点坐标"""
    points = []
    if os.path.exists(filename):
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                points.append((float(row['x']), float(row['y'])))
    return points

def plot_guides_and_directions():
    """绘制引导点和指向点"""
    # 读取引导点和指向点数据
    guides = read_csv_points('log/guides.csv')
    directions = read_csv_points('log/direction.csv')
    
    # 创建图形
    plt.figure(figsize=(12, 10))
    
    # 绘制引导点
    if guides:
        guide_x = [point[0] for point in guides]
        guide_y = [point[1] for point in guides]
        plt.plot(guide_x, guide_y, 'b-o', label='Guide Points', markersize=4)
        plt.scatter(guide_x[0], guide_y[0], color='green', s=100, label='Start', zorder=5)
        plt.scatter(guide_x[-1], guide_y[-1], color='red', s=100, label='End', zorder=5)
    
    # 绘制指向点
    if directions:
        direction_x = [point[0] for point in directions]
        direction_y = [point[1] for point in directions]
        plt.scatter(direction_x, direction_y, color='orange', s=80, label='Direction Point', zorder=5)
    
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Guide Points and Direction Points Visualization')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # 保存并显示图形
    plt.savefig('log/visualization.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print(f"Loaded {len(guides)} guide points")
    print(f"Loaded {len(directions)} direction points")
    
    if guides:
        print(f"Start point: ({guide_x[0]}, {guide_y[0]})")
        print(f"End point: ({guide_x[-1]}, {guide_y[-1]})")
    
    if directions:
        print(f"Direction point: ({direction_x[0]}, {direction_y[0]})")

if __name__ == "__main__":
    plot_guides_and_directions()