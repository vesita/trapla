# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import math

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'FangSong', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

class SqDot:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
    
    def distance(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle(self, other):
        return math.atan2(other.y - self.y, other.x - self.x)

class Foot:
    def __init__(self, position=SqDot(), rz=0.0, length=1.0, width=0.5):
        self.position = position
        self.rz = rz
        self.length = length
        self.width = width
    
    def corner(self):
        """计算足部的四个角点"""
        half_length = self.length / 2
        half_width = self.width / 2
        
        # 足部的四个角点（相对于足部中心）
        corners_local = [
            SqDot(-half_length, -half_width),  # 左下
            SqDot(half_length, -half_width),   # 右下
            SqDot(half_length, half_width),    # 右上
            SqDot(-half_length, half_width)    # 左上
        ]
        
        # 旋转并平移到世界坐标系
        cos_rz = math.cos(self.rz)
        sin_rz = math.sin(self.rz)
        rotated_corners = []
        
        for corner in corners_local:
            # 旋转变换
            x_rot = corner.x * cos_rz - corner.y * sin_rz
            y_rot = corner.x * sin_rz + corner.y * cos_rz
            
            # 平移变换
            x_world = x_rot + self.position.x
            y_world = y_rot + self.position.y
            
            rotated_corners.append(SqDot(x_world, y_world))
        
        return rotated_corners

def calculate_near_side_line(support_foot, swing_foot):
    """计算支撑脚的近侧外形线"""
    # 根据最新代码，近侧外形线就是通过支撑脚中心点、方向为支撑脚朝向的线
    near_side_point = support_foot.position
    line_direction = support_foot.rz
    
    return near_side_point, line_direction

def distance_point_to_line(point, line_point, line_direction):
    """计算点到直线的距离"""
    # 直线方向向量
    line_vec_x = math.cos(line_direction)
    line_vec_y = math.sin(line_direction)
    
    # 从直线上一点到目标点的向量
    to_point_x = point.x - line_point.x
    to_point_y = point.y - line_point.y
    
    # 使用向量叉积计算点到直线的距离
    distance = abs(to_point_x * line_vec_y - to_point_y * line_vec_x)
    return distance

def satisfy_spacing(swing_foot, support_foot, new_pos, min_foot_separation, max_foot_separation):
    """检查新位置是否满足足部间距限制条件"""
    # 创建新足部位置
    new_foot = Foot(new_pos, swing_foot.rz, swing_foot.length, swing_foot.width)
    
    # 获取新足部的角点
    points = new_foot.corner()
    
    # 计算支撑脚的近侧外形线
    near_side_point, line_direction = calculate_near_side_line(support_foot, swing_foot)
    
    # 计算新足部角点到近侧外形线的最小距离
    spacing = float('inf')
    for point in points:
        dist = distance_point_to_line(point, near_side_point, line_direction)
        spacing = min(spacing, dist)
    
    # 模拟近侧外形线
    half_width = support_foot.width / 2.0
    spacing = min(0.0, spacing - half_width)
    
    # 使用epsilon处理浮点数比较问题
    epsilon = 1e-3
    min_allowed = min_foot_separation - epsilon
    max_allowed = max_foot_separation + epsilon
    
    # 检查间距是否满足约束
    satisfied = spacing >= min_allowed and spacing <= max_allowed
    
    return satisfied, spacing

def visualize_foot_positions(swing_foot, support_foot, new_pos, min_separation, max_separation):
    """可视化足部位置和间距约束"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    
    # 绘制支撑脚
    support_corners = support_foot.corner()
    support_polygon = Polygon([(p.x, p.y) for p in support_corners], 
                             facecolor='lightblue', edgecolor='blue', linewidth=2, alpha=0.7, label='支撑脚')
    ax.add_patch(support_polygon)
    
    # 绘制摆动脚当前位置
    swing_corners = swing_foot.corner()
    swing_polygon = Polygon([(p.x, p.y) for p in swing_corners], 
                           facecolor='lightcoral', edgecolor='red', linewidth=2, alpha=0.7, label='摆动脚当前位置')
    ax.add_patch(swing_polygon)
    
    # 绘制摆动脚新位置
    new_foot = Foot(new_pos, swing_foot.rz, swing_foot.length, swing_foot.width)
    new_corners = new_foot.corner()
    new_polygon = Polygon([(p.x, p.y) for p in new_corners], 
                         facecolor='lightgreen', edgecolor='green', linewidth=2, alpha=0.7, label='摆动脚新位置')
    ax.add_patch(new_polygon)
    
    # 计算并绘制近侧外形线
    near_side_point, line_direction = calculate_near_side_line(support_foot, swing_foot)
    
    # 绘制一条足够长的线段表示近侧外形线
    line_length = 10
    line_start = SqDot(
        near_side_point.x - line_length * math.cos(line_direction),
        near_side_point.y - line_length * math.sin(line_direction)
    )
    line_end = SqDot(
        near_side_point.x + line_length * math.cos(line_direction),
        near_side_point.y + line_length * math.sin(line_direction)
    )
    
    ax.plot([line_start.x, line_end.x], [line_start.y, line_end.y], 
            'b--', linewidth=1, label='近侧外形线')
    
    # 计算新足部角点到近侧外形线的距离
    satisfied, spacing = satisfy_spacing(swing_foot, support_foot, new_pos, min_separation, max_separation)
    
    # 绘制新足部角点到近侧外形线的垂直线
    corner_labels = ['左下角', '右下角', '右上角', '左上角']
    for i, point in enumerate(new_corners):
        # 计算垂足
        line_vec_x = math.cos(line_direction)
        line_vec_y = math.sin(line_direction)
        to_point_x = point.x - near_side_point.x
        to_point_y = point.y - near_side_point.y
        
        # 投影长度
        proj_length = to_point_x * line_vec_x + to_point_y * line_vec_y
        foot_x = near_side_point.x + proj_length * line_vec_x
        foot_y = near_side_point.y + proj_length * line_vec_y
        
        # 绘制从角点到垂足的线
        ax.plot([point.x, foot_x], [point.y, foot_y], 'g:', alpha=0.7)
        ax.plot(foot_x, foot_y, 'go', markersize=4)
        # 标注角点和坐标值
        ax.text(point.x, point.y, f'{corner_labels[i]}\n({point.x:.2f}, {point.y:.2f})', 
                fontsize=9, ha='right', va='bottom')
    
    # 添加标注
    ax.plot(support_foot.position.x, support_foot.position.y, 'bo', markersize=6, label='支撑脚中心')
    ax.plot(swing_foot.position.x, swing_foot.position.y, 'ro', markersize=6, label='摆动脚中心')
    ax.plot(new_pos.x, new_pos.y, 'go', markersize=6, label='新位置中心')
    
    # 标注中心点坐标
    ax.text(support_foot.position.x, support_foot.position.y, 
            f'支撑脚中心\n({support_foot.position.x:.2f}, {support_foot.position.y:.2f})',
            fontsize=9, ha='left', va='top')
    ax.text(swing_foot.position.x, swing_foot.position.y, 
            f'摆动脚中心\n({swing_foot.position.x:.2f}, {swing_foot.position.y:.2f})',
            fontsize=9, ha='left', va='top')
    ax.text(new_pos.x, new_pos.y, 
            f'新位置中心\n({new_pos.x:.2f}, {new_pos.y:.2f})',
            fontsize=9, ha='left', va='top')
    
    # 添加信息文本
    ax.text(0.02, 0.98, f'间距: {spacing:.3f}\n最小间距: {min_separation}\n最大间距: {max_separation}\n满足约束: {satisfied}', 
            transform=ax.transAxes, fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title('机器人足部间距约束可视化')
    
    plt.tight_layout()
    plt.show()
    
    return satisfied, spacing

def main():
    # 创建测试用的足部
    support_foot = Foot(SqDot(0, 0), 0, 1.0, 0.5)  # 支撑脚在原点，朝向0度
    swing_foot = Foot(SqDot(-1, 0), 0, 1.0, 0.5)   # 摆动脚在(-1, 0)，朝向0度
    
    # 测试不同的新位置
    test_positions = [
        SqDot(1.5, 0.3),   # 应该满足约束
        SqDot(1.5, 1.0),   # 可能超出最大间距
        SqDot(1.5, -1.0),  # 可能小于最小间距
        SqDot(0.5, 0.0),   # 可能在支撑脚内部
    ]
    
    min_separation = 0.8
    max_separation = 1.2
    
    print("测试足部间距约束：")
    print(f"最小间距: {min_separation}, 最大间距: {max_separation}")
    print("=" * 50)
    
    for i, new_pos in enumerate(test_positions):
        satisfied, spacing = satisfy_spacing(swing_foot, support_foot, new_pos, min_separation, max_separation)
        print(f"测试 {i+1}: 位置({new_pos.x:.1f}, {new_pos.y:.1f}) -> 间距: {spacing:.3f}, 满足约束: {satisfied}")
        
        # 可视化第一个测试案例
        if i == 0:
            print("生成可视化...")
            visualize_foot_positions(swing_foot, support_foot, new_pos, min_separation, max_separation)

if __name__ == "__main__":
    main()