import numpy as np

# 定义单个DH变换矩阵，加入偏移量offset
def dh_transform(theta, d, a, alpha):
    """ 计算 DH 变换矩阵 """
    T = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
        [0, 0, 0, 1]
    ])
    return T

# 输入6个关节的DH参数（theta, d, a, alpha）以及偏移量
dh_params = [
    (0, 0.1525,    0,     0,        0),        # 第1个关节没有偏移量
    (0, 0.0345,    0,     -np.pi/2, -np.pi/2), # 第2个关节偏移量 -pi/2
    (0, 0,         0.411, 0,        np.pi/2),  # 第3个关节偏移量 pi/2
    (0, 0.368,     0,     np.pi/2,  0),        # 第4个关节没有偏移量
    (0, 0,         0,     -np.pi/2, 0),        # 第5个关节没有偏移量
    (0, 0.121,     0,     np.pi/2,  0)         # 第6个关节没有偏移量
]

#-----------------更新角度--------------------------
joint_angles = [0, 0, 0, 0, 0, 0]

# 初始化总变换矩阵为单位矩阵
T_0_6 = np.eye(4)

# 逐个关节计算变换矩阵并乘以总变换矩阵
for i in range(6):
    theta, d, a, alpha, offset = dh_params[i]
    # 对应的theta是关节的角度与offset相加
    T_i = dh_transform(joint_angles[i] + offset + theta, d, a, alpha)
    T_0_6 = np.dot(T_0_6, T_i)

# 输出末端到基座的变换矩阵
print("末端到基座的变换矩阵1：")
print(T_0_6)
print("保留小数点：")
print(np.round(T_0_6,4))
np.set_printoptions(precision=4, suppress=True)
print(T_0_6)

#---------------------------------------------------------------------------------------
# 点 p 在末端坐标系下的表示 (3x1)，这里假设 p 在末端坐标系下为 [x, y, z]
p = np.array([0, 0, 0.01])  # 假设传感器在末端坐标系下的相对位置为[0, 0, 10]

# 构建平移矩阵 T_translation (4x4)，将末端坐标系原点平移到 p
T_translation = np.eye(4)
T_translation[0, 3] = p[0]
T_translation[1, 3] = p[1]
T_translation[2, 3] = p[2]

# 最终的变换矩阵：从以 p 为原点的坐标系到基座坐标系
T_sensor_to_base = np.dot(T_0_6, T_translation)

# 输出最终的变换矩阵
print("以 传感器 为原点的坐标系到基座坐标系的变换矩阵：")
print(T_sensor_to_base)

# 提取旋转矩阵 R (3x3)
R = T_sensor_to_base[:3, :3]

# 提取平移向量 r_base (3x1)
r_base = T_sensor_to_base[:3, 3]

# 打印结果
print("旋转矩阵 R：")
print(R)

print("\n平移向量 r_base：")
print(r_base)


F_sensor = np.array([1, 2, 3])
M_sensor = np.array([0.1, 0.12, 0.11])

# 将力从传感器坐标系转换到基座坐标系
F_base = np.dot(R, F_sensor)

# 计算由于平移引起的额外力矩
extra_M = np.cross(r_base, F_base)

# 将力矩从传感器坐标系转换到基座坐标系
M_base = np.dot(R, M_sensor) + extra_M

# 输出结果
print("在基座坐标系下的力和力矩表示：")
print("力：", F_base)
print("力矩：", M_base)