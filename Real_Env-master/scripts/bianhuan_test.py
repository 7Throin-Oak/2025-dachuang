import numpy as np

# 假设的末端到基座的变换矩阵 T_end_to_base (4x4)
T_end_to_base = np.array([
    [0.35355339, -0.612372,  0.70710678,  0.612372],
    [0.35355339,  0.612372,  0.70710678,  1.03923048],
    [0.8660254,   0.5,        0.0,         0.93540374],
    [0.0,         0.0,        0.0,         1.0]
])

# 点 p 在末端坐标系下的表示 (3x1)，这里假设 p 在末端坐标系下为 [x, y, z]
p = np.array([0.5, 0.5, 1.0])  # 假设 p 为 [0.5, 0.5, 1.0]

# 构建平移矩阵 T_translation (4x4)，将末端坐标系原点平移到 p
T_translation = np.eye(4)
print(T_translation)
T_translation[0, 3] = p[0]
T_translation[1, 3] = p[1]
T_translation[2, 3] = p[2]
print(T_translation)
# 最终的变换矩阵：从以 p 为原点的坐标系到基座坐标系
T_p_to_base = np.dot(T_end_to_base, T_translation)

# 输出最终的变换矩阵
print("以 p 为原点的坐标系到基座坐标系的变换矩阵：")
print(T_p_to_base)