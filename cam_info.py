import xvsdk
import numpy as np

xvsdk.init()
# xvsdk.slam_start()
xvsdk.stereo_start()
xvsdk.rgb_start()

# 获取左右鱼眼相机内参
fish_trans_size, fish_pdcm_size, fish_transform, fish_pdcm = xvsdk.xv_get_fisheye_intrinsics()
# tof_trans_size, tof_pdcm_size, tof_transform, tof_pdcm = xvsdk.xv_get_tof_intrinsics()

# 获取RGB相机内参
rgb_trans_size, rgb_pdcm_size, rgb_transform, rgb_pdcm = xvsdk.xv_get_rgb_intrinsics()

# 打印右鱼眼相机变换矩阵  
print("左右鱼眼相机变换矩阵:")
for i in range(fish_trans_size.value):
    R = np.array(fish_transform[i].rotation).reshape(3,3)
    T = np.array(fish_transform[i].translation)
    print(f"R{i}:\n{R}")
    print(f"T{i}:\n{T}\n")

# print("左右tof相机变换矩阵:")
# for i in range(tof_trans_size.value):
#     R = np.array(tof_transform[i].rotation).reshape(3,3)
#     T = np.array(tof_transform[i].translation)
#     print(f"R{i}:\n{R}")
#     print(f"T{i}:\n{T}\n")

# 打印RGB相机变换矩阵
print("RGB相机变换矩阵:")
for i in range(rgb_trans_size.value):
    R = np.array(rgb_transform[i].rotation).reshape(3,3)
    T = np.array(rgb_transform[i].translation)
    print(f"R{i}:\n{R}")
    print(f"T{i}:\n{T}\n")
