概述
xvsdk.py包含python封装的xvsdk相关接口及数据定义。

1.0 文件目录
平台	目录
Windows	安装目录\bin\python-wrapper 默认C:\Program Files\xvsdk\bin\python-wrapper
Linux/Ubuntu	/usr/share/python-wrapper
2.0 接口介绍
xvsdk.py包含python封装的xvsdk相关接口及数据定义。具体内容如下：

Vector3F结构体，用于接收float类型的三维数据:

class Vector3F(Structure):
fields = [('x', c_float),
('y', c_float),
('z', c_float)]

Quaternion结构体，用于接收转换后的四元数数据:

class Quaternion(Structure):
fields = [('q0', c_float),
('q1', c_float),
('q2', c_float),
('q3', c_float)]

Vector3D结构体，用于接收double类型的三维数据:

class Vector3D(Structure):
fields = [('x', c_double),
('y', c_double),
('z', c_double)]

Transform_Matrix结构体，用于接收转换矩阵包括rotation及translation数据:

class Transform_Matrix(Structure):
fields = [('rotation', c_double * 9),
('translation', c_double * 3)]

UnifiedCameraModel结构体，用于接收标定参数UnifiedCameraModel类型数据:

class UnifiedCameraModel(Structure):
fields = [('w', c_int),
('h', c_int),
('fx', c_double),
('fy', c_double),
('u0', c_double),
('v0', c_double),
('xi', c_double)]

PolynomialDistortionCameraModel结构体，用于接收标定参数PolynomialDistortionCameraModel类型数据:

class PolynomialDistortionCameraModel(Structure):
fields = [('w', c_int),
('h', c_int),
('fx', c_double),
('fy', c_double),
('u0', c_double),
('v0', c_double),
('distor', c_double * 5)]

TagData结构体，用于接收标定参数Apriltag数据，包含ID，位置信息，orientation及quaternion数据，edge及host时间戳，以及confidence数据:

class TagData(Structure):
fields = [('tagID', c_int),
('position', c_float * 3),
('orientation', c_float * 3),
('quaternion', c_float * 4),
('edgeTimestamp', c_longlong),
('hostTimestamp', c_double),
('confidence', c_double)]

init接口，用来初始化设备:

def init():
return dll.xvisio_device_init()

stop接口，用来注销设备:

def stop():
dll.xslam_uninit()

slam_start接口，用来开启slam:

def slam_start():
return dll.xvisio_start_slam()

slam_stop接口，用来停止slam:

def slam_stop():
return dll.xvisio_stop_slam()

stereo_start接口，用来开启FE相机:

def stereo_start():
return dll.xvisio_start_stereo()

imu_start接口，用来开启IMU相机:

def imu_start():
return dll.xvisio_start_imu()

rgb_start接口，用来开启RGB相机:

def rgb_start():
return dll.xvisio_start_rgb()

tof_start接口，用来开启TOF相机:

def tof_start():
return dll.xvisio_start_tof()

sgbm_start接口，用来开启SGBM相机:

def sgbm_start():
return dll.xvisio_start_sgbm()

xvisio_get_6dof接口，用来获取callback接口返回的slam数据，返回6dof信息，orientation及quaternion数据，edge及host时间戳，以及confidence数据 :

def xvisio_get_6dof():
dll.xvisio_get_6dof(byref(position), byref(orientation), byref(quaternion), byref(slam_edgeTimestamp), byref(slam_hostTimestamp), byref(slam_confidence))
return position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence

xvisio_get_6dof_prediction接口，用来获取getPose接口返回的slam数据，参数为预测值，单位为s，返回6dof信息，orientation及quaternion数据，edge及host时间戳，以及confidence数据 :

def xvisio_get_6dof_prediction(prediction):
dll.xvisio_get_6dof_prediction(byref(position), byref(orientation), byref(quaternion), byref(slam_edgeTimestamp), byref(slam_hostTimestamp), byref(slam_confidence), prediction)
return position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence

xslam_get_imu接口，用来获取IMU数据，返回加速度，角速度，edge时间戳以及host时间戳:

def xslam_get_imu():
dll.xvisio_get_imu(byref(accel), byref(gyro), byref(imu_edgeTimestamp), byref(imu_hostTimestamp))
return accel, gyro, imu_edgeTimestamp, imu_hostTimestamp

xvisio_get_stereo接口，用来获取Fisheye相机数据，返回图像宽度，高度，左右眼原始图像数据，图像数据大小，edge时间戳以及host时间戳:

def xvisio_get_stereo():
dll.xvisio_get_stereo_info(byref(fe_width), byref(fe_height), byref(stereo_edgeTimestamp), byref(stereo_hostTimestamp), byref(fe_dataSize))
fe_left_data = (c_ubyte * fe_dataSize.value)()
fe_right_data = (c_ubyte * fe_dataSize.value)()
dll.xvisio_get_stereo_image(byref(fe_left_data), byref(fe_right_data))
return fe_width, fe_height, stereo_edgeTimestamp, stereo_hostTimestamp, fe_left_data, fe_right_data, fe_dataSize

xvisio_get_rgb接口，用来获取RGB相机数据，返回图像宽度，高度，原始图像数据，图像数据大小，edge时间戳以及host时间戳:

def xvisio_get_rgb():
dll.xvisio_get_rgb_info(byref(rgb_width), byref(rgb_height),byref(rgb_edgeTimestamp), byref(rgb_hostTimestamp), byref(rgb_dataSize))
rgb_data = (c_ubyte * rgb_dataSize.value)()
dll.xvisio_get_rgb_image(byref(rgb_data))
return rgb_width, rgb_height, rgb_edgeTimestamp, rgb_hostTimestamp, rgb_data, rgb_dataSize

xvisio_get_tof接口，用来获取TOF相机数据，返回图像宽度，高度，原始图像数据，图像数据大小，edge时间戳以及host时间戳:

def xvisio_get_tof():
dll.xvisio_get_tof_info(byref(tof_width), byref(tof_height),byref(tof_edgeTimestamp), byref(tof_hostTimestamp), byref(tof_dataSize))
tof_data = (c_ubyte * tof_dataSize.value)()
dll.xvisio_get_tof_image(byref(tof_data))
return tof_width, tof_height, tof_edgeTimestamp, tof_hostTimestamp, tof_data, tof_dataSize

xvisio_get_sgbm接口，用来获取SGBM相机数据，返回图像宽度，高度，原始图像数据，图像数据大小，edge时间戳以及host时间戳:

def xvisio_get_sgbm():
dll.xvisio_get_sgbm_info(byref(sgbm_width), byref(sgbm_height),byref(sgbm_edgeTimestamp), byref(sgbm_hostTimestamp), byref(sgbm_dataSize))
sgbm_data = (c_ubyte * sgbm_dataSize.value)()
dll.xvisio_get_sgbm_image(byref(sgbm_data))
return sgbm_width, sgbm_height, sgbm_edgeTimestamp, sgbm_hostTimestamp, sgbm_data, sgbm_dataSize

xvisio_get_sn接口，用来获取设备SN号，返回SN字符串信息:

def xvisio_get_sn():
sn = c_char_p()
dll.xvisio_get_sn(byref(sn))
return sn

xvisio_get_fisheye_intrinsics接口，用来获取Fisheye相机标定参数，返回transform标定信息，ucm标定信息及标定数据数组大小:

def xvisio_get_fisheye_intrinsics():
fe_trans_size = c_int()
fe_ucm_size = c_int()
dll.xvisio_get_fe_camera_intrinsics_param(byref(fe_trans_size), byref(fe_ucm_size))
fe_transform = (Transform_Matrix * fe_trans_size.value)()
fe_ucm = (UnifiedCameraModel * fe_ucm_size.value)()
dll.xvisio_get_fe_camera_intrinsics(fe_transform, fe_ucm)
return fe_trans_size, fe_ucm_size, fe_transform, fe_ucm

xvisio_get_rgb_intrinsics接口，用来获取RGB相机标定参数，返回transform标定信息，ucm标定信息及标定数据数组大小:

def xvisio_get_rgb_intrinsics():
rgb_trans_size = c_int()
rgb_pdcm_size = c_int()
dll.xvisio_get_rgb_camera_intrinsics_param(byref(rgb_trans_size), byref(rgb_pdcm_size))
rgb_transform = (Transform_Matrix * rgb_trans_size.value)()
rgb_pdcm = (PolynomialDistortionCameraModel * rgb_pdcm_size.value)()
dll.xvisio_get_rgb_camera_intrinsics(rgb_transform, rgb_pdcm)
return rgb_trans_size, rgb_pdcm_size, rgb_transform, rgb_pdcm

xvisio_get_tof_intrinsics接口，用来获取TOF相机标定参数，返回transform标定信息，ucm标定信息及标定数据数组大小:

def xvisio_get_tof_intrinsics():
tof_trans_size = c_int()
tof_pdcm_size = c_int()
dll.xvisio_get_tof_camera_intrinsics_param(byref(tof_trans_size), byref(tof_pdcm_size))
tof_transform = (Transform_Matrix * tof_trans_size.value)()
tof_pdcm = (PolynomialDistortionCameraModel * tof_pdcm_size.value)()
dll.xvisio_get_tof_camera_intrinsics(tof_transform, tof_pdcm)
return tof_trans_size, tof_pdcm_size, tof_transform, tof_pdcm

xvisio_set_rgb_camera_resolution接口，用来设置RGB相机分辨率。0-1920x1080， 1-1280x720， 2-640x480， 3-320x240， 4-2560x1920， 5-3840x2160:

def xvisio_set_rgb_camera_resolution(resolution):
dll.xvisio_set_rgb_camera_resolution(resolution)

xvisio_get_fe_april_tags接口，用来获取April-tag数据信息。返回tag结构体，包含ID，位置信息，orientation及quaternion数据，edge及host时间戳，以及confidence数据:

def xvisio_get_fe_april_tags():
tagDetectorID = c_char_p()
tagSize = c_int()
dll.xvisio_get_fe_tag_size(byref(tagDetectorID), byref(tagSize))
tags = (TagData * tagSize.value)()
dll.xvisio_get_fe_tag_detection(tags)
return tags