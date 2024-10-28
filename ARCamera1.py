from ctypes import *
import xvsdk
import numpy as np
import cv2
import pygame
from pygame.math import Vector3
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.arrays import vbo
from OpenGL.GL import shaders
import os
from plyfile import PlyData

# 设置使用独立显卡
os.environ["CUDA_VISIBLE_DEVICES"] = "0"

class ColorCameraResolution():
        RGB_1920x1080=0
        RGB_1280x720=1
        RGB_640x480=2
        RGB_320x240=3
        RGB_2560x1920=4
        RGB_3840x2160=5
fe_auto_Exposure=c_bool()
fe_gain=c_int()
fe_exposureTimeMs=c_int()
fe_auto_Exposure.value=True

class ARCamera:
    def __init__(self):
        self.init_xvsdk()
        self.vertices, self.faces, self.normals = self.load_ply('initial_mesh.ply')
        self.camera_intrinsic, self.camera_extrinsic = self.get_rgb_camera_intrinsics()
        self.display = self.init_pygame_opengl()
        self.init_vbo()
        self.shader_program = self.create_shader_program()
        self.texture = glGenTextures(1)
        self.rgb_hostTimestamp = c_double()

    def init_xvsdk(self):
        """初始化xvsdk"""
        xvsdk.init()
        xvsdk.slam_start()
        xvsdk.stereo_start()
        xvsdk.imu_start()
        xvsdk.rgb_start()
        xvsdk.xv_set_rgb_camera_resolution(ColorCameraResolution.RGB_640x480)
        xvsdk.xv_set_fe_autoExposure(fe_auto_Exposure)

    def init_pygame_opengl(self):
        """初始化Pygame和OpenGL"""
        pygame.init()
        display = (self.camera_intrinsic["width"], self.camera_intrinsic["height"])
        pygame.display.set_mode(display, pygame.DOUBLEBUF | pygame.OPENGL)
        glEnable(GL_DEPTH_TEST)
        return display

    def load_ply(self, filename):
        """加载ply格式的mesh"""
        plydata = PlyData.read(filename)
        vertices = np.array([list(vertex) for vertex in plydata['vertex'].data], dtype=np.float32)
        faces = np.array([list(face[0]) for face in plydata['face'].data], dtype=np.int32)
        
        # 计算法向量
        normals = np.zeros_like(vertices)
        for face in faces:
            v1, v2, v3 = vertices[face]
            normal = np.cross(v2 - v1, v3 - v1)
            normals[face] += normal
        normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)
        vertices = vertices
        # 模型移动到0，0，0
        vertices = vertices - np.mean(vertices, axis=0)
        return vertices, faces, normals

    def init_vbo(self):
        """初始化顶点缓冲对象"""
        self.vertices_vbo = vbo.VBO(self.vertices)
        self.normals_vbo = vbo.VBO(self.normals)
        self.faces_vbo = vbo.VBO(self.faces, target=GL_ELEMENT_ARRAY_BUFFER)

    def create_shader_program(self):
        """创建着色器程序"""
        vertex_shader = """
        #version 330
        in vec3 position;
        in vec3 normal;
        uniform mat4 modelViewMatrix;
        uniform mat4 projectionMatrix;
        out vec3 fragNormal;
        void main() {
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            fragNormal = mat3(modelViewMatrix) * normal;
        }
        """

        fragment_shader = """
        #version 330
        in vec3 fragNormal;
        out vec4 fragColor;
        void main() {
            vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
            float diffuse = max(dot(normalize(fragNormal), lightDir), 0.0);
            vec3 color = vec3(0.8, 0.8, 0.8) * (diffuse + 0.2);
            fragColor = vec4(color, 1.0);
        }
        """

        shader_program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER)
        )
        return shader_program

    def draw_mesh(self, transform_matrix):
        """绘制mesh"""
        glUseProgram(self.shader_program)
        # # 获取当前的模型视图矩阵&投影矩阵
        modelViewMatrix = glGetFloatv(GL_MODELVIEW_MATRIX).T
        modelViewMatrix = np.dot(modelViewMatrix, transform_matrix).T
        projectionMatrix = glGetFloatv(GL_PROJECTION_MATRIX)
        # # 将模型视图矩阵和投影矩阵传递给着色器
        glUniformMatrix4fv(glGetUniformLocation(self.shader_program, "modelViewMatrix"), 1, GL_FALSE, modelViewMatrix)
        glUniformMatrix4fv(glGetUniformLocation(self.shader_program, "projectionMatrix"), 1, GL_FALSE, projectionMatrix)
        
        # 绑定顶点缓冲对象并启用顶点属性数组
        self.vertices_vbo.bind()
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, None)
        # 绑定法线缓冲对象并启用法线属性数组
        self.normals_vbo.bind()
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, None)
        # 绑定面索引缓冲对象并绘制三角形
        self.faces_vbo.bind()
        glDrawElements(GL_TRIANGLES, len(self.faces) * 3, GL_UNSIGNED_INT, None)
        # 清理：禁用顶点属性数组并解绑着色器程序
        glDisableVertexAttribArray(0)
        glDisableVertexAttribArray(1)
        glUseProgram(0)

    def get_rgb_camera_intrinsics(self):
        """获取RGB相机内参"""
        rgb_trans_size, rgb_pdcm_size, rgb_transform, rgb_pdcm = xvsdk.xv_get_rgb_intrinsics()
        rgb_pdcm1080 = rgb_pdcm[0]  # 假设使用1080p分辨率
        camera_intrinsic = {
            "fx": rgb_pdcm1080.fx,
            "fy": rgb_pdcm1080.fy,
            "u0": rgb_pdcm1080.u0,
            "v0": rgb_pdcm1080.v0,
            "width": rgb_pdcm1080.w,
            "height": rgb_pdcm1080.h,
            "distortion": np.array([rgb_pdcm1080.distor[0], rgb_pdcm1080.distor[1], rgb_pdcm1080.distor[2], rgb_pdcm1080.distor[3], rgb_pdcm1080.distor[4]])
        }
        camera_extrinsic = {
            "R": np.array(rgb_transform[0].rotation).reshape(3,3),
            "T": np.array(rgb_transform[0].translation)
        }
        return camera_intrinsic, camera_extrinsic

    def get_rgb_image(self):
        """获取RGB图像"""
        rgb_width, rgb_height, self.rgb_hostTimestamp, _, _, rgb_data, rgb_dataSize = xvsdk.xv_get_rgb()
        
        if rgb_dataSize.value > 0:
            color_image = np.asfortranarray(rgb_data)
            nheight = int(rgb_height.value*3/2)
            color_image = color_image[:rgb_width.value*nheight]
            color_image = color_image.reshape(nheight, rgb_width.value)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_YUV2RGB_IYUV) 
            color_image = cv2.flip(color_image, 0)
            # cv2.imwrite("color_image.jpg", color_image)
            return color_image
        return None

    def draw_background(self, surface):
        """绘制背景图��"""
        glDisable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.texture)
        texture_data = pygame.image.tostring(surface, 'RGB', 1)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self.display[0], self.display[1], 0, GL_RGB, GL_UNSIGNED_BYTE, texture_data)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0, -1.0, -1.0)
        glTexCoord2f(1.0, 1.0); glVertex3f(1.0, -1.0, -1.0)
        glTexCoord2f(1.0, 0.0); glVertex3f(1.0, 1.0, -1.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, 1.0, -1.0)
        glEnd()
        
        glDisable(GL_TEXTURE_2D)
        glEnable(GL_DEPTH_TEST)

    def get_slam_6dof_matrix(self):
        """获取SLAM 6DOF位姿并构建模型矩阵"""
        position, orientation, quaternion, slam_edgeTimestamp, slam_hostTimestamp, slam_confidence = xvsdk.xv_get_6dof()
        print("6dof: position x =", position.x , ", y = ", position.y, ", z = ", position.z, ", pitch =", orientation.x * 180 / 3.14 , ", yaw = ", orientation.y * 180 / 3.14, ", roll = ", orientation.z * 180 / 3.14, ", quaternion[0] = ", quaternion.q0, ", quaternion[1] = ", quaternion.q1, ", quaternion[2] = ", quaternion.q2, ", quaternion[3] = ", quaternion.q3, ", edgeTimestamp = ", slam_edgeTimestamp.value, ", hostTimestamp = ", slam_hostTimestamp.value, ", confidence = ", slam_confidence.value, "\n")
        
        R = np.array([
            [1 - 2*(quaternion.q2**2 + quaternion.q3**2), 2*(quaternion.q1*quaternion.q2 - quaternion.q0*quaternion.q3), 2*(quaternion.q1*quaternion.q3 + quaternion.q0*quaternion.q2)],
            [2*(quaternion.q1*quaternion.q2 + quaternion.q0*quaternion.q3), 1 - 2*(quaternion.q1**2 + quaternion.q3**2), 2*(quaternion.q2*quaternion.q3 - quaternion.q0*quaternion.q1)],
            [2*(quaternion.q1*quaternion.q3 - quaternion.q0*quaternion.q2), 2*(quaternion.q2*quaternion.q3 + quaternion.q0*quaternion.q1), 1 - 2*(quaternion.q1**2 + quaternion.q2**2)]
        ])
        
        T = np.array([position.x, position.y, position.z])
        
        # 对model_matrix进行矫正，将xv相机坐标系转换为OpenGL坐标系
        correction_matrix = np.diag([1, -1, -1])
        R = np.dot(correction_matrix, R)
        T = np.dot(correction_matrix, T)
        
        # 记录下第一帧slam的结果
        if(np.sum(np.abs(T)) != 0) and self.first_slam_R is not None:
            self.first_slam_R = R
            self.first_slam_T = T
        return R,T

    def set_projection_by_cam_intrinsic1(self, camera_intrinsic):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fx = camera_intrinsic["fx"]
        fy = camera_intrinsic["fy"]
        fovy = 2 * np.arctan(0.5 * camera_intrinsic["height"] / fy) * 180 / np.pi
        aspect = (camera_intrinsic["width"] * fy) / (camera_intrinsic["height"] * fx)
        gluPerspective(fovy, aspect, 0.1, 1000.0)

    def set_projection_by_cam_intrinsic2(self, K):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        near, far = 0.1, 100.0
        width, height = self.display
        
        # 设置平头视锥体，代表投影矩阵
        glFrustum(
            -cx * near / fx,            # 左
            (width - cx) * near / fx,   # 右
            -(height - cy) * near / fy, # 下
            cy * near / fy,             # 上
            near,                       # 近
            far)                        # 远

    def set_modelview_by_RT(self, R, t):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        rot_mat = np.eye(4)
        rot_mat[:3, :3] = R
        trans_mat = np.eye(4)
        trans_mat[:3, 3] = t
        glMultMatrixf(rot_mat.T)
        glMultMatrixf(trans_mat.T)

    def run(self):
        """主循环"""
        running = True
        while running:
            # 处理pygame事件
            for event in pygame.event.get():
                # 如果接收到退出事件，结束主循环
                if event.type == pygame.QUIT:
                    running = False

            # 获取RGB图像
            color_image = self.get_rgb_image()
            if color_image is not None:
                # 调整图像大小以适应显示窗口
                # 将图像转换为pygame surface对象
                surface = pygame.surfarray.make_surface(color_image.swapaxes(0,1))

            # 清除颜色缓冲区和深度缓冲区
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # 我们将视频流以正交投影的方式渲染在视窗中，且这个画面在后续保持不变
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            glOrtho(-1, 1, -1, 1, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            self.draw_background(surface)
            
            # 我们将mesh以透视投影的方式进行渲染，透视投影相机的投影参数和rgb相机一致 
            self.set_projection_by_cam_intrinsic1(self.camera_intrinsic)
            
            
            # 获取SLAM位姿矩阵
            slam_R, slam_T = self.get_slam_6dof_matrix() # w2c
            
            # 使用glLookAt设置相机位置
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            
            # 计算相机位置、目标点和上向量，相机在世界坐标系下的位置
            camera_pos = slam_T
            target_pos = np.dot(slam_R, np.array([0, 0, -1])) + slam_T  # 相机朝向z轴负方向
            up_vector = np.dot(slam_R, np.array([0, 1, 0]))   # 相机的上方向
            up_vector = up_vector / np.linalg.norm(up_vector)
            
            gluLookAt(
                camera_pos[0], camera_pos[1], camera_pos[2],
                target_pos[0], target_pos[1], target_pos[2],
                up_vector[0], up_vector[1], up_vector[2]
            )
            
            # 设置mesh在相机坐标系下的位置 view
            initial_transform = np.eye(4)
            initial_transform[:3, 3] = np.array([0, 0, -0.5])  # 将mesh放置在世界坐标系原点前方0.5米处
            
            # 绘制mesh
            self.draw_mesh(initial_transform)
            
            pygame.display.flip()

        # 清理资源
        self.vertices_vbo.delete()
        self.normals_vbo.delete()
        self.faces_vbo.delete()
        glDeleteTextures([self.texture])
        glDeleteProgram(self.shader_program)

        pygame.quit()
        xvsdk.stop()

if __name__ == "__main__":
    ar_camera = ARCamera()
    ar_camera.run()