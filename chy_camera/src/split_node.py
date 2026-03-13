import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
import numpy as np
import yaml
import os
import copy  # ✅ 新增：深拷贝模块

class StereoSplitNode(Node):
    def __init__(self):
        super().__init__('stereo_split_node')
        self.bridge = CvBridge()
        

        self.declare_parameter('left_calibration_file', '/home/disco/study/orbslam/usb_camera/src/stereo_image_splitter/calibrationdata/calibrationdata/left.yaml') #相机标定文件
        self.declare_parameter('right_calibration_file', '/home/disco/study/orbslam/usb_camera/src/stereo_image_splitter/calibrationdata/calibrationdata/right.yaml')

        left_calib_file = self.get_parameter('left_calibration_file').value
        right_calib_file = self.get_parameter('right_calibration_file').value

        
        # ✅ 修复：QoS配置
        camera_qos = QoSProfile(
            depth=30,
            reliability=ReliabilityPolicy.RELIABLE,  
            durability=DurabilityPolicy.VOLATILE  
        )

        # ✅ 修复：所有发布者/订阅者必须使用相同的 qos_profile
        self.subscription = self.create_subscription(
            Image,
            '/stereo/image_raw',
            self.image_callback,
            camera_qos
        )
        
        self.left_pub = self.create_publisher(Image, '/stereo/left/image_raw', camera_qos)
        self.right_pub = self.create_publisher(Image, '/stereo/right/image_raw', camera_qos)


        # ✅ 发布 camera_info
        self.left_info_pub = self.create_publisher(CameraInfo, '/stereo/left/camera_info', camera_qos)
        self.right_info_pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', camera_qos)
        
        # ✅ 加载标定信息
        self.left_camera_info  = self.load_camera_info(left_calib_file)
        self.right_camera_info  = self.load_camera_info(right_calib_file)

    def load_camera_info(self, file_path):
        """从YAML文件加载相机标定信息"""
        try:
            with open(file_path, 'r') as file:
                calib_data = yaml.safe_load(file)
            
            camera_info = CameraInfo()
            camera_info.width = calib_data['image_width']
            camera_info.height = calib_data['image_height']
            camera_info.distortion_model = calib_data['distortion_model']
            
            # 设置相机参数
            camera_info.k = calib_data['camera_matrix']['data']
            camera_info.d = calib_data['distortion_coefficients']['data']
            camera_info.r = calib_data['rectification_matrix']['data']
            camera_info.p = calib_data['projection_matrix']['data']
            
                        # ✅ 验证投影矩阵包含基线信息
            baseline = -camera_info.p[3] / camera_info.p[0]  # 从 P[0,3] = -f*Tx 计算
            self.get_logger().info(f'相机基线: {baseline:.3f}m')

            return camera_info
            
        except Exception as e:
            self.get_logger().error(f'加载相机标定文件失败 {file_path}: {e}')
            return None


    def image_callback(self, msg):
        try:
            # ✅ 使用当前时间戳确保同步
            # current_time = self.get_clock().now().to_msg()
            # self.get_logger().info(f'current_time ={current_time}')
            #✅ 验证图像尺寸与标定匹配
            if msg.width != self.left_camera_info.width * 2:
                self.get_logger().warn(
                    f'输入图像宽度({msg.width})不是标定宽度({self.left_camera_info.width})的2倍！'
                )
                return
            
            # # ✅ 解码 MJPEG 压缩图像
            # if msg.encoding == "8UC1":  # raw_mjpeg 的编码标识
            #     np_arr = np.frombuffer(msg.data, np.uint8)
            #     cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #     if cv_image is None:
            #         self.get_logger().error("MJPEG 解码失败")
            #         return
            # else:
            #     # 非压缩格式（备用）
            #     cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        # 解码图像
            if msg.encoding == "rgb8" or msg.encoding == "bgr8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                # MJPEG 解码
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is None:
                    self.get_logger().error("图像解码失败")
                    return
            height, width = cv_image.shape[:2]
            
            # 分割
            left_img = cv_image[:, :width // 2]
            right_img = cv_image[:, width // 2:]
            
            # ✅ 创建消息并深拷贝时间戳
            left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")


            # 在分割后
            # timestamp = self.get_clock().now().to_msg()  # 使用统一时间戳
            header = msg.header
            # header.stamp = current_time  # 强制同步时间戳
            header.frame_id = 'camera_link'


            left_msg.header = copy.deepcopy(header)
            right_msg.header = copy.deepcopy(header)
            
            # ✅ 为 CameraInfo 设置时间戳和 frame_id
            left_info = copy.deepcopy(self.left_camera_info)
            right_info = copy.deepcopy(self.right_camera_info)

            if left_info is not None and right_info is not None:
                left_info.header = header
                right_info.header = header
                left_info.header.frame_id = 'camera_link'  # 确保一致
                right_info.header.frame_id = 'camera_link'  # 确保一致
                
                # ✅  原子性发布（先发布 CameraInfo）
                self.left_info_pub.publish(left_info)
                self.right_info_pub.publish(right_info)
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)

                # ✅ 添加调试日志（可选）
                self.get_logger().debug(
                    f' 发布双目数据 at timestamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')

            
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

