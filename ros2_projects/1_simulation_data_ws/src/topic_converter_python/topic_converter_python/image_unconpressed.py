#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageDecompressor(Node):
    def __init__(self):
        super().__init__('image_decompressor')
        
        # 初始化CV桥
        self.bridge = CvBridge()
        
        # 创建订阅者，订阅压缩图像话题
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image/compressed',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        
        # 创建发布者，发布解压后的图像
        self.publisher = self.create_publisher(
            Image,
            '/camera/color/image/uncompressed',
            10)
        
        self.get_logger().info('图像解压节点已启动，等待压缩图像输入...')

    def listener_callback(self, msg):
        try:
            # 将压缩图像消息转换为OpenCV格式
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            
            # 将OpenCV图像转换为未压缩的图像消息
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # 复制原消息的头部信息（时间戳、坐标系等）
            image_msg.header = msg.header
            
            # 发布未压缩图像
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    image_decompressor = ImageDecompressor()
    
    try:
        rclpy.spin(image_decompressor)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理工作
        image_decompressor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


