import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.subscription = self.create_subscription(
            Image,
            '/camera_visual_servo/image_raw',  # change this
            self.listener_callback,
            10
        )
        self.saved = False

    def listener_callback(self, msg):
        if self.saved:
            return

        # Convert raw bytes → numpy array
        img = np.frombuffer(msg.data, dtype=np.uint8)

        # reshape based on encoding
        if msg.encoding == 'rgb8':
            img = img.reshape((msg.height, msg.width, 3))
        elif msg.encoding == 'bgr8':
            img = img.reshape((msg.height, msg.width, 3))
            img = img[:, :, ::-1]  # BGR → RGB
        elif msg.encoding == 'mono8':
            img = img.reshape((msg.height, msg.width))
        else:
            self.get_logger().error(f'Unsupported encoding: {msg.encoding}')
            return

        # Save using PIL (no OpenCV)
        from PIL import Image as PILImage
        PILImage.fromarray(img).save('captured_image.png')

        self.get_logger().info('Saved image!')
        self.saved = True
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ImageSaver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
