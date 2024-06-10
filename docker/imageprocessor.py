import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, '/compressed_video', 10)
        self.bridge = CvBridge()
        self.compression_quality = 80 # Set the desired compression quality (0-100)

    def listener_callback(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

            # Encode OpenCV image to WebP format with specified quality
            encode_param = [int(cv2.IMWRITE_WEBP_QUALITY), self.compression_quality]
            result, encoded_image = cv2.imencode('.webp', cv_image, encode_param)

            # Create a new CompressedImage message
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header = msg.header
            compressed_image_msg.format = 'webp'
            compressed_image_msg.data = encoded_image.tobytes()

            # Publish the compressed image
            self.publisher_.publish(compressed_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    image_compressor = ImageCompressor()
    rclpy.spin(image_compressor)
    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
