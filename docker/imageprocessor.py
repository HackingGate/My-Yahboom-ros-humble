import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import numpy as np

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, '/compressed_video/webp', 10)
        self.bridge = CvBridge()
        self.compression_quality = 50  # Set the desired compression quality (0-100)
        self.lock = threading.Lock()  # To ensure thread safety

    def listener_callback(self, msg):
        threading.Thread(target=self.process_and_publish_image, args=(msg,)).start()

    def process_and_publish_image(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

            # Resize image to 320x240
            cv_image = cv2.resize(cv_image, (320, 240))

            # Check average brightness
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            avg_brightness = np.mean(gray_image)

            # Enhance image brightness using CLAHE if it is too dark
            if avg_brightness < 50:  # Adjust threshold as needed
                lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
                l, a, b = cv2.split(lab)
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                cl = clahe.apply(l)
                limg = cv2.merge((cl, a, b))
                cv_image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

            # Encode OpenCV image to WebP format with specified quality
            encode_param = [int(cv2.IMWRITE_WEBP_QUALITY), self.compression_quality]
            result, encoded_image = cv2.imencode('.webp', cv_image, encode_param)

            if result:
                # Create a new CompressedImage message
                compressed_image_msg = CompressedImage()
                compressed_image_msg.header = msg.header
                compressed_image_msg.format = 'webp'
                compressed_image_msg.data = encoded_image.tobytes()

                # Acquire lock to publish the compressed image
                with self.lock:
                    self.publisher_.publish(compressed_image_msg)
            else:
                self.get_logger().error("Failed to encode image to WebP")
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
