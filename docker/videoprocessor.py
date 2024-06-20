import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import ffmpeg
import numpy as np
import asyncio
from janus_client import JanusSession, JanusVideoRoomPlugin

class VideoProcessor(Node):
    def __init__(self):
        super().__init__('video_processor')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam/image_raw/compressed',
            self.listener_callback,
            10)
        self.url_publisher = self.create_publisher(String, '/compressed_video/webrtc', 10)
        self.bridge = CvBridge()
        self.process = None
        self.frame_size = None
        self.streaming_url = ''
        self.room_id = 1234  # Replace with your actual room ID
        asyncio.run(self.publish_streaming_url())

    def start_ffmpeg_process(self, width, height):
        self.get_logger().info("Starting ffmpeg process to stream video to Janus Gateway")
        return (
            ffmpeg
            .input('pipe:', format='rawvideo', pix_fmt='bgr24', s=f'{width}x{height}', r=20)
            .output('rtp://localhost:10000', vcodec='libx264', pix_fmt='yuv420p', r=20, f='rtp', payload_type=96)
            .overwrite_output()
            .run_async(pipe_stdin=True)
        )

    def listener_callback(self, msg):
        try:
            # Convert compressed image message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)  # This loads the image in BGR format

            # Get frame size
            height, width, _ = cv_image.shape
            if self.frame_size is None:
                self.frame_size = (width, height)
                self.process = self.start_ffmpeg_process(width, height)
            elif self.frame_size != (width, height):
                self.frame_size = (width, height)
                self.restart_ffmpeg_process(width, height)

            # Write frame to ffmpeg process
            self.process.stdin.write(
                cv_image
                .astype(np.uint8)
                .tobytes()
            )
        except Exception as e:
            self.get_logger().error(f"Failed to process video frame: {str(e)}")
            self.restart_ffmpeg_process(height, width)

    def restart_ffmpeg_process(self, width, height):
        self.get_logger().info("Restarting ffmpeg process.")
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        self.process = self.start_ffmpeg_process(width, height)

    async def publish_streaming_url(self):
        self.get_logger().info("Connecting to Janus Gateway...")
        session = JanusSession(base_url="ws://localhost:8188/janusbasews/janus")
        plugin = JanusVideoRoomPlugin()
        await plugin.attach(session=session)
        
        # Now, join the room
        success = await plugin.join(room_id=self.room_id, display_name='ROS Video Stream')
        
        if success:
            self.streaming_url = f"ws://localhost:8188/janusbasews/janus"  # This should be the WebRTC URL
            self.get_logger().info(f"Publishing streaming URL: {self.streaming_url}")
            url_msg = String()
            url_msg.data = self.streaming_url
            self.url_publisher.publish(url_msg)
        else:
            self.get_logger().error("Failed to join the Janus video room")

    def destroy_node(self):
        if self.process:
            self.process.stdin.close()
            self.process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    video_processor = VideoProcessor()
    rclpy.spin(video_processor)
    video_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
