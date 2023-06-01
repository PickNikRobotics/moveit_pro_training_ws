"""
ROS 2 Node with a service client to test AprilTag detections.
"""

from apriltag_ros_msgs.srv import GetAprilTagDetections
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from threading import Thread


class AprilTagDetectionClient(Node):
    def __init__(self):
        super().__init__("apriltag_detection_client")

        self.client = self.create_client(GetAprilTagDetections, "detect_apriltags")
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for service...")

        self.rate = self.create_rate(1.0)

    def send_request(self):
        self.get_logger().info("Requesting AprilTag detection...")

        # Get camera info
        self.camera_info = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/wrist_mounted_camera/color/camera_info",  # TODO: Do not hardcode
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info("Waiting for camera info...")

        # Get image from camera
        self.latest_image = None
        self.image_sub = self.create_subscription(
            Image,
            "/wrist_mounted_camera/color/image_raw",  # TODO: Do not hardcode
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Wait to have data
        while not self.camera_info:
            self.get_logger().info("Waiting for camera info...")
            self.rate.sleep()
        while not self.latest_image:
            self.get_logger().info("Waiting for image...")
            self.rate.sleep()

        # Package and send a request
        request = GetAprilTagDetections.Request(
            camera_info=self.camera_info,
            image=self.latest_image,
        )
        self.future = self.client.call_async(request)
        while not self.future.done():
            self.rate.sleep()

        response = self.future.result()
        print(f"Got response:\n{response}")
        return response

    def camera_info_callback(self, msg):
        self.get_logger().info("Got camera info")
        self.camera_info = msg
        self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, msg):
        self.get_logger().info("Got image")
        self.latest_image = msg
        self.destroy_subscription(self.image_sub)


def main():
    rclpy.init()

    node = AprilTagDetectionClient()

    Thread(target=node.send_request).start()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
