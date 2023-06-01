"""
ROS 2 Node hosting a service that performs AprilTag detections.
"""

from apriltag_ros_msgs.msg import Detection
from apriltag_ros_msgs.srv import GetAprilTagDetections
import cv2
import cv_bridge
import dt_apriltags
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AprilTagDetectionServer(Node):
    def __init__(self):
        super().__init__("apriltag_detection_server")

        self.bridge = cv_bridge.CvBridge()
        self.detector = dt_apriltags.Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        self.server = self.create_service(
            GetAprilTagDetections, "detect_apriltags", self.get_detections_callback
        )
        self.get_logger().info("Started AprilTag detection server.")

    def get_detections_callback(self, request, response):
        self.get_logger().info("Got incoming request")

        img = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(img_gray, True)

        self.get_logger().info(f"Got tags:\n{tags}")
        return response


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagDetectionServer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
