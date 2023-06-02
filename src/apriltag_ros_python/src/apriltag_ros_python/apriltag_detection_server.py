"""
ROS 2 Node hosting a service that performs AprilTag detections.
"""

import cv2
import cv_bridge
import dt_apriltags
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from apriltag_ros_msgs.msg import Detection
from apriltag_ros_msgs.srv import GetAprilTagDetections
from geometry_msgs.msg import PoseStamped


class AprilTagDetectionServer(Node):
    def __init__(self, visualize=False):
        super().__init__("apriltag_detection_server")
        self.visualize = visualize

        self.bridge = cv_bridge.CvBridge()
        self.detector = dt_apriltags.Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )
        self.apriltag_size = 1.0

        self.server = self.create_service(
            GetAprilTagDetections, "detect_apriltags", self.get_detections_callback
        )
        self.get_logger().info("Started AprilTag detection server.")

    def get_detections_callback(self, request, response):
        self.get_logger().info("Got incoming request")

        # Detect AprilTags
        img = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(
            img_gray,
            estimate_tag_pose=True,
            camera_params=self.msg_to_camera_params(request.camera_info),
            tag_size=self.apriltag_size,
        )
        self.get_logger().info(f"Detected {len(tags)} AprilTags.")
        if self.visualize:
            self.visualize_tags(img, tags)

        # Package the detections into the service response
        frame_id = request.camera_info.header.frame_id
        for tag in tags:
            response.detections.append(self.tag_to_msg(tag, frame_id))
        return response

    def msg_to_camera_params(self, msg):
        """
        Converts a ROS CameraInfo message to AprilTag detector camera parameters.
        """
        return (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def tag_to_msg(self, tag, frame_id):
        """
        Converts a single AprilTag detection to a ROS message.
        """
        msg = Detection()
        msg.family = tag.tag_family.decode()
        msg.id = tag.tag_id

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        position = tag.pose_t.flatten()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        q = Rotation.from_matrix(tag.pose_R.astype(np.float32)).as_quat()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        msg.pose = pose
        return msg

    def visualize_tags(self, img, tags):
        """Helper function to visualize detected AprilTags with OpenCV."""
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(
                    img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0),
                )

            cv2.putText(
                img,
                str(tag.tag_id),
                org=(
                    tag.corners[0, 0].astype(int) + 10,
                    tag.corners[0, 1].astype(int) + 10,
                ),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255),
            )

        cv2.imshow("Detected tags", img)
        cv2.waitKey(1000)  # Small delay to show the window


def main(args=None):
    rclpy.init(args=args)

    node = AprilTagDetectionServer(visualize=True)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
