import hashlib
import math
import uuid
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
import zenoh


@dataclass
class RobotState:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    frame_id: str
    topic: str


class DetectionNode(Node):
    """ROS 2 node that runs YOLO on camera frames and publishes detection events over Zenoh."""

    def __init__(self) -> None:
        super().__init__("maze_detection_node")

        # Parameters
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("robot_id", "tb3_sim")
        self.declare_parameter("run_id", str(uuid.uuid4()))
        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("zenoh_locator", "tcp/127.0.0.1:7447")

        self.camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        self.run_id = self.get_parameter("run_id").get_parameter_value().string_value
        self.yolo_model_path = self.get_parameter("yolo_model").get_parameter_value().string_value
        self.zenoh_locator = self.get_parameter("zenoh_locator").get_parameter_value().string_value

        self.bridge = CvBridge()
        self.latest_odom: Optional[Odometry] = None

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO model
        self.get_logger().info(f"Loading YOLO model: {self.yolo_model_path}")
        self.model = YOLO(self.yolo_model_path)

        # Zenoh session and publishers
        self.sequence = 0
        self.event_key_prefix = f"maze/{self.robot_id}/{self.run_id}"

        self.get_logger().info("Connecting to Zenoh...")
        self.zenoh_session = zenoh.open({"connect": [self.zenoh_locator]})
        self.get_logger().info("Zenoh session established.")

        # Explicit publisher for run metadata; detection events are written ad-hoc
        self.runmeta_key = f"{self.event_key_prefix}/runmeta/v1"
        self._publish_run_metadata_once()

        # ROS subscriptions
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.image_sub = self.create_subscription(Image, self.camera_topic, self._image_cb, 10)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _image_cb(self, msg: Image) -> None:
        # Time alignment: use image stamp, use latest odom available
        if self.latest_odom is None:
            self.get_logger().warn_throttle(5.0, "No odom yet, skipping image.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        # Compute SHA-256 of raw image buffer
        image_bytes = cv2.imencode(".png", cv_image)[1].tobytes()
        image_sha256 = hashlib.sha256(image_bytes).hexdigest()

        # Run YOLO
        results = self.model(cv_image, verbose=False)

        detections = self._extract_detections(results, msg.width, msg.height)

        # Build robot state from odom
        robot_state = self._build_robot_state(self.latest_odom)

        # Get TF base->camera
        t_base_camera, tf_ok = self._lookup_base_to_camera_transform(msg.header.stamp)

        # Build event JSON
        event_json = self._build_event_json(
            msg,
            image_sha256,
            robot_state,
            t_base_camera,
            tf_ok,
            detections,
        )

        # Publish over Zenoh
        event_id = event_json["event_id"]
        key = f"{self.event_key_prefix}/detections/v1/{event_id}"

        try:
            self.zenoh_session.put(key, zenoh.Value(json=event_json))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to publish detection event to Zenoh: {exc}")
            return

        self.sequence += 1

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------

    def _build_robot_state(self, odom: Odometry) -> RobotState:
        pose = odom.pose.pose
        twist = odom.twist.twist

        # Yaw from quaternion
        q = pose.orientation
        yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)

        return RobotState(
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw,
            vx=twist.linear.x,
            vy=twist.linear.y,
            wz=twist.angular.z,
            frame_id=odom.header.frame_id,
            topic=self.odom_topic,
        )

    def _quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        # Standard yaw extraction from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _lookup_base_to_camera_transform(
        self, stamp
    ) -> Tuple[Optional[List[float]], bool]:
        try:
            # Use TF2 to lookup transform from base to camera at the image timestamp
            ts: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                stamp,
            )
        except TransformException as exc:
            self.get_logger().warn_throttle(
                5.0, f"TF lookup failed for {self.base_frame}->{self.camera_frame}: {exc}"
            )
            return None, False

        t = ts.transform.translation
        r = ts.transform.rotation

        # Convert to 4x4 homogeneous transform (row-major) flattened to length-16 list
        # Translation
        tx, ty, tz = t.x, t.y, t.z

        # Rotation matrix from quaternion
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        # Precompute repeated terms
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wz)
        r02 = 2.0 * (xz + wy)

        r10 = 2.0 * (xy + wz)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wx)

        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)

        t_base_camera = [
            r00,
            r01,
            r02,
            tx,
            r10,
            r11,
            r12,
            ty,
            r20,
            r21,
            r22,
            tz,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        return t_base_camera, True

    def _extract_detections(self, results, width: int, height: int) -> List[dict]:
        detections: List[dict] = []
        if not results:
            return detections

        result = results[0]
        if result.boxes is None:
            return detections

        names = result.names

        for box in result.boxes:
            xyxy = box.xyxy.cpu().numpy().astype(float)[0].tolist()
            cls_id = int(box.cls)
            conf = float(box.conf)
            det_id = str(uuid.uuid4())

            detections.append(
                {
                    "det_id": det_id,
                    "class_id": cls_id,
                    "class_name": names.get(cls_id, str(cls_id)),
                    "confidence": conf,
                    "bbox_xyxy": xyxy,
                }
            )

        return detections

    def _build_event_json(
        self,
        img_msg: Image,
        image_sha256: str,
        robot_state: RobotState,
        t_base_camera: Optional[List[float]],
        tf_ok: bool,
        detections: List[dict],
    ) -> dict:
        event_id = str(uuid.uuid4())

        event = {
            "schema": "maze.detection.v1",
            "event_id": event_id,
            "run_id": self.run_id,
            "robot_id": self.robot_id,
            "sequence": self.sequence,
            "image": {
                "topic": self.camera_topic,
                "stamp": {
                    "sec": int(img_msg.header.stamp.sec),
                    "nanosec": int(img_msg.header.stamp.nanosec),
                },
                "frame_id": img_msg.header.frame_id,
                "width": int(img_msg.width),
                "height": int(img_msg.height),
                "encoding": img_msg.encoding,
                "sha256": image_sha256,
            },
            "odometry": {
                "topic": robot_state.topic,
                "frame_id": robot_state.frame_id,
                "x": float(robot_state.x),
                "y": float(robot_state.y),
                "yaw": float(robot_state.yaw),
                "vx": float(robot_state.vx),
                "vy": float(robot_state.vy),
                "wz": float(robot_state.wz),
            },
            "tf": {
                "base_frame": self.base_frame,
                "camera_frame": self.camera_frame,
                "t_base_camera": t_base_camera if t_base_camera is not None else [0.0] * 16,
                "tf_ok": bool(tf_ok),
            },
            "detections": detections,
        }
        return event

    def _publish_run_metadata_once(self) -> None:
        """Publish a single run metadata JSON blob for this run_id."""
        runmeta = {
            "schema": "maze.runmeta.v1",
            "run_id": self.run_id,
            "robot_id": self.robot_id,
        }
        try:
            self.zenoh_session.put(self.runmeta_key, zenoh.Value(json=runmeta))
            self.get_logger().info(f"Published run metadata to {self.runmeta_key}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to publish run metadata: {exc}")

    def destroy_node(self) -> None:
        try:
            self.zenoh_session.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

