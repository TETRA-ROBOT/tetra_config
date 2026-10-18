#!/usr/bin/env python3
"""TF 에서 로봇 위치를 읽어 /footprint (geometry_msgs/Pose) 로 발행한다.

웹 관제(tetra-single-api)는 로봇 위치를 /footprint 토픽으로만 받는데, 이 토픽을 발행하는 노드가 없어
화면의 로봇이 원점에 고정되고 "현재 위치 저장"이 (0,0) 으로 저장됐다.
지도 좌표(map → base_footprint)를 우선 쓰고, SLAM/AMCL 이 아직 없으면 odom 좌표로 대신 발행한다.
"""

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class RobotPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_pose_publisher')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 10.0)

        self.global_frame: str = self.get_parameter('global_frame').value
        self.fallback_frame: str = self.get_parameter('fallback_frame').value
        self.base_frame: str = self.get_parameter('base_frame').value
        publish_rate: float = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(Pose, 'footprint', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_pose)
        self.last_reference_frame = ''

    def lookup_pose(self, reference_frame: str) -> Pose | None:
        try:
            transform = self.tf_buffer.lookup_transform(reference_frame, self.base_frame, Time())
        except TransformException:
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def publish_pose(self) -> None:
        for reference_frame in (self.global_frame, self.fallback_frame):
            pose = self.lookup_pose(reference_frame)
            if pose is None:
                continue
            if reference_frame != self.last_reference_frame:
                self.get_logger().info(f'publishing /footprint in {reference_frame} frame')
                self.last_reference_frame = reference_frame
            self.pose_publisher.publish(pose)
            return


def main() -> None:
    rclpy.init()
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
