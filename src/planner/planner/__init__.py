#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Point
import time

class FixedWingTargetTracker(Node):
    def __init__(self):
        super().__init__('fixed_wing_target_tracker')

        # QoS 设置，与 MAVROS 匹配
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅飞控状态
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos)
        self.current_state = None
        self.previous_mode = None

        # 订阅当前高度
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_callback, qos)
        self.current_altitude = 0.0
        self.altitude_received = False

        # 订阅目标位置
        self.target_sub = self.create_subscription(
            Point, '/target_position', self.target_callback, qos)
        self.target_x = 0.0  # 目标 X 坐标 [-1, 1]
        self.target_y = 0.0  # 目标 Y 坐标 [-1, 1]
        self.target_received = False

        # 服务客户端
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # 发布 AttitudeTarget 指令
        self.attitude_pub = self.create_publisher(
            AttitudeTarget, '/mavros/setpoint_raw/attitude', qos)

        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        # self.get_logger().info("Timer created with 10Hz")

        # 任务状态
        self.mission_state = "INIT"
        self.service_pending = None
        self.target_thrust = 0.7  # 固定推力值 (0.0-1.0)
        self.target_roll_rate = 0.0  # 目标滚转速率 (rad/s)
        self.target_pitch_rate = 0.0  # 目标俯仰速率 (rad/s)

        # self.get_logger().info("Fixed wing target tracker initialized")

    def state_callback(self, msg):
        self.current_state = msg
        if self.mission_state in ["INIT", "IDLE"]:
            self.previous_mode = msg.mode
        # self.get_logger().info(f"Mode: {self.current_state.mode}, Armed: {self.current_state.armed}")

    def local_pos_callback(self, msg):
        self.current_altitude = msg.pose.position.z
        self.altitude_received = True
        # self.get_logger().info(f"Current altitude: {self.current_altitude:.2f} m")

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_received = True
        self.get_logger().info(f"Target position: x={self.target_x:.2f}, y={self.target_y:.2f}")

    def call_service_async(self, client, request, callback):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{client.srv_name} service not available")
            return False
        if self.service_pending is None:
            self.service_pending = client.call_async(request)
            self.service_pending.add_done_callback(callback)
            self.get_logger().info(f"Service {client.srv_name} called")
            return True
        return False

    def set_mode_callback(self, future):
        result = future.result()
        if result is not None and result.mode_sent:
            self.get_logger().info("Mode set successfully")
        else:
            self.get_logger().error("Failed to set mode")
        self.service_pending = None

    def arm_callback(self, future):
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info("Vehicle armed successfully")
        else:
            self.get_logger().error("Failed to arm vehicle")
        self.service_pending = None

    def publish_attitude(self, roll_rate, pitch_rate, thrust):
        """发布滚转速率、俯仰速率和推力指令"""
        attitude = AttitudeTarget()
        attitude.header = Header()
        attitude.header.stamp = self.get_clock().now().to_msg()
        # 忽略姿态和偏航速率，只使用 roll_rate 和 pitch_rate
        attitude.type_mask = (AttitudeTarget.IGNORE_ATTITUDE | AttitudeTarget.IGNORE_YAW_RATE)
        attitude.body_rate.x = roll_rate   # 滚转速率
        attitude.body_rate.y = pitch_rate  # 俯仰速率
        attitude.body_rate.z = 0.0         # 偏航速率固定为 0
        attitude.thrust = thrust
        self.attitude_pub.publish(attitude)
        self.get_logger().info(f"Attitude published: roll_rate={roll_rate:.2f}, pitch_rate={pitch_rate:.2f}, thrust={thrust:.2f}")

    def timer_callback(self):
        # self.get_logger().info(f"Timer triggered at {time.time():.3f}")
        if self.current_state is None:
            self.get_logger().warn("No state received yet")
            self.publish_attitude(roll_rate=0.0, pitch_rate=0.0, thrust=self.target_thrust)
            return

        if self.mission_state == "INIT":
            self.publish_attitude(roll_rate=0.0, pitch_rate=0.0, thrust=self.target_thrust)
            if not self.altitude_received:
                self.get_logger().warn("Waiting for altitude data")
                return
            if self.current_altitude < 1.0:
                self.get_logger().warn(f"Altitude too low: {self.current_altitude:.2f} m, waiting for takeoff")
                return
            self.get_logger().info("Data received, switching to --> IDLE")
            self.mission_state = "IDLE"

        elif self.mission_state == "IDLE":
            self.publish_attitude(roll_rate=0.0, pitch_rate=0.0, thrust=self.target_thrust)
            if self.current_state.mode != "OFFBOARD":
                self.call_service_async(
                    self.set_mode_client,
                    SetMode.Request(custom_mode="OFFBOARD"),
                    self.set_mode_callback
                )
            elif not self.current_state.armed:
                self.call_service_async(
                    self.arming_client,
                    CommandBool.Request(value=True),
                    self.arm_callback
                )
            elif self.service_pending is None:
                if not self.target_received:
                    self.get_logger().warn("Waiting for target position data")
                    return
                self.get_logger().info("Offboard activated, starting target tracking")
                self.mission_state = "TRACKING"

        elif self.mission_state == "TRACKING":
            # 目标追踪控制
            # X 偏差控制 roll_rate，Y 偏差控制 pitch_rate
            self.target_roll_rate = self.target_x * 0.5  # 滚转速率，范围 [-0.5, 0.5] rad/s
            self.target_pitch_rate = -self.target_y * 0.5  # 俯仰速率，范围 [-0.5, 0.5] rad/s
            self.publish_attitude(
                roll_rate=max(min(self.target_roll_rate, 0.5), -0.5),
                pitch_rate=max(min(self.target_pitch_rate, 0.5), -0.5),
                thrust=self.target_thrust
            )

    def shutdown_hook(self):
        if self.previous_mode and self.current_state and self.current_state.mode == "OFFBOARD":
            self.get_logger().info(f"Restoring previous mode: {self.previous_mode}")
            self.call_service_async(
                self.set_mode_client,
                SetMode.Request(custom_mode=self.previous_mode),
                self.set_mode_callback
            )
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = FixedWingTargetTracker()
    node.get_logger().info("Node starting to spin")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
        node.shutdown_hook()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()