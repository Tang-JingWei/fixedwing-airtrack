#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Header
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Point
import time
import math
from geometry_msgs.msg import Quaternion
from trackgui.msg import Target  # 导入自定义消息


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
            Target, '/target_info', self.target_callback, qos)
        self.target_detect_flag = False
        self.target_x = 0.0  # 目标 X 坐标
        self.target_y = 0.0  # 目标 Y 坐标
        self.target_received = False

        # 服务客户端
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # 发布 AttitudeTarget 指令
        self.attitude_pub = self.create_publisher(
            AttitudeTarget, '/mavros/setpoint_raw/attitude', qos)
        
        # 发布 PoseStamped 指令
        self.target_attitude_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_attitude/attitude', qos)

        # 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        # self.get_logger().info("Timer created with 10Hz")

        # 任务状态
        self.mission_state = "INIT"
        self.service_pending = None
        self.target_thrust = 0.7  # 固定推力值 (0.0-1.0)
        self.target_roll_rate = 0.0  # 目标滚转速率 (rad/s)
        self.target_pitch_rate = 0.0  # 目标俯仰速率 (rad/s)
        self.target_roll = 0.0  # 目标滚转角 (rad)
        self.target_pitch = 0.0  # 目标俯仰角 (rad)

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
        self.target_detect_flag = msg.detect_flag
        self.target_x = msg.cx
        self.target_y = msg.cy
        self.target_received = True
        # self.get_logger().info(f"{self.target_detect_flag} || Target position: x={self.target_x:.2f}, y={self.target_y:.2f}")

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

    def quaternion_from_euler(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def publish_target_attitude(self, roll, pitch, yaw, thrust):
        """发布姿态角和推力指令"""
        attitude = AttitudeTarget()
        attitude.header = Header()
        attitude.header.stamp = self.get_clock().now().to_msg()
        # 设置姿态四元数
        quaternion = self.quaternion_from_euler(roll, pitch, yaw)
        attitude.orientation = quaternion
        # 忽略角速度，只使用姿态角
        attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | \
                                AttitudeTarget.IGNORE_PITCH_RATE | \
                                AttitudeTarget.IGNORE_YAW_RATE
        attitude.thrust = thrust
        self.attitude_pub.publish(attitude)
        self.get_logger().info(f"Target_Attitude published: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}, thrust={thrust:.2f}")

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

    # PID算出速度
    def pid_Cal(self, cx, cy, roll_angle=0.0, pitch_angle=0.0):
        k = 50; # 中心%50

        Y_OVERLOOK_K = 12 # 图像中心点y坐标无动作范围	
        X_OVERLOOK_K = 15 # 图像中心点x坐标无动作范围

        # roll角度控制
        if (abs(cx - k) <= X_OVERLOOK_K):
            roll = 0.0
        elif (cx - k > X_OVERLOOK_K):
            roll = roll_angle
        elif (cx - k < -X_OVERLOOK_K):
            roll = -roll_angle

        # pitch角度控制
        if (abs(cy - k) <= Y_OVERLOOK_K):
            pitch = 0.0
        elif (cy - k > Y_OVERLOOK_K):
            pitch = pitch_angle
        elif (cy - k < -Y_OVERLOOK_K):
            pitch = -pitch_angle

        return roll, pitch

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
                # if not self.target_received:
                #     self.get_logger().warn("Waiting for target position data")
                #     return
                self.get_logger().info("Offboard activated, starting target tracking")
                self.mission_state = "TRACKING"

        elif self.mission_state == "TRACKING":
            # 目标追踪控制
            if self.target_detect_flag:
                # X 偏差控制 roll，Y 偏差控制 pitch
                self.target_roll, self.target_pitch = self.pid_Cal(cx = self.target_x, cy = self.target_y, roll_angle=0.3, pitch_angle=0.3)
                self.publish_target_attitude(roll=self.target_roll, pitch=self.target_pitch, yaw=0.0, thrust=self.target_thrust)
            else:
                # 无目标时，回归直线姿态
                self.publish_target_attitude(roll=0.0, pitch=0.0, yaw=0.0, thrust=self.target_thrust)

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