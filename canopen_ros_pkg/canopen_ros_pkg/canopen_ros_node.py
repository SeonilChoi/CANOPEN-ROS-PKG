import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from canopen_msgs.msg import MotorState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import sys
from functools import partial
from canopen_sdk.manager import load_motor_manager

PI = 3.141592653589793

class CANopenROSNode(Node):
    QOS_REKL5V = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        durability=QoSDurabilityPolicy.VOLATILE
    )

    def __init__(self):
        super().__init__('canopen_ros_node')

        self.declare_parameter('motor_config_file_path', '')
        motor_config_file_path = self.get_parameter(
            'motor_config_file_path').get_parameter_value().string_value
        
        try:
            self.motor_manager = load_motor_manager(motor_config_file_path, channel='can1')
            self.motor_manager.start_sync_all_motors(interval=0.01)
        except Exception as e:
            self.get_logger().error(f'\n[CANopenROSNode::init] ❌ Failed to initialize motors: {e}')

        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', self.QOS_REKL5V
        )
        self.motor_state_publisher = self.create_publisher(
            MotorState, 'motor_states', self.QOS_REKL5V
        )
        self.timer = self.create_timer(
            0.02, self.publish_joint_state
        )
        self.joint_trajectory_subscriber = self.create_subscription(
            JointTrajectory, 'joint_trajectory',
            self.joint_trajectory_callback, self.QOS_REKL5V
        )

        for motor_name in self.motor_manager.name_to_id.keys():
            callback = partial(self.joint_command_callback, name=motor_name)
            self.create_subscription(
                Float64, f'{motor_name}/command',
                callback, self.QOS_REKL5V
            )

        states, self.is_error, error_code = self.motor_manager.check_motor_states()
        self.check_error("init")
        
        self.get_logger().info(f'\n[CANopenROSNode::init] ✅ Initialization complete')
        self.position_range_limits = self.motor_manager.get_position_range_limits()
        
    def publish_joint_state(self):
        try:
            states, self.is_error, error_codes = self.motor_manager.check_motor_states()
        except Exception as e:
            self.get_logger().error(f'\n[CANopenROSNode::publish_joint_state] ❌ Failed to read motor info: {e}')
        self.check_error("publish_joint_state")
            
        joint_msg = JointState()
        motor_msg = MotorState()

        joint_msg.header.stamp = self.get_clock().now().to_msg()
        motor_msg.stamp = self.get_clock().now().to_msg()
        
        for motor_name, motor_id in self.motor_manager.name_to_id.items():
            joint_msg.name.append(motor_name)
            joint_msg.position.append(states[motor_name]['position'])
            joint_msg.velocity.append(states[motor_name]['velocity'])
            joint_msg.effort.append(states[motor_name]['torque'])
            
            motor_msg.id.append(motor_id)
            motor_msg.name.append(motor_name)
            motor_msg.position.append(states[motor_name]['position'] / PI * 180.0)
            motor_msg.velocity.append(states[motor_name]['velocity'])
            motor_msg.torque.append(states[motor_name]['torque'])
            motor_msg.min_position_limit.append(self.position_range_limits[motor_name][0] / PI * 180.0)
            motor_msg.max_position_limit.append(self.position_range_limits[motor_name][1] / PI * 180.0)
            motor_msg.status_word.append(states[motor_name]['statusword'])
            motor_msg.error_code.append(error_codes[motor_name])

        self.joint_state_publisher.publish(joint_msg)
        self.motor_state_publisher.publish(motor_msg)

    def joint_trajectory_callback(self, msg):
        self.check_error("joint_trajectory_callback")
        for idx, joint_name in enumerate(msg.joint_names):
            goal_position = msg.points[0].positions[idx]
            try:
                self.motor_manager.set_position(joint_name, goal_position)
            except Exception as e:
                self.get_logger().error(f'\n[CANopenROSNode::joint_trajectory_callback] ❌ Failed to set goal position for {joint_name}: {e}')
        
    def joint_command_callback(self, msg, name):
        self.check_error("joint_command_callback")
        try:
            self.motor_manager.set_position(name, msg.data)
        except Exception as e:
            self.get_logger().error(f'\n[CANopenROSNode::joint_command_callback] ❌ Failed to set goal position for {joint_name}: {e}')
            
    def check_error(self, func_name):
        if self.is_error:
            self.get_logger().error(f'\n[CANopenROSNode::{func_name}] ❌ Invalid motor states')
            self.motor_manager.stop_sync_all_motors()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    
def main(args=None):
    rclpy.init(args=args)
    context = rclpy.get_default_context()
    try:
        canopen_ros_node = CANopenROSNode()
        rclpy.spin(canopen_ros_node)
    except KeyboardInterrupt:
        print("[main] ⚠️  KeyboardInterrupt detected, shutting down...\n", flush=True)
    finally:
        print("[main] 🛑 Stop Motor Manager...", flush=True)
        canopen_ros_node.motor_manager.stop_sync_all_motors()
        canopen_ros_node.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)
