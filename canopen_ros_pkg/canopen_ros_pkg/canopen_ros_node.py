import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from functools import partial
from canopen_sdk.manager import load_motor_manager

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
            self.motor_manager = load_motor_manager(motor_config_file_path)
            self.motor_manager.start_sync_all_motors(interval=0.01)
        except Exception as e:
            self.get_logger().error(f'[CANopenROSNode::init] Failed to initialize motors: {e}')

        self.joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', self.QOS_REKL5V
        )
        self.timer = self.create_timer(
            0.01, self.publish_joint_state
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

        self.is_error, _ = self.motor_manager.check_motor_states()

    def publish_joint_state(self):
        self.is_error, _ = self.motor_manager.check_motor_states()
        if self.is_error:
            self.motor_manager.stop_sync_all_motors()
            self.get_logger().info(f'[CANopenROSNode::publish_joint_state] Invalid motor states')
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            positions = self.motor_manager.get_positions()
            velocities = self.motor_manager.get_velocities()
            torques = self.motor_manager.get_torques()
        except Exception as e:
            self.get_logger().error(f'[CANopenROSNode::publish_joint_state] Failed to read motor info: {e}')
        
        for motor_name in self.motor_manager.name_to_id.keys():
            msg.name.append(motor_name)
            msg.position.append(positions[motor_name])
            msg.velocity.append(velocities[motor_name])
            msg.effort.append(torques[motor_name])
        self.joint_state_publisher.publish(msg)

    def joint_trajectory_callback(self, msg):
        if self.is_error:
            self.motor_manager.stop_sync_all_motors()
            self.get_logger().info(f'[CANopenROSNode::publish_joint_state] Invalid motor states')
        
        for idx, joint_name in enumerate(msg.joint_names):
            goal_position = msg.points[0].positions[idx]
            try:
                self.motor_manager.set_position(joint_name, goal_position)
            except Exception as e:
                self.get_logger().error(f'[CANopenROSNode::joint_trajectory_callback] Failed to set goal position for {joint_name}: {e}')
        
    def joint_command_callback(self, msg, name):
        if self.is_error:
            self.motor_manager.stop_sync_all_motors()
            self.get_logger().info(f'[CANopenROSNode::publish_joint_state] Invalid motor states')
        
        try:
            self.motor_manager.set_position(name, msg.data)
        except Exception as e:
                self.get_logger().error(f'[CANopenROSNode::joint_command_callback] Failed to set goal position for {joint_name}: {e}')
    
def main(args=None):
    rclpy.init(args=args)
    context = rclpy.get_default_context()
    try:
        canopen_ros_node = CANopenROSNode()
        rclpy.spin(canopen_ros_node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected, shutting down...")
    finally:
        canopen_ros_node.motor_manager.stop_sync_all_motors()
        canopen_ros_node.destroy_node()
        if rclpy.ok(context=context):
            rclpy.shutdown(context=context)