#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from functools import partial
from canopen_sdk.manager import load_motor_manager
from threading import Lock

PI = 3.141592653589793

class CANopenROSNode():
    def __init__(self):
        rospy.init_node('canopen_ros_node', anonymous=True)
        self.lock = Lock()

        motor_config_file_path = rospy.get_param('~motor_config_file_path', '')
        channel = rospy.get_param('~channel', 'can0')

        try:
            self.motor_manager = load_motor_manager(motor_config_file_path, channel=channel)
            self.motor_manager.start_sync_all_motors(interval=0.01)
        except Exception as e:
            rospy.logerr(f'\n[CANopenROSNode::init] ❌ Failed to initialize motors: {e}')

        self.joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=5)
        rospy.Subscriber('joint_trajectory', JointTrajectory, self.joint_trajectory_callback)

        for motor_name in self.motor_manager.name_to_id.keys():
            rospy.Subscriber(f'{motor_name}/command', Float64,
                             partial(self.joint_command_callback, name=motor_name))

        states, self.is_error, error_code = self.motor_manager.check_motor_states()
        self.check_error('init')

        rospy.loginfo('\n[CANopenROSNode::init] ✅ Initialization complete')
        self.position_range_limits = self.motor_manager.get_position_range_limits()

        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_joint_state)

    def publish_joint_state(self, event):
        with self.lock:
            try:
                states, self.is_error, error_codes = self.motor_manager.check_motor_states()
            except Exception as e:
                rospy.logerr(f'\n[publish_joint_state] ❌ Failed to read motor info: {e}')
                return
            self.check_error("publish_joint_state")

            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            
            for motor_name, motor_id in self.motor_manager.name_to_id.items():
                joint_msg.name.append(motor_name)
                joint_msg.position.append(states[motor_name]['position'])
                joint_msg.velocity.append(states[motor_name]['velocity'])
                joint_msg.effort.append(states[motor_name]['torque'])

            self.joint_state_publisher.publish(joint_msg)

    def joint_trajectory_callback(self, msg):
        self.check_error("joint_trajectory_callback")
        for idx, joint_name in enumerate(msg.joint_names):
            goal_position = msg.points[0].positions[idx]
            try:
                self.motor_manager.set_position(joint_name, goal_position)
            except Exception as e:
                rospy.logerr(f'\n[joint_trajectory_callback] ❌ Failed to set goal for {joint_name}: {e}')

    def joint_command_callback(self, msg, name):
        self.check_error("joint_command_callback")
        try:
            self.motor_manager.set_position(name, msg.data)
        except Exception as e:
            rospy.logerr(f'\n[joint_command_callback] ❌ Failed to set goal for {name}: {e}')

    def check_error(self, func_name):
        if self.is_error:
            rospy.logerr(f'[CANopenROSNode::{func_name}] ❌ Invalid motor states')
            self.motor_manager.stop_sync_all_motors()

def main():
    node = CANopenROSNode()
    rospy.on_shutdown(lambda: node.motor_manager.stop_sync_all_motors())
    rospy.spin()

if __name__ == '__main__':
    main()
