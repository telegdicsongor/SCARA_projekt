#!/usr/bin/env python3

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ControllerStateToJointStates(Node):
    def __init__(self):
        super().__init__('controller_state_to_joint_states')

        self.declare_parameter('controller_state_topic', '/arm_controller/controller_state')
        self.declare_parameter('joint_states_topic', '/joint_states')

        controller_state_topic = (
            self.get_parameter('controller_state_topic').get_parameter_value().string_value
        )
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value

        self.joint_states_publisher = self.create_publisher(JointState, joint_states_topic, 10)
        self.controller_state_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            controller_state_topic,
            self.publish_joint_states,
            10,
        )

        self.get_logger().info(
            f'Relaying {controller_state_topic} feedback positions to {joint_states_topic}'
        )

    def publish_joint_states(self, controller_state):
        joint_count = len(controller_state.joint_names)
        if joint_count == 0 or len(controller_state.feedback.positions) < joint_count:
            return

        joint_states = JointState()
        joint_states.header = controller_state.header
        joint_states.name = list(controller_state.joint_names)
        joint_states.position = list(controller_state.feedback.positions[:joint_count])

        if len(controller_state.feedback.velocities) >= joint_count:
            joint_states.velocity = list(controller_state.feedback.velocities[:joint_count])
        if len(controller_state.feedback.effort) >= joint_count:
            joint_states.effort = list(controller_state.feedback.effort[:joint_count])

        self.joint_states_publisher.publish(joint_states)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerStateToJointStates()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
