"""This module contains implementation of the GripperControlServer for ROS 2."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory

from .gripper_controller import OnRobotDriver



# def _get_joint_info(joint_name):
#     limit = None
#     mimics = {}

#     robot =  get_package_share_directory("prl_ur5_descriptio") + '/' + "urdf/mantis.urdf"
#     for joint in robot.joints:
#         if joint.name == joint_name:
#             limit = joint.limit
#         elif joint.mimic is not None:
#             if joint.mimic.joint == joint_name:
#                 mimics[joint.name] = joint.mimic

#     if limit is None:
#         raise RuntimeError(
#             'Cannot find limits for joint "{}" in the robot description'.format(joint_name))

#     return limit, mimics

class GripperControlServer(Node):
    """The GripperControlServer class for ROS 2."""

    def __init__(self):
        super().__init__('gripper_control_server')

        # self.declare_parameter('state_publish_rate', 50)
        # self.declare_parameter('action_monitor_rate', 5)
        # self.declare_parameter('joint', 'gripper_joint')
        # self.declare_parameter('robot_description', '')

        self._state_publish_rate = 50.0
        self._action_monitor_rate = 5.0
        self._joint_name = "left_gripper_joint"
        self._limit_upper = 1.3
        self.limit_effort = 10

        # self._limit, self._mimics = _get_joint_info(self._joint_name, self._robot_description)

        self._gripper = OnRobotDriver()

        self._server = ActionServer(
            self,
            GripperCommand,
            'gripper_cmd',
            execute_callback=self.execute,
        )

        self._joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)

        timer_period = 1.0 / self._state_publish_rate
        # self.create_timer(timer_period, self.publish_state_cb)

    def execute(self, goal_handle):
        """Action server callback.

        Gripper has only two commands: fully open, fully close.

        Args:
            goal_handle: action goal handle
        """
        goal = goal_handle.request
        try:
            close = goal.command.position > 1e-3
            low_force_mode = goal.command.max_effort < 1e-3

            if close:
                self._gripper.close(low_force_mode, wait=False)
            else:
                self._gripper.open(low_force_mode, wait=False)

            rate = self._action_monitor_rate
            feedback_msg = GripperCommand.Feedback()
            # while rclpy.ok() and goal_handle.is_active:
            #     rclpy.sleep(1.0 / rate)

            #     width = (1.0 - self._gripper.opening) * self._limit.upper
            #     force = self._limit.effort * (0.125 if low_force_mode else 1.0)

            #     if self._gripper.is_ready:
            #         reached_goal = abs(goal.command.position - width) < 0.001
            #         stalled = not reached_goal

            #         result = GripperCommand.Result()
            #         result.position = width
            #         result.effort = force
            #         result.stalled = stalled
            #         result.reached_goal = reached_goal
            #         goal_handle.succeed()
            #         return result

            #     feedback_msg.position = width
            #     feedback_msg.effort = force
            #     feedback_msg.stalled = False
            #     feedback_msg.reached_goal = False
                # goal_handle.publish_feedback(feedback_msg)
        except Exception as ex:
            self.get_logger().error(f'Gripper command failed: {ex}')
            goal_handle.abort()
        return GripperCommand.Result()

    # def publish_state_cb(self):
    #     """Periodic task to publish the joint states message."""
    #     try:
    #         width = (1.0 - self._gripper.opening) * self._limit.upper

    #         msg = JointState()
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.name.append(self._joint_name)
    #         msg.position.append(width)
    #         msg.velocity.append(0.0)
    #         msg.effort.append(0.0)

    #         for name, mimic in self._mimics.items():
    #             msg.name.append(name)
    #             msg.position.append(width * mimic.multiplier + mimic.offset)
    #             msg.velocity.append(0.0)
    #             msg.effort.append(0.0)

    #         self._joint_states_pub.publish(msg)
    #     except Exception as ex:
    #         self.get_logger().error(f'Joint state publish failed: {ex}')



def main(args=None):
    rclpy.init(args=args)
    node = GripperControlServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()