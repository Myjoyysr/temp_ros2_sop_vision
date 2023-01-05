import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration

class Vision2ActionMsgs(Node):

    def __init__(self):
        super().__init__('vision2_action_msgs')

        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')

        self.subscriber = self.create_subscription(
            Faces,
            "/face_details",
            self.handle_action_msgs,
            10,
        )

    def handle_action_msgs(self, faces_details):
        # todo... 
        # parse message (face_details)...
        # Calculate closest person
        # calculate projected goal position based on expression.. 
        # happy look towards? neutral reset position? angry -> lookaway?
        # goal is https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html "positions"
        # print(faces_details)

        goal = [0.5, 0.0, 0.0, 0.0]
        self.send_goal(goal)

    def send_goal(self, goal):

        head_joints = ['head_pan_joint', 
                        'head_tilt_right_joint', 
                        'head_tilt_left_joint', 
                        'head_tilt_vertical_joint']

        # "correct" duration might be in lecture videos
        duration = Duration(sec=0,nanosec=0)
        msg1 = [JointTrajectoryPoint(positions=goal,time_from_start=duration)]
        msg2 = JointTrajectory(joint_names=head_joints,points=msg1)
        self._action_client.wait_for_server()
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg2

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    vision2_action_msgs = Vision2ActionMsgs()

    rclpy.spin(vision2_action_msgs)

    vision2_action_msgs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
