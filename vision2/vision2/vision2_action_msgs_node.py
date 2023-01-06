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
        # calculate projected goal position based on expression.. 
        # happy look towards? neutral reset position? angry -> lookaway?
        # goal is https://docs.ros2.org/foxy/api/trajectory_msgs/msg/JointTrajectoryPoint.html "positions"

        # Need to check if server is available, not spam...

        msg1 = faces_details.faces
        print(" ")
        print(faces_details)
        box_h = 0
        msg2 = msg1
        for face in msg1:
            if not msg2:
                msg2 = face
                box_h = ((face.top_left.x-face.bottom_right.x)**2 +(face.top_left.x-face.bottom_right.x)**2)**0.5
                print(box_h)
            else:
                if box_h < ((face.top_left.x-face.bottom_right.x)**2 +(face.top_left.x-face.bottom_right.x)**2)**0.5:
                    msg2 = face

        emotion = msg2.emotion

        if emotion == 'Neutral':
            goal = [0.0, 0.0, 0.0, 0.0]
        elif emotion == 'Happy':
            goal = [0.5, 0.0, 0.0, 0.0]
        else:
            goal = [0.0, 0.0, 0.0, 0.0]

        self._action_client.wait_for_server()
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
