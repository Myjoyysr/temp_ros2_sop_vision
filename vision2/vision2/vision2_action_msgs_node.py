import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages

#Not sure if correct
from control_msgs.action import FollowJointTrajectory


class Vision2ActionMsgs(Node):

    def __init__(self):
        super().__init__('vision2_action_msgs')


        #need to find right parameters for actionclient
        self._action_client = ActionClient(self, FollowJointTrajectory, '/head_controller/follow_joint_trajectory')


        #msg sub
        self.subscriber = self.create_subscription(
            Faces,
            "/face_details",
            self.handle_action_msgs,
            10,
        )



    def handle_action_msgs(self, faces_details):

            #need to parse msgs -> send action to look towards if happy -> else reset pos for example
            #able to include hello voice for every happy detection with new id -> look towards say hello
        print(faces_details)
        print("yes")
        self.send_goal(1)



    def send_goal(self, order):
        goal_msg = "{ trajectory: { joint_names: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint],points: [{ positions: [0.5, 0.0, 0.0, 0.0], time_from_start: { sec: 1, nanosec: 0 } }]}}"

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    vision2_action_msgs = Vision2ActionMsgs()

    rclpy.spin(vision2_action_msgs)

    vision2_action_msgs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
