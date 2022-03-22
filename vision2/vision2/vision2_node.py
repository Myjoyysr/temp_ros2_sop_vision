import rclpy
from rclpy.node import Node
import cv2

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image
#from vision2_msgs.msg import Faces, Face, Point2
from face_tracker_msgs.msg import Faces, Face, Point2

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
face_cascade_name = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')


class Vision2(Node):

    def __init__(self):
        super().__init__('vision')


        self.subscriber = self.create_subscription(
            Image,
            "/image_raw",
            self.detect_face,
            10,
        )
        
        faces_topic = (
            self.declare_parameter(
                "face_image_topic", "faces"
            )
            .get_parameter_value()
            .string_value
        )
        
        self.face_publisher = self.create_publisher(
            Faces, faces_topic, 10)
        

        #TODOOO
        #face coordinate topic, 38x38 pixel image array topic

    def detect_face(self, img: Image):
        try:
            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")

            face = face_cascade_name.detectMultiScale(
                cv2_gray_img, scaleFactor=1.2, minNeighbors=7, minSize=(35, 35))

            faces_array = []
            
            msg_faces = []
            for x, y, w, h in face:

                new_face = cv2_bgr_img[y:y + h, x:x + w] 
                new_face2 = cv2_gray_img[y:y + h, x:x + w]
                faces_array.append(new_face)

                cv2.rectangle(cv2_bgr_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                for_classifier = cv2.resize(new_face2, (48, 48))
                faces_array.append(for_classifier)
                msg_face = Face(top_left=Point2(x, y), bottom_right=Point2((x+w), (y+h)))
                msg_faces.append(msg_face)

            #blank_image = numpy.zeros((480,640,3), numpy.uint8)

            #self.face_img_publisher.publish( )
            self.face_publisher.publish(Faces(faces=msg_faces))


        except:
            print("error")


def main(args=None):
    rclpy.init(args=args)

    visions = Vision2()

    rclpy.spin(visions)

    visions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
