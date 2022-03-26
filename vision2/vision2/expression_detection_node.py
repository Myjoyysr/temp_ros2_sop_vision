import rclpy
from rclpy.node import Node
import cv2
import numpy
import os


from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from sensor_msgs.msg import Image
import message_filters

from vision2_msgs.msg import Faces, Face, Point2, FaceImage, FaceImages
from keras.models import load_model
from keras.preprocessing.image import img_to_array

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Need to fix everything here after vision2_msgs are working

class Vision(Node):

    def __init__(self):
        super().__init__('vision')

        classifier = (
            self.declare_parameter("classifier", "fer_2013.h5")
            .get_parameter_value()
            .string_value
        )

        self.classifier = load_model(
            os.path.join(
                get_package_share_directory("vision"),
                "classifier",
                classifier,
            )
        )

        # for recieving face img array + coord array
        self.faces_sub = message_filters.Subscriber(
            self,
            FaceImages,
            "/faces", #when not using namespaces
        )
        # for recieving face coords with tracker ids
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            "/image_raw",
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            (self.faces_sub, self.image_sub), 4, 0.1, allow_headerless=True)

        self.sync.registerCallback(self.detect_expression)

    def detect_expression(self, msg_ids, msg_face_imgs):
        try:
            #todo: handle msgs, detect expressions, send coords, expression, id array to next step (ai and vizualisation_node)





            cv2_bgr_img = bridge.imgmsg_to_cv2(img, "bgr8")
            cv2_gray_img = bridge.imgmsg_to_cv2(img, "mono8")


            faces_array = []
            i = 0

            for x, y, w, h in face:

                new_face = cv2_bgr_img[y:y + h, x:x + w] 
                new_face2 = cv2_gray_img[y:y + h, x:x + w]
                faces_array.append(new_face)

                cv2.rectangle(cv2_bgr_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                temp = cv2.resize(new_face2, (48, 48))

                pixels = img_to_array(temp)
                pixels = numpy.expand_dims(pixels, axis=0)

                predictions = self.classifier.predict(pixels)

                index = numpy.argmax(predictions[0])
                print(predictions)
                emotions = ("Angry", "Disgust", "Fear", "Happy",
                            "Neutral", "Sad", "Surprised")

                predicted_emotion = emotions[index]

                cv2.putText(faces_array[i], predicted_emotion, (int(20), int(
                    20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                print(predicted_emotion)
                faces_array[i] = faces_array[i]
                i = i+1

            #blank_image = numpy.zeros((480,640,3), numpy.uint8)
            dim = (96, 128)
            i = 1
            x_offset = 0
            y_offset = 0
            for face in faces_array:
                fixed = cv2.resize(face, dim)

                cv2_bgr_img[y_offset:y_offset+fixed.shape[0],
                            x_offset:x_offset+fixed.shape[1]] = fixed
                # print(i)
                i = i+1
                x_offset = x_offset + 96
                if x_offset > 512:
                    y_offset = y_offset+128
                    x_offset = 0

            self.face_img_publisher.publish(
                bridge.cv2_to_imgmsg(cv2_bgr_img, "bgr8"))
            #self.face_img_publisher.publish(bridge.cv2_to_imgmsg(blank_image, "bgr8"))

        except:
            print("error")


def main(args=None):
    rclpy.init(args=args)

    visions = Vision()

    rclpy.spin(visions)

    visions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
