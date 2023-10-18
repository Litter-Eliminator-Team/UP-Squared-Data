import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import dill
import sklearn
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

class usbcamSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_image_subber")
        self.bridge = CvBridge()
        self.intel_subber = self.create_subscription(
            Image, 
            "/camera/color/image_raw",
            self.intel_callback,
            10
        )
        self.intel_pubber = self.create_publisher(Image, '/out/image', 3)
        self.motor_go = self.create_publisher(Int32, '/out/motor_go', 3)
        self.dill_filename = "/home/literbotpro/ros2_ws/src/perception_navigation/perception_navigation/SVM_classify2.pkl"
    def intel_callback(self, inp_im):
        try: 
            imCV = self.bridge.imgmsg_to_cv2(inp_im, "bgr8")
        except CvBridgeError as e:
            print(e)

        if imCV is None:
            print('frame dropped, skipping tracking')
        else:
            self.ImageProcessor(imCV)

    def ImageProcessor(self, imCV):

        go = Int32()

        newFrame = cv2.resize(imCV, dsize = (300, 300), interpolation=cv2.INTER_CUBIC)
        framePixels = np.array(newFrame)
        preprocessedFrame = framePixels.ravel()[np.newaxis,:]
        preprocessedFrame = preprocessedFrame/255
        test = self.svm_model.predict(preprocessedFrame)

        if test == 2:
            go.data = 1
        else:
            go.data = 0

        # faceCascade = cv2.CascadeClassifier('/home/literbotpro/ros2_ws/src/perception_navigation/perception_navigation/haarcascade_frontalface_default.xml')

        # gray = cv2.cvtColor(imCV, cv2.COLOR_BGR2GRAY)

        # faces = faceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30,30), flags = cv2.CASCADE_SCALE_IMAGE)

        # for (x, y, w, h) in faces:
        #     cv2.rectangle(imCV, (x,y), (x+w, y+h), (0,255,0), 2)
        #     if w*h > 0:
        #         go.data = 1
        #     else:
        #         go.data = 0

        imCV = self.bridge.cv2_to_imgmsg(imCV, "bgr8")

        self.intel_pubber.publish(imCV)
        self.motor_go.publish(go)

    def loadML(self):
        with open(self.dill_filename, 'rb') as file:
            self.svm_model = dill.load(file)
    

def main(args=None):
    rclpy.init(args=args)
    node = usbcamSubscriberNode()
    node.loadML()
    rclpy.spin(node)
    rclpy.shutdown()