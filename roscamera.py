#!/usr/bin/env python2
#!/usr/bin/env python3

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import roslib
from sensor_msgs.msg import Image

roslib.load_manifest('roscamera')


class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = detected(cv_image)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Kamera", cv_image)
        cv2.waitKey(3)


def main(args):
    print("Initializing ROS Camera ...")
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    
    main(args=(sys.argv))