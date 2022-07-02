#!/usr/bin/env python2
#!/usr/bin/env python3

import sys
from multiprocessing import Manager, Queue
from multiprocessing.process import Process
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import roslib
from sensor_msgs.msg import Image
from dronekit import connect


roslib.load_manifest('roscamera')

import mission, utils


sharedQueue = None

class image_converter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw", Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # cv_image = utils.detect(cv_image, sharedQueue)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Kamera", cv_image)
        cv2.waitKey(3)


def main(vehicle):
    print("Initializing ROS Camera ...")
    
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        

if __name__ == "__main__":
    # connection_string = '/dev/serial/by-id/usb-ArduPilot_fmuv2_27003F000651373236393336-if00'
    connection_string = "127.0.0.1:14550"
    baud_rate = 115200

    print(">>>> Connecting with the UAV <<<")
    vehicle = connect(
            connection_string, 
            # baud=baud_rate, 
            wait_ready=True,
            timeout=100,
            # vehicle_class=manager.Vehicle
        )
    print(">>>> UAV is ready <<<")
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s' % vehicle.version)

    sharedQueue = Queue()
    manager = Manager()
    target_pos_x = manager.Value('f', 0)
    target_pos_y = manager.Value('f', 0)

    t1 = Process(target=main, args=(vehicle, ))
    t2 = Process(target=mission.main, args=(vehicle, (target_pos_x, target_pos_y), ))
    
    t1.start()
    t2.start()

    t1.join()
    t2.join()
