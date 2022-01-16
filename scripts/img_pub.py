import os
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys

def operator():

    filepath = sys.argv[1]

    rospy.init_node('image_publisher', anonymous=True)
    pub = rospy.Publisher('image', Image, queue_size=10)
    # read image
    #filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lena.png')
    im = cv2.imread(filepath, cv2.IMREAD_COLOR)
    # make bridge
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(im, encoding="bgr8")
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        operator()
    except rospy.ROSInterruptException:
        pass

