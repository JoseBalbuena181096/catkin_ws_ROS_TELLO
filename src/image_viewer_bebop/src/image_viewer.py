#!/usr/bin/env python3
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 

class Image_converter:
    def __init__(self):
        self.bridge = CvBridge()    
        self.image_sub = rospy.Subscriber("/bebop/image_raw",
                                          Image,
                                          self.callback,
                                          queue_size=1,
                                          buff_size=2**24)

    def resize_image(self, image, scale_percent):
        width = int(image.shape[1]*(scale_percent/100))
        height = int(image.shape[0]*(scale_percent/100))
        dim = (width, height)
        resize = cv2.resize(image, dim, interpolation= cv2.INTER_AREA)
        return resize

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")                         
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        #cv_image = self.resize_image(cv_image, 60)
        print(f'rows: {rows}, cols: {cols}, channels: {channels}')
        cv2.imshow("Image window bebop", cv_image)
        cv2.waitKey(3)

def main(args):
    ic = Image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
