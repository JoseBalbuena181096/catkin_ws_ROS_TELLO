#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from  std_msgs.msg import Int8
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FollowFaceTello:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.Ovr = rospy.Subscriber('/keyboard/override', Int8, self.callBackFlag)
        self.detMsg = rospy.Subscriber('/face_detection/center', 
                                        Int32MultiArray,
                                        self.detCallBack,
                                        queue_size=100)
        self.image_sub = rospy.Subscriber('/tello/image_raw',
                                          Image,
                                          self.callback,
                                          queue_size=1, 
                                          buff_size=2**24)
        self.vel_msg = Twist()
        self.speed_values = 0.3
        self.run = 0
        self.image_h = int(rospy.get_param("Camera.height"))
        self.image_w = int(rospy.get_param("Camera.width"))
        self.image_reduction = rospy.get_param("/followFaceTello/imageReduction")
        self.image_h = int(self.image_h * (self.image_reduction/100))
        self.image_w = int(self.image_w * (self.image_reduction/100))
        self.height_by_2 = int(self.image_h/2)
        self.width_by_2 = int(self.image_h/2)
        self.centerXY = [self.width_by_2, self.height_by_2]

    def callBackFlag(self, msg):
        rospy.loginfo(f'Override: {msg.data}')
        if msg.data == 5:
            self.run = 1
            rospy.loginfo("Autonomus Mode ON")
        else:
            self.run = 0
            rospy.loginfo("Autonomus Mode OFF")

    def detCallBack(self, msg):
        self.centerXY = msg.data

    def control_flight(self):
        cx = self.centerXY[0]
        cy = self.centerXY[1]
        error_x = self.width_by_2 - cx
        error_y = self.height_by_2 - cy
        rospy.loginfo(f'error x: {error_x}, error_y: {error_y}')
        if self.run == 1:
            if abs(error_x) < 10 and abs(error_y) < 10:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)
            else:
                signal_angle_z = 0.45 * (error_x) / self.width_by_2
                rospy.loginfo(f'signal control angular axis z {signal_angle_z}')
                signal_linear_z = 0.3 * (error_y) / self.height_by_2
                rospy.loginfo(f'signal control linear axis z {signal_linear_z}')

                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = round(signal_linear_z, 2)
                self.vel_msg.angular.z = round(signal_angle_z, 2)
                self.pub_cmd_vel.publish(self.vel_msg)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.control_flight()

def main():
    rospy.init_node('followFaceTello', anonymous=True)
    followFaceTello = FollowFaceTello()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
