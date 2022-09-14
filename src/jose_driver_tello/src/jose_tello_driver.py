#!/usr/bin/env python3

import threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import av
import cv2
import numpy
from tellopy._internal.tello import Tello
from cv_bridge import CvBridge


class TelloDriver(object):

    def __init__(self):
        # Initialize ROS
        rospy.init_node('jose_driver_node', anonymous=False)

        # ROS publishers

        self._image_pub = rospy.Publisher('/tello/image_raw', Image, queue_size=10)

        # ROS subscriptions
        rospy.Subscriber('/tello/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/tello/takeoff', Empty, self.takeoff_callback)
        rospy.Subscriber('/tello/land', Empty, self.land_callback)


        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

        # Connect to the drone
        self._drone = Tello()
        self._drone.connect()
        self._drone.wait_for_connection(60.0)
        rospy.loginfo('connected to drone')


        # Start video thread
        self._stop_request = threading.Event()
        video_thread = threading.Thread(target=self.video_worker)
        video_thread.start()

        # Spin until interrupted
        rospy.spin()

        # Force a landing
        self._drone.land()

        # Stop the video thread
        self._stop_request.set()
        video_thread.join(timeout=2)

        # Shut down the drone
        self._drone.quit()
        self._drone = None


    def cmd_vel_callback(self, msg):
        self._drone.set_pitch(msg.linear.x)
        self._drone.set_roll(-msg.linear.y)     # Note sign flip
        self._drone.set_yaw(-msg.angular.z)     # Note sign flip
        self._drone.set_vspeed(msg.linear.z)

    def takeoff_callback(self, msg):
        self._drone.takeoff()

    def land_callback(self, msg):
        self._drone.land()


    def video_worker(self):
        # Get video stream, open in PyAV
        container = av.open(self._drone.get_video_stream())

        # Decode h264
        rospy.loginfo('starting video pipeline')
        for frame in container.decode(video=0):

            # Convert PyAV frame => PIL image => OpenCV Mat
            color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            color_mat = cv2.resize(color_mat,(360,240))
            # Convert OpenCV Mat => ROS Image message and publish
            self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

            # Check for normal shutdown
            if self._stop_request.isSet():
                return


if __name__ == '__main__':
    driver = TelloDriver()