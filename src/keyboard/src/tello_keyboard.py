#!/usr/bin/env python3

import rospy
import sys
from getkey import getkey, keys
from geometry_msgs.msg import  Twist 
from std_msgs.msg import Int8
from std_msgs.msg import Empty


class Keyboard:
    def __init__(self):
        self.pub_takeoff = rospy.Publisher('/tello/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=10)
        self.pub_override = rospy.Publisher('/keyboard/override', Int8, queue_size=10)
        self.vel_msg = Twist()
        self.ovr_msg = 0
        self.speed_value = 0.3
        self.init_msg = """
        ###########################################################
                KEYBOARD CONTROL DRON TELLO WITH ROS
        
        LAND            -SPACE BAR
        TAKEOFF         -T
        FORWARD         -W
        BACKWARD        -S
        RIGHT           -D
        LEFT            -A
        ROTATE(RIGHT)   -E
        ROTATE(LEFT)    -Q
        ASCEND          -UP ARROW
        DESCEND         -DOWN ARROW

        AUTONOMUS       -X
        MANUAL CONTROL  -C      

        ###########################################################
        """

        self.final_msg = """
        CTRL + C TO QUIT
        """
        print(self.init_msg)
        print(f"             SPEED DRON: {self.speed_value}")
        print(self.final_msg)
        self.cmd_vel()

    def cmd_vel(self):
        while not rospy.is_shutdown():
            key = getkey()
            
            if key == keys.T:
                msg = Empty()
                self.pub_takeoff.publish(msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: TAKEOFF")
                print(self.final_msg)

            elif key == keys.SPACE:
                msg = Empty()
                self.pub_land.publish(msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: LAND")
                print(self.final_msg)
            
            elif key == keys.W:
                self.vel_msg.linear.x = round(self.speed_value, 2)
                self.vel_msg.linear.y = 0.0 
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: FORWARD")
                print(self.final_msg)

            elif key == keys.S:
                self.vel_msg.linear.x = -round(self.speed_value, 2)
                self.vel_msg.linear.y = 0.0 
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: BACKWARD")
                print(self.final_msg)
            
            elif key == keys.A:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = -round(self.speed_value, 2) 
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: LEFT")
                print(self.final_msg)

            elif key == keys.D:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = round(self.speed_value, 2) 
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: RIGHT")
                print(self.final_msg)
            
            elif key == keys.Q:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = round(self.speed_value, 2) 
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: ROTATE(LEFT)")
                print(self.final_msg)

            elif key == keys.E:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = 0.0
                self.vel_msg.angular.z = -round(self.speed_value, 2) 
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: ROTATE(RIGHT)")
                print(self.final_msg)
                   
            elif key == keys.UP:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = round(self.speed_value, 2)
                self.vel_msg.angular.z = 0.0 
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: ASCEND")
                print(self.final_msg)
            
            elif key == keys.DOWN:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_msg.linear.z = -round(self.speed_value, 2)
                self.vel_msg.angular.z = 0.0 
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: DESCEND")
                print(self.final_msg)
            
            elif key == keys.X:
                self.ovr_msg = 5
                self.pub_override.publish(self.ovr_msg)
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: AUTONOMUS")
                print(self.final_msg)
            
            elif key == keys.C:
                self.ovr_msg = 10
                self.pub_override.publish(self.ovr_msg)
                self.pub_cmd_vel.publish(self.vel_msg)
                print(self.init_msg)
                print(f"             SPEED DRON: {round(self.speed_value, 2)}")
                print("              LAST COMMAND SEND: CANCEL AUTONOMUS")
                print(self.final_msg)

def main(): 
    rospy.init_node('keyboard', anonymous=True)
    keyboard = Keyboard()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()