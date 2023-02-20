#!/usr/bin/python
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import String

def subscriber():
        sub=rospy.Subscriber('light_publish',String,callback_function)
        rospy.spin()

def callback_function(message):
        d=message.data
        print (d)

if __name__ == "__main__":
    rospy.init_node("light_subscriber")
    subscriber()
