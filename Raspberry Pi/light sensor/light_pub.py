#!/usr/bin/env python
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String

GPIO.setmode(GPIO.BOARD)
GPIO.setup(8, GPIO.IN)

# Main program loop
if __name__ == "__main__":
        rospy.init_node("light_publisher")
        pub=rospy.Publisher('light_publish',String,queue_size=10)
        rate=rospy.Rate(10)
        msg_to_publish=String()

        while True:
                if GPIO.input(8) == 1:
                        d="off"
                        msg_to_publish.data=d
                        pub.publish(msg_to_publish)
                else:
                        d="on"
                        msg_to_publish.data=d
                        pub.publish(msg_to_publish)

                rate.sleep()
