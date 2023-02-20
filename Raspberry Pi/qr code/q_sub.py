#!/usr/bin/env python

#the line above this was to show that it is an executable, and where to find python
#the line below this is to use python library
import rospy
import serial
import subprocess
# import String from /opt/ros/noetic/share/std_msgs/msg
#you can use nano to open it like in unix to see what it composes of
from std_msgs.msg import String

def subscriber():
    #this saying making subscriber that suscribes to topic string_publish
    #callback_function is the function to be called whenever topic receives a message
    sub=rospy.Subscriber('string_publish',String,callback_function)
    #will continue while it lasts
    rospy.spin()

def callback_function(message):
    #will print this when function called
    if message != "":
       rate=rospy.Rate(1)
       ser.write(b"DONE\n")
       line = ser.readline().decode('utf-8').rstrip()
       print(line)
       print(message)
       subprocess.run(["rosrun","map_server","map_saver","-f","new_map"])
       subprocess.run(["convert","new_map.pgm","new_map.png"])
       #subprocess.run(["eog","new_map.png"])
       rate.sleep()

#what will happen when you call this script
if __name__=="__main__":
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    rospy.init_node("qr_subscriber")
    subscriber()
