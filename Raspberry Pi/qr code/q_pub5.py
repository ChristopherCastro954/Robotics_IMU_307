#!/usr/bin/env python
import cv2
import csv
import numpy as np
from pyzbar import pyzbar
import rospy
import subprocess


#adding time and date stuff and rearranging>from datetime import date, datetime
from datetime import date, datetime
from std_msgs.msg import String

rospy.init_node("qr_publisher")
pub=rospy.Publisher('string_publish',String,queue_size=10)
msg_to_publish=String()

today = date.today()
date = today.strftime("%d-%b-%Y")

now = datetime.now()
timeRN = now.strftime("%H:%M:%S")



# Initialize the camera
camera = cv2.VideoCapture(0)
count = 0
breakout=0

# Loop through frames from the camera
while True:
    # Read a frame from the camera
    ret, frame = camera.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1 is 32" bottom
    # 2 is 4' bottom
    # optimal 9" from wall 8-9" operational 
    #sets the coordinates
    tl=(150,210) #(190,220)2 #(190,280)1
    bl=(120,420) #(190,320)2 #(180,345)1
    tr=(385,200) #(355,210)2 #(320,260)1
    br=(480,385) #(380,305)2 #(345,325)1

    #this makes the circle, not necessary
    cv2.circle(frame,tl,5,(0,0,255),-1)
    cv2.circle(frame,bl,5,(0,0,255),-1)
    cv2.circle(frame,tr,5,(0,0,255),-1)
    cv2.circle(frame,br,5,(0,0,255),-1)

    pts1=np.float32([tl,bl,tr,br])
    pts2=np.float32([[0,0],[0,480],[640,0],[640,480]])

    matrix=cv2.getPerspectiveTransform(pts1,pts2)
    transformed_frame=cv2.warpPerspective(gray,matrix,(640,480))

    # Find the QR code in the frame
    qr_codes = pyzbar.decode(transformed_frame)
    # Loop through each QR code found
    for qr_code in qr_codes:
        # Draw a rectangle around the QR code
        x, y, w, h = qr_code.rect
        cv2.rectangle(transformed_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Print the data encoded by the QR code
        print(qr_code.data.decode('utf-8'),end="")
        #print(count)
        #count=count+1
        msg_to_publish.data=qr_code.data.decode('utf-8')
        pub.publish(msg_to_publish)
        with open('Database.csv', mode='a') as csvfile:

                csvfileWriter = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
                csvfileWriter.writerow([qr_code.data, date, timeRN])
        rate=rospy.Rate(1)
        rate.sleep()
        #break
        breakout=1

    # Display the frame
    cv2.imshow('QR Code Scanner', frame)
    cv2.imshow('zoomed', transformed_frame)

    if breakout==1:
       break
    # Check for the 'q' key to quit the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
       break

# Release the camera and close the window
camera.release()
cv2.destroyAllWindows()
