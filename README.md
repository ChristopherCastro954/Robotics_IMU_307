# Robotics-IMU-307-
Robotics &amp; IMU Senior Design Project Team 307 (2022-2023)

Objective:
The objective of this project is to create an autonomous mobile platform that can navigate an office environment while avoiding obstacles and people, to detect and scan a QR code at an unknown location.

Abstract:

Automation in robotics has been a growing topic for years. With its use in warehouses and manufacturing, autonomous robotics help drive the backbone of businesses. These same concepts found in industrial services can be trickled down into helping the day-to-day work of an office. We wanted to build an autonomous robot that could function in an office of people and obstacles that serves the purpose of finding a QR code along the wall. 

For our project, we created a self-driving robot that navigates an office space without hitting people or objects. While the robot moved, it was not allowed to accept any GPS input. The reason for this is to imitate navigating spaces where GPS is not available for use. The robot constantly scans while it navigates until it finds the QR code on a wall. QR codes have different sizes according to what information they store. In this project, we used a smaller size that can store messages between coworkers. The type of algorithm we used to navigate in the room was to stick close to the walls. If it detects an obstacle, it will avoid it and then go back to sticking close to the wall. As the robot moves, it constantly scans to find the QR code. The robot is small and lightweight, and its speed is limited to a human walking pace. These physical limitations are set to ensure it is safe to use in an office environment, allowing it to avoid harming people and/or damaging objects. Once the robot scans the QR code the project is deemed successful, and the robot would stop operating.

![Alt text](robot1.jpg?raw=true "Title")
