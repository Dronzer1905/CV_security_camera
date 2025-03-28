#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial

xpos, ypos = 90, 90  # Servo positions

try:
    arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  
    rospy.loginfo("Connected to Arduino on /dev/ttyACM0")
except serial.SerialException as e:
    rospy.logerr(f"Failed to connect to Arduino: {e}")
    arduino = None  # Handle the case where Arduino is not connected

def face_subscriber():
    global xpos, ypos

    rospy.init_node('face_position_subscriber', anonymous=True)
    
    width, height = 640, 480
    angle = 1

    while not rospy.is_shutdown():
        try:
            # Wait for a new face position message
            msg = rospy.wait_for_message('/face_position', String, timeout=2.0)
            face_position = msg.data
            rospy.loginfo(f"Received face position: {face_position}")

            # Parse face position
            coordinates = face_position.split(" ")
            x_mid = int(coordinates[0])  
            y_mid = int(coordinates[1])  

            # Adjust servo positions
            if x_mid > (width / 2 + 60):
                xpos = min(xpos + angle, 180)  
            elif x_mid < (width / 2 - 60):
                xpos = max(xpos - angle, 0)    

            if y_mid < (height / 2 - 60):
                ypos = min(ypos + angle, 180)
            elif y_mid > (height / 2 + 60):
                ypos = max(ypos - angle, 0)

            pos_str = f"X{xpos} Y{ypos}\n"

            # Send to Arduino
            if arduino and arduino.is_open:
                arduino.write(pos_str.encode())  
                rospy.loginfo(f"Sent to Arduino: {pos_str.strip()}")
            else:
                rospy.logwarn("Serial connection not available")

        except rospy.ROSException:
            rospy.logwarn("No message received. Retrying...")

        except (ValueError, IndexError) as e:
            rospy.logerr(f"Error parsing coordinates: {e}")

def shutdown_hook():
    if arduino and arduino.is_open:
        arduino.close()
        rospy.loginfo("Closed Arduino connection.")

if __name__ == '__main__':
    rospy.on_shutdown(shutdown_hook)
    try:
        face_subscriber()
    except rospy.ROSInterruptException:
        pass
