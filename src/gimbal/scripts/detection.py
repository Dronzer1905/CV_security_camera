#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import cv2
import time
import numpy as np

def face_publisher():
    rospy.init_node('face_position_publisher', anonymous=True)
    pub_face = rospy.Publisher('/face_position', String, queue_size=10)
    pub_video = rospy.Publisher('/video_frames', CompressedImage, queue_size=10)
    rate = rospy.Rate(10)  

    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)

    time.sleep(1)  

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame")
            continue

        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.1, 6)  

        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            face_str = '{0:d} {1:d}'.format((x + w // 2), (y + h // 2))
            pub_face.publish(face_str)
            rospy.loginfo("Published: %s", face_str)
            cv2.circle(frame, (x + w // 2, y + h // 2), 2, (0, 255, 0), 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
        
        cv2.rectangle(frame, (640 // 2 - 60, 480 // 2 - 60),
                      (640 // 2 + 60, 480 // 2 + 60), (255, 255, 255), 3)
        
        _, buffer = cv2.imencode('.jpg', frame)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()
        pub_video.publish(msg)
        
        cv2.imshow('Face Detection', frame)
        
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        
        rate.sleep()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        face_publisher()
    except rospy.ROSInterruptException:
        pass
