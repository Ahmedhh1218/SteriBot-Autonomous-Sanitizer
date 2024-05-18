#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import cv2
from cv2 import aruco

rooms_sterilized = [1]  # Example list of sterilized rooms

class ArucoPublisher:
    def __init__(self):
        rospy.init_node('aruco_publisher', anonymous=True)
        self.marker_pub = rospy.Publisher("/aruco_marker_flag", Int32, queue_size=10)
        self.cap = cv2.VideoCapture(0)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture frame")
                break
            
            # Rotate The Screen 
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            # Convert frame to grayscale for marker detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            try:
                # Detect markers
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
                
                if ids is not None:
                    # Draw detected markers on the frame
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    
                    for marker_id in ids:
                        # Check if the detected marker ID is in the list of sterilized rooms
                        if marker_id[0] in rooms_sterilized:
                            # Publish 1 if the room is sterilized
                            self.marker_pub.publish(1)
                        else:
                            # Publish 0 if the room is not sterilized
                            self.marker_pub.publish(0)
                    
            except Exception as e:
                rospy.logerr("Error: {}".format(e))
            
            # Display the frame
            cv2.imshow('Aruco Detection', frame)
            
            # Check for key press to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rate.sleep()
        
        # Release the capture and close all OpenCV windows
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        ArucoPublisher().run()
    except rospy.ROSInterruptException:
        pass
