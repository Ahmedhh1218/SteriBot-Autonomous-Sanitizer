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
                    # Filter markers based on aspect ratio
                    valid_ids = []
                    for i, marker_id in enumerate(ids):
                        # Calculate aspect ratio of the marker's bounding box
                        aspect_ratio = self.calculate_aspect_ratio(corners[i])
                        
                        # Check if the aspect ratio indicates the marker is roughly perpendicular
                        if self.is_perpendicular(aspect_ratio):
                            valid_ids.append(marker_id)
                    
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    # Process only the valid markers
                    for marker_id in valid_ids:
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
    def calculate_aspect_ratio(self, corners):
        # Calculate aspect ratio of the marker's bounding box
        x, y, w, h = cv2.boundingRect(corners)
        return float(w) / h
        
    def is_perpendicular(self, aspect_ratio):
        # Check if the aspect ratio indicates the marker is roughly perpendicular
        perpendicular_threshold = 0.9  # Adjust this threshold as needed
        print(aspect_ratio >= perpendicular_threshold)
        return aspect_ratio >= perpendicular_threshold

if __name__ == "__main__":
    try:
        ArucoPublisher().run()
    except rospy.ROSInterruptException:
        pass
    

