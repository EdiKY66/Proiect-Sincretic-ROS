#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist

class BallFollower:
    def __init__(self):
        rospy.init_node('ball_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.detecting_ball = True  # Starea de detectare a mingii
        self.threshold_area = 1000  # Ajustează acest prag în funcție de nevoi

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range for all colors in HSV
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([179, 255, 255])

        # Threshold the HSV image to get all colors
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if self.detecting_ball:
            # Rotește robotul dacă nu s-a detectat nicio minge
            if not contours:
                self.twist.angular.z = 0.5  # Rotație la dreapta
                self.cmd_vel_pub.publish(self.twist)
            else:
                # Filtru pentru conturile mici
                valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.threshold_area]

                if valid_contours:
                    # Obține cel mai mare contur
                    largest_contour = max(valid_contours, key=cv2.contourArea)

                    # Calculează centroidul
                    M = cv2.moments(largest_contour)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Oprește rotația și se deplasează înainte către minge
                    self.twist.angular.z = 0
                    self.twist.linear.x = 0.2
                    self.cmd_vel_pub.publish(self.twist)
                    self.detecting_ball = False  # Schimbă starea la detectarea mingei

        else:
            # Lovește mingea
            self.twist.linear.x = 0.5  # Se deplasează înainte
            self.cmd_vel_pub.publish(self.twist)
            rospy.sleep(1)  # Așteaptă un moment pentru a lovi mingea
            self.stop_robot()  # Oprește robotul după 1 secundă
            self.detecting_ball = True  # Reia detectarea mingii

    def stop_robot(self):
        # Oprește complet robotul
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        rospy.sleep(0.1)  # Așteaptă 0.1 secunde după impact

if __name__ == '__main__':
    try:
        ball_follower = BallFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

