import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from dronekit import connect, Command, LocationGlobal, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time, sys, argparse, math
import cv2, imutils
import threading

rospy.init_node('image_subscriber', anonymous=True)

bridge = CvBridge()

image_data = None
distance_x = 0
distance_y = 0

def image_callback(msg):
    global image_data
    image_data = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    process_image()

rospy.Subscriber("/webcam/image_raw", Image, image_callback)

def process_image():
    global distance_x
    global distance_y
    global image_data
    if image_data is not None:
        hsv = cv2.cvtColor(image_data, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            
            moments = cv2.moments(max_contour)
            if moments["m00"] != 0:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                
                cv2.drawContours(image_data, [max_contour], -1, (0, 255, 0), 2)
                cv2.line(image_data, (x, y), (image_data.shape[1] // 2, image_data.shape[0] // 2), (255, 0, 0), 2)
                
                distance_x = image_data.shape[1] // 2 - x
                distance_y = image_data.shape[0] // 2 - y
                
                cv2.putText(image_data, f'x distance: {distance_x}, y distance: {distance_y}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        center_x = image_data.shape[1] // 2
        center_y = image_data.shape[0] // 2
        cv2.circle(image_data, (center_x, center_y), 50, (0, 255, 0), 2)

        cv2.imshow("Processed Image", image_data)
        cv2.waitKey(1)

def hız(posx, posy, yaw_rate, posz, vehicle):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,
        0, 0, 0,  # pozisyonlar(metre)
        posx, posy, posz,  # hizlar(metre/s)
        0, 0, 0,  # akselarasyon(fonksiyonsuz)
        0, math.radians(yaw_rate))  # yaw,yaw_rate(rad,rad/s)

    vehicle.send_mavlink(msg)

def find_it():
    tolerance = 7
    while True:
        print(f'distance_x: {distance_x}, distance_y: {distance_y}, altitude: {vehicle.location.global_relative_frame.alt}')
        if abs(distance_x) < tolerance and abs(distance_y) < tolerance:
            print("in the circle")
        if vehicle.location.global_relative_frame.alt >= 20:
            x_value = distance_x / 100.0
            y_value = distance_y / 100.0
            hız(y_value, -x_value, 0, 3, vehicle)
            time.sleep(3)
        
        elif vehicle.location.global_relative_frame.alt >= 10:
            x_value = distance_x / 100.0
            y_value = distance_y / 100.0
            hız(y_value, -x_value, 0, 1, vehicle)
            time.sleep(3)

        elif vehicle.location.global_relative_frame.alt > 4:
            x_value = distance_x / 100.0
            y_value = distance_y / 100.0
            hız(y_value, -x_value, 0, 0.3, vehicle)
            time.sleep(3)

        elif vehicle.location.global_relative_frame.alt > 1.5:
            x_value = distance_x / 100.0
            y_value = distance_y / 100.0
            hız(0, 0, 0, 0.1, vehicle)
            time.sleep(3)

    print("landed")
    




baglanti_yolu = "127.0.0.1:14550"

global drone
drone = connect(baglanti_yolu, wait_ready=True)
global vehicle
vehicle = drone

find_it()

rospy.spin()

cv2.destroyAllWindows()
