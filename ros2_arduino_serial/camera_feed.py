import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading

import numpy as np
import cv2 as cv

##########################################
# Create a node
# Every t seconds, it takes a picture and stores it in 
# a given location
mutex = threading.Lock()

class CameraFeed(Node):

    def __init__(self):
    
        super().__init__('camera_feed')

        self.declare_parameter('image_folder', "/home")
        self.declare_parameter('image_name', "test_image.png")
        self.declare_parameter('timer_dt', 1.0)
        self.declare_parameter('timer_capture', 0.03)
        
        param_image_folder = self.get_parameter('image_folder')
        param_image_name = self.get_parameter('image_name')
        param_timer_dt = self.get_parameter('timer_dt')
        param_timer_capture = self.get_parameter('timer_capture')
                
        self.image_filename_ = str(param_image_folder.value) + str("/") + str(param_image_name.value)
        
        self.timer_write_ = self.create_timer(param_timer_dt.value, self.write_callback)
        self.timer_capture_ = self.create_timer(param_timer_capture.value, self.capture_callback)
        self.initialized_ = True
        #self.frame_ = cv.Image()
        
        self.cap_ = cv.VideoCapture(-1)
        if not self.cap_.isOpened():
          self.get_logger().warn("Cannot open camera")
          self.initialized_ = False

    def initialized(self):
        return self.initialized_

    def write_callback(self):
        mutex.acquire()
        #gray = cv.cvtColor(self.frame_, cv.COLOR_BGR2GRAY)
        img_scaled = cv.resize(self.frame_, (96, 72))
        mutex.release()
        if not cv.imwrite(self.image_filename_, img_scaled):
          self.get_logger().warn("Error saving image!")

    def capture_callback(self):
        mutex.acquire()
        ret, self.frame_ = self.cap_.read()
        mutex.release()
        #if not ret:
        #  print("Can't receive frame. Exiting...")
        #  return
       


    # When everything is done, release the capture
    def destroyCap(self):              
        self.cap_.release()
        cv.destroyAllWindows() # No needed here, no GUI used


###################################################
def main(args=None):
    rclpy.init(args=args)

    cfd = CameraFeed()

    if not cfd.initialized():
      print("Error while initializing camera feed")
      rclpy.shutdown()
      return
      
    rclpy.spin(cfd)

    cfd.destroyCap()
    cfd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
