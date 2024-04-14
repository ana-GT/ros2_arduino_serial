import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Serial
import serial


##########################################
class CarCommSerial(Node):

    def __init__(self, serial_port, baud_rate):
    
        super().__init__('car_comm_serial')
        self.sub_twist_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        self.sub_twist_  # prevent unused variable warning
        
        self.pub_odom_ = self.create_publisher(Odometry, '/odom', 10)
        self.odom_ = Odometry();
        self.odom_.header.frame_id = "world"
        self.odom_.child_frame_id = "car"
        self.x_ = 0;
        self.y_ = 0;
        self.theta_ = 0;
        
        self.timer_dt_ = 1.0;
        self.timer = self.create_timer(self.timer_dt_, self.timer_callback)
        self.twist_ = Twist()
        self.twist_last_time_ = self.get_clock().now()
        self.max_dt_ = 2.0;

        # Serial
        self.serial_ = serial.Serial(serial_port, baud_rate, timeout=1)
        self.serial_.reset_input_buffer()
        self.rate_ = self.create_rate(5) # Hz

    def twist_callback(self, msg):
        self.twist_ = msg
        self.twist_last_time_ = self.get_clock().now()
        #self.get_logger().info("Twist callback, lin: {} and ang: {}".format(self.twist_.linear.x, self.twist_.angular.z))

    def timer_callback(self):
        #self.get_logger().info("Timer call")
       
        # If twist message is old, do not send anything
        if (self.get_clock().now().nanoseconds - self.twist_last_time_.nanoseconds)*1e-9 > self.max_dt_ :
          return; 
       
        # Send twist
        #self.get_logger().info("From twist msg: Lin: {} ang: {}".format(self.twist_.linear.x, self.twist_.angular.z))
        
        command = "<" + "vel_lin" + "," + str(self.twist_.linear.x) + ", " + "vel_ang" + "," + str(self.twist_.angular.z) +  ">\n"
        #self.get_logger().info("\t ** Command sent: {}".format(command))
        
        self.serial_.write(command.encode('utf-8'))
        self.rate_.sleep()
        
        # Update odometry
        vlin = self.twist_.linear.x
        vang = self.twist_.angular.z
        
        self.theta_ += vang*self.timer_dt_
        self.x_ += vlin*math.cos(self.theta_)*self.timer_dt_
        self.y_ += vlin*math.sin(self.theta_)*self.timer_dt_
        
        # Update odometry message
        self.odom_.pose.pose.position.x = self.x_ 
        self.odom_.pose.pose.position.y = self.y_
        self.odom_.pose.pose.orientation.z = math.sin(self.theta_/2.0);
        self.odom_.pose.pose.orientation.w = math.cos(self.theta_/2.0);
        self.odom_.header.stamp = self.get_clock().now().to_msg()
        self.pub_odom_.publish(self.odom_)
        
        # Check if data back is being received from car
        line = self.serial_.readline().decode('utf-8').rstrip()
        self.get_logger().info("From Arduino: {}".format(line))
        

###################################################
def main(args=None):
    rclpy.init(args=args)

    serial_port = "/dev/ttyACM0"
    baud_rate = 9600
    ccs = CarCommSerial(serial_port, baud_rate)

    rclpy.spin(ccs)

    ccs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
