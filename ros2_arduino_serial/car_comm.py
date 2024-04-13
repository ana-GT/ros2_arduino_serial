import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Serial
import serial


##########################################
class CarCommSerial(Node):

    def __init__(self, serial_port, baud_rate):
    
        super().__init__('car_comm_serial')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1, self.timer_callback)
        self.twist_ = Twist()

        # Serial
        self.serial_ = serial.Serial(serial_port, baud_rate, timeout=1)
        self.serial_.reset_input_buffer()
        self.rate_ = self.create_rate(5) # Hz

    def twist_callback(self, msg):
        self.twist_ = msg

    def timer_callback(self):
        self.get_logger().info("Timer call")
       
        # Send twist
        self.get_logger().info("Received twist: Lin: {} ang: {}".format(self.twist_.linear.x, self.twist_.angular.z))
        
        command = "<" + "vel_lin" + "," + str(self.twist_.linear.x) + ", " + "vel_ang" + "," + str(self.twist_.angular.z) +  ">\n"
        self.get_logger().info("** Command sent: {}".format(command))
        
        self.serial_.write(command.encode('utf-8'))
        self.rate_.sleep()
        
        # Check if data back is being received from car
        line = self.serial_.readline().decode('utf-8').rstrip()
        self.get_logger().info("Received from Arduino: {}".format(line))
        

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
