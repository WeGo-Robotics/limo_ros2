import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool 
from sensor_msgs.msg import LaserScan
from math import pi

class LimoEStop(Node):

    def __init__(self):
        super().__init__('limo_e_stop')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Bool, 'e_stop', 10)

        self.lidar_flag = False
        self.deg = 10
        
    def laser_callback(self, msg):
        num = 0
        if not self.lidar_flag:
            self.degrees = [
                (msg.angle_min + (i * msg.angle_increment)) * 180 / pi
                for i, data in enumerate(msg.ranges)
            ]
            self.lidar_flag = True

        for i, data in enumerate(msg.ranges):
            if (-self.deg < self.degrees[i] < self.deg and i < len(self.degrees) and 0 < msg.ranges[i] < 0.5):
                num += 1
        
        if num < len(self.degrees):
            if num < 10:
                estop = Bool()
                estop.data = False
            else:
                estop = Bool()
                estop.data = True
        else:
            print("Invalid lidar data, index out of range")
        self.publisher_.publish(estop)

def main(args=None):
    rclpy.init(args=args)

    limo_e_stop = LimoEStop()

    rclpy.spin(limo_e_stop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    limo_e_stop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()