import rclpy
from rclpy.node import Node 
from std_msgs.msg import Float64,Int32
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import random

class RandomPublisher(Node):
	def __init__(self):
		super().__init__('sens')
		qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
		self.sensor_publisher = self.create_publisher(Float64,'sensor',qos_profile)
		self.odometry_publisher = self.create_publisher(Int32,'odometry',qos_profile)
		timer_period = 1
		self.timer = self.create_timer(timer_period,self.timer_callback)
		self.pos = 0
		self.temp = 0
	def timer_callback(self):
		calculated_distance = Float64()
		odometry_reading = Int32()
		self.temp = random.randint(0,20)
		odometry_reading.data = self.temp
		self.pos += self.temp
		calculated_distance.data = float(self.pos)
		calculated_distance.data += random.uniform(-1.5,1.5)
		self.sensor_publisher.publish(calculated_distance)
		self.odometry_publisher.publish(odometry_reading)
		self.get_logger().info("Total Distance measured : %f"%calculated_distance.data)
		self.get_logger().info("Distance Moved : %f"%odometry_reading.data)


def main(args = None):
	try:
		rclpy.init(args=args)
		random_pub = RandomPublisher()
		rclpy.spin(random_pub)
		random_pub.destroy_node()
		rclpy.shutdown()
	except KeyboardInterrupt:
		pass
	finally:
		pass

if __name__ == '__main__':
	main()
