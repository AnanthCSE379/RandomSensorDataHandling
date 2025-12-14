import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sklearn.linear_model import SGDRegressor
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
import time
import threading
import numpy as np 

class Dataprocessing(Node):
    def __init__(self):
        super().__init__('microprocessor')
        
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.parallel_group = ReentrantCallbackGroup()

        self.sensor_reading = self.create_subscription(
            Float64, 'sensor', self.distance_measured, qos_profile,
            callback_group=self.parallel_group
        )
        self.odometry_reading = self.create_subscription(
            Int32, 'odometry', self.odometry_measured, qos_profile,
            callback_group=self.parallel_group
        )

        self.measured_storage = []
        self.actual_storage = []
        self.data_lock = threading.Lock() 
        self.model = SGDRegressor(max_iter=1, warm_start=True, tol=None, penalty='l2', eta0=0.00001, random_state=42)
        self.total_dist = 0
        self.b = 0
        self.w = 1
        self.model.coef_ = np.array([float(self.w)])
        self.model.intercept_ = np.array([float(self.b)])

    def distance_measured(self, msg):
        # Only lock while appending to the list
        with self.data_lock:
            self.measured_storage.append(msg.data)
            if len(self.measured_storage) >= 10:
                temp = "Actual len : " + str(len(self.actual_storage)) + " Measured len : " + str(len(self.measured_storage))
                self.get_logger().info(temp)

        s = 'Received data, Distance measured: ' + str(msg.data)
        self.get_logger().info(s)

        s = "Predicted distance covered  : "+str(self.total_dist + self.w*msg.data + self.b)
       	self.get_logger().info(s)

    def heavy_processing_task(self, x_data, y_data):
        self.get_logger().warn(f"LOCKED & LOADED. Processing {len(x_data)} items...")
        
        print(x_data)
        print(y_data)
        X = np.array(x_data).reshape(-1, 1)
        y = np.array(y_data)

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
        
        # 4. Train
        self.model.partial_fit(X_train, y_train)
        y_pred = self.model.predict(X_test)
        
        print("Test Values:", y_test)
        print("Predictions:", y_pred)
        
        self.b = self.model.intercept_
        self.w = self.model.coef_
        
        self.get_logger().info(f"==== PROCESSED====")

    def odometry_measured(self, msg):
        batch_X = []
        batch_Y = []
        should_process = False

        with self.data_lock:
            self.total_dist += msg.data
            self.actual_storage.append(self.total_dist)
            
            current_length = len(self.actual_storage)
            
            if current_length >= 10:
                temp = "Actual len : " + str(len(self.actual_storage)) + " Measured len : " + str(len(self.measured_storage))
                self.get_logger().info(temp)
                
                batch_Y = self.actual_storage.copy()
                batch_X = self.measured_storage.copy()
                
                self.actual_storage.clear()
                self.measured_storage.clear()
                should_process = True
            else:
                should_process = False
        
        s = 'Receives data, Velocity measured: ' + str(msg.data)
        self.get_logger().info(s)
       	
        if should_process:
            self.heavy_processing_task(batch_X, batch_Y)

def main(args=None):
    try:
        rclpy.init(args=args)
        processor = Dataprocessing()
        executor = MultiThreadedExecutor()
        executor.add_node(processor)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            processor.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
