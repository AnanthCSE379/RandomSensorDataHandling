# RandomSensorDataHandling
This is a ROS2 project which will have one node simulate random sensor data from a distance measuring sensor such as ultrasonic sensor and another node which will try to determine the actual value of the sensor using machine learning algorithms.

# Tech Stack Used 
-ROS2 Humble
-Python
-ScikitLearn
-Pandas 

# Prerequisites 
-A ROS2 Node is an Object in a ROS2 network which performs an action, it can either publish data or obtain data, the data is published to a "Topic" which is essentially allows ROS2 Nodes to subscribe to obtain data / publishing data. 
-Scikitlearn is a Machine learning library in Python which contains various types of machine learning algorithms which can be used. 
-Pandas is a Data processing library in Python which allows us to process data which will be given to a machine learning model, useful for ensuring our model gets clean data to train upon. 

# Working 
- There will be One Publisher ROS2 node which will publish some values over a topic , these values will try to mimic real time sensor data from a distance sensor lets say ultrasonic range finder or laser range finders.
- There will be One subscriber ROS2 node which will subscribe to the topic containing the sensor data and will try to process the data to give a prediction upon the actual distance value.
- Since distance sensor values change linearly with time, we can make use of a Linear Regression Machine learning model in order to make our prediction of the actual distance.
- Here for more adaptability, we will allow the model to learn as the robot keeps moving, hence we will be using Stochastic Gradient Descent optimization for near constant updation of parameters.
- Once the model has been trained on the data, any new incomming data can be sent through the model for a prediction on the actual distance the sensor is measuring

# Where it can be used? 
- This can be used in real world sensor measurements since there can be a lot of noise in sensor data, especially when it comes to the ultrasonic sensor which does not give accurate distance measurements.
- This also demonstrates how machine learning can be applied to make accurate predictions of real world data.

# Limitations
- If the distance change is very fast, then it may have travelled a long distance before the training of one batch of data takes places, hence will not be accurate with fast moving robotics.
- Choosing the wrong learning rate and batch size can potentially create a very inaccurate model.
