
# RandomSensorDataHandling: Adaptive ROS 2 Sensor Filtering

![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Python 3.10](https://img.shields.io/badge/Python-3.10-yellow)
![Scikit-Learn](https://img.shields.io/badge/ML-Scikit--Learn-orange)

## 📖 Overview

**RandomSensorDataHandling** is a ROS 2 package designed to solve a common problem in robotics: **noisy sensor data in dynamic environments**.

Standard filters (like Kalman filters) often rely on static parameters. This project implements an **Online Machine Learning** approach using Stochastic Gradient Descent (SGD) to dynamically learn the noise patterns of a sensor in real-time. 

To ensure the robot never "freezes" while training the model, the system utilizes a custom **Multithreaded Architecture** with `ReentrantCallbackGroups` and atomic locking, allowing it to process heavy mathematical operations on a background thread while continuing to receive high-frequency sensor data.

---

## 🚀 Key Features

* **Online Learning:** continuously trains an `SGDRegressor` model on incoming data streams, adapting to changes in the environment immediately.
* **Multithreaded Execution:** Uses `MultiThreadedExecutor` to separate data collection (high priority) from model training (lower priority/heavy computation).
* **Thread Safety:** Implements strict `threading.Lock` protocols to manage shared data buffers between callbacks and processing threads.
* **Real-Time Simulation:** Includes a generator node that simulates a robot moving with random velocity and noisy sensor readings.

---

## 🛠️ Tech Stack

* **Framework:** ROS 2 Humble
* **Language:** Python 3.10+
* **Machine Learning:** Scikit-Learn (`SGDRegressor`, `StandardScaler`)
* **Data Processing:** NumPy
* **Concurrency:** Python `threading`, ROS 2 Executors

---

## ⚙️ Architecture

### 1. Data Generator (`data_gen.py`)
Acts as the hardware simulation layer.
* **Publishes:** * `/sensor` (`Float64`): Distance measurements with added random noise.
    * `/odometry` (`Int32`): The actual control/velocity inputs (Ground Truth).

### 2. Data Processor (`dataprocessing.py`)
Acts as the intelligent driver/filter.
* **Subscribes:** Listens to both `/sensor` and `/odometry`.
* **Buffer Logic:** Accumulates data in a temporary list.
* **Thread Trigger:** When the buffer hits 10 items, it atomically locks the data, copies it for processing, and clears the main buffer.
* **Background Task:** A separate thread trains the ML model on the copied batch and updates the global model weights (`w`) and bias (`b`).

---

## 📦 Installation

### Prerequisites
Ensure you have the following installed:
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* Python 3 dependencies:

```bash
pip install scikit-learn numpy
````

### Setup Workspace

1.  Create a ROS 2 workspace (if you haven't already):

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  Clone this repository or create the package:

    ```bash
    ros2 pkg create --build-type ament_python random_sensor_handling
    # Place the python scripts inside the package folder
    ```

3.  Build the package:

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select random_sensor_handling
    source install/setup.bash
    ```

-----

## ▶️ Usage

You will need two terminal windows to run this simulation.

**Terminal 1: Start the Processor (The Brain)**
This node listens for data and performs the ML training.

```bash
source install/setup.bash
ros2 run random_sensor_handling data_processing
```

**Terminal 2: Start the Generator (The Simulation)**
This node starts publishing noisy data.

```bash
source install/setup.bash
ros2 run random_sensor_handling data_gen
```

### Expected Output

In the **Processor** terminal, you should see logs like:

```text
[INFO]: Received data, Distance measured: 12.5
[INFO]: Actual len : 10 Measured len : 10
[WARN]: LOCKED & LOADED. Processing 10 items...
[INFO]: ==== PROCESSED ====
[INFO]: Predicted distance covered : 14.2
```

-----

## ⚠️ Limitations

1.  **Latency:** If the robot moves significantly faster than the model can train (processing time \> data arrival rate), the predictions may lag behind the current state.
2.  **Hyperparameters:** The learning rate (`eta0`) is currently static. An adaptive learning rate would improve convergence in highly variable environments.
3.  **Data Sync:** The system assumes a loose temporal correlation between the sensor and odometry messages arriving within the same batch window.

## 📜 License

This project is open-source and available under the MIT License.

```
