# 🎙️ Voice-Controlled Robot Navigation in ROS 2 🤖  

This ROS 2 package enables **voice-controlled navigation** for a robot using **speech recognition**. It listens for voice commands, processes them, and moves the robot accordingly in a simulated or real-world environment.

---

## 🚀 Features  
✅ **Voice Recognition:** Uses `speech_recognition` and `PyAudio` to process commands  
✅ **Real-time ROS 2 Control:** Converts speech to movement commands  
✅ **Robot Navigation:** Uses `geometry_msgs/Twist` to send velocity commands  
✅ **Simulation & Real Robot Support:** Works with Gazebo and physical robots  
✅ **Modular & Extendable:** Can integrate with other ROS 2 packages  

---

## 🛠️ Installation  

### 1️⃣ **Clone the Repository**  
```bash
cd ~/ros_ws/src
git clone https://github.com/yashbhaskar/Voice-Controlled-Robot-Navigation-in-ROS2.git
cd ~/ros_ws
```

### 2️⃣ **Install Dependencies** 
Ensure you have the necessary Python libraries installed:
```bash
pip install speechrecognition pyaudio
```

### 3️⃣ **Build the Package** 
```bash
colcon build --packages-select ros2_bot_description
source install/setup.bash
```

## 🎮 Usage

### 1️⃣ Launch Gazebo and State_Publisher:
```bash
ros2 launch ros2_bot_description gazebo.launch.py
ros2 launch ros2_bot_description state_publisher.launch.py
```

### 2️⃣ Run the Voice Command Node:
```bash
ros2 run ros2_bot_description voice_command.py
```

### 3️⃣ Run the Robot Controller Node:
```bash
ros2 run ros2_bot_description robot_controller.py
```

### 4️⃣ Speak Commands:
Once the node starts, it will listen for voice commands such as:
``"move forward"`` – Moves the robot forward
``"turn left"`` – Turns the robot left
``"turn right"`` – Turns the robot right
``"stop"`` – Stops the robot

## 📹 Demo Video

https://github.com/user-attachments/assets/ffe5805c-e38b-40e1-bdbb-48c0156fc791

## 📡 ROS 2 Topics & Services Used

| Topic Name   | Message Type      | Description             |
|--------------|-------------------|--------------------------|
| `/cmd_vel` | `geometry_msgs/Twist`| Publishes velocity commands to the robot|
|`/voice_command` | `std_msgs/String`| Publishes recognized voice commands |


## 📂 Project Structure
```
ros2_bot_description/
│── launch/                        # Launch files for ROS 2
│   ├── gazebo.launch.py
│   ├── state_publisher.launch.py
│   ├── slave.launch.py
│── models/
│   ├── meshes
│   ├── urdf
│── scripts/                        # Python scripts for voice recognition
│   ├── robot_controller.py
│   ├── voice_command.py
│── worlds/
│   ├── new_world.sdf
│── CMakeLists.txt                  # CMake build configuration
│── package.xml
│── README.md
```

## 📡 RQT Graph Visualization
Below is an RQT graph of the ROS 2 nodes and topics used in this package:

![Screenshot from 2025-02-09 16-07-54](https://github.com/user-attachments/assets/36fdc976-b912-4bae-b3a4-cb0c9236f477)

This shows how voice commands are processed and sent to the robot.

## 🤝 Contributing

Feel free to fork this repository, create a pull request, or open an issue if you have suggestions or find bugs.

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com
📌 GitHub: https://github.com/yashbhaskar
