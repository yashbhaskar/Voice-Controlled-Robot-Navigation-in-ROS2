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
git clone https://github.com/YourGitHubUsername/ros2_bot_description.git
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


## 📡 ROS 2 Topics & Services Used

| Column 1      | Column 2           | Column 3                  |
|--------------|-------------------|--------------------------|
| Row 1, Col 1 | Row 1, Col 2      | Row 1, Col 3            |
| Row 2, Col 1 | Row 2, Col 2      | Row 2, Col 3            |
