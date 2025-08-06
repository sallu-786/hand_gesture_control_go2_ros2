# 🤖 Hand Gesture Control for Unitree Go2 (ROS2)

A ROS2 Python package to control the **Unitree Go2 robot** using **real-time hand gestures**, powered by `unitree_sdk2_python` and `MediaPipe`.

---

## ✨ Features

- 🤚 Hand gesture recognition with **MediaPipe**
- 🐾 Real-time control of Unitree Go2 robot using **Python SDK**
- ⚙️ ROS2 integration for flexible deployment
- 🛠️ Developer-friendly launch file support

---

## 📦 Prerequisites

Make sure the **Unitree Python SDK** is installed:

```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .  
```
visit [Official SDK](https://github.com/unitreerobotics/unitree_sdk2_python.git)  if facing error



Install required Python packages:

```bash
pip3 install opencv-python numpy cyclonedds==0.10.2 rich-click mediapipe
```

---

## 🧰 Step-by-Step Installation

### 1. Create Your ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

### 2. Clone This Repository into `src`

```bash
cd ~/ros2_ws/src
git clone https://github.com/sallu-786/hand_gesture_control_go2_ros2.git
```

---

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 Run the Node

### Using ROS2 CLI

```bash
ros2 run hand_gesture hand_gesture_node
```

---

### Using Launch File

```bash
ros2 launch hand_gesture hand_gesture_launch.py
```

---

## ⚡ Developer Tips

If you're only editing existing Python files:

```bash
colcon build --symlink-install
```

> ⚠️ **Note**: If you add new files, run a full `colcon build` again.

---

## 🧠 Auto-Source ROS2 Environment

Add this line to your `~/.bashrc` for automatic sourcing:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 📄 License & Acknowledgements

This project uses the following libraries and frameworks:

- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [MediaPipe](https://github.com/google/mediapipe)
- ROS2 (**Humble** / **Foxy**)

---

