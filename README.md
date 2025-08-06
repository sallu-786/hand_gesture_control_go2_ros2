# 🤖 Hand Gesture Control for Unitree Go2 (ROS2)

A ROS2 Python package to control the **Unitree Go2 robot** using **real-time hand gestures**, powered by `unitree_sdk2_python` and `MediaPipe`.

---

## ✨ Features

- 🤚 Hand gesture recognition with **MediaPipe**
- 🐾 Real-time control of Unitree Go2 robot using **Python SDK**
- ⚙️ Demo code for working with webcam
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
pip3 install opencv-python cyclonedds==0.10.2 rich-click mediapipe
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
ros2 run hand_gesture hand_gesture_demo_node   #run for webcam
ros2 run hand_gesture hand_gesture_node        #run for Go2 camera
```

---

### Using Launch File

```bash
ros2 launch hand_gesture hand_gesture_demo_launch.py    # run for webcam
ros2 launch hand_gesture hand_gesture_launch.py         #run for Go2 camera
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

## 📄 Acknowledgements

This project uses the following libraries and frameworks:

- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [MediaPipe](https://github.com/google/mediapipe)
- ROS2 [Humble](https://docs.ros.org/en/humble/index.html) / [Foxy](https://docs.ros.org/en/foxy/index.html) (Tested on both)
- [h-naderi's python_sdk implmentation](https://github.com/h-naderi/hand_gesture_unitree/tree/master)
  
Supported by [Toyota-Boshoku](https://www.toyota-boshoku.com/)

---

