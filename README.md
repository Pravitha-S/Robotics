# Robotics Projects

This repo contains my robotics project work.  
Currently includes:
- 🤖 **Rover + Nav2** → SLAM, navigation, teleop
- 🦾 **Robotic Arm** → MG995,ESP32

---

## 📂 nav2_rover
   Raspberry Pi based rover with SLAM & navigation.

### Features
- Raspberry Pi with **Ubuntu 22 + ROS2 Humble**
- Lidar (SLIDAR) → odometry (`rf2o_odom`)
- Teleop via `/cmd_vel` → serial → motor driver
- ROS node subscribes to `/cmd_vel`, converts to PWM, and sends serial to Arduino
- Arduino (ESP32/UNO + L298N) receives `"linear,angular"`, drives motors
- Mapping with `slam_toolbox`
- Remote SSH from laptop to Pi
- Tested with teleop (`i j k l u m , .` keys) and SLAM mapping
- Tested with 6.5 cm wheels, 13 cm wheelbase

Example screenshots:

![Teleop](nav2-rover/screenshots/teleop-twist-keyboard)  
![Mapping](nav2-rover/images/rover-gazebo-mapping.jpg)

### Workflow
1. **Teleop**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
→ publishes /cmd_vel
2. **Velocity subscriber**
   ros2 run nav2_rover cmdvel_listener --ros-args -p port:=/dev/ttyUSB0
3. **Arduino side (ESP32 / Uno + L298N)**
   Reads serial input "linear,angular"
   Converts to PWM (–255 to +255) for left/right wheels
   Tested with 6.5 cm wheels, 13 cm base width, ~50 RPM motors
4. **SLAM**
   ros2 launch nav2_rover mapping.launch.py

**Steps to Do**

   1. ROS2 Humble setup (Raspberry Pi + Laptop)
   2. Implemented /cmd_vel subscriber → serial output
   3. Verified motor actuation (PWM scaling)
   4. SLIDAR working with rf2o_odom
   5. Rover chassis assembly
   6. SSH connection from laptop → Raspberry Pi
   7. Remote teleop tested
   8. Mapping with slam_toolbox
   9. Navigation tuning

📂 Folder Structure

   Robotics/
│── README.md
│
├── robotic-arm-pick-place/   # Arm to pick and place objects project
|    │── images
|    │── videos
|    │── app.py              
│
└── nav2_rover/               # Main rover project 
    │── package.xml
    │── CMakeLists.txt
    │── setup.py
    │
    ├── src/
    │   ├── cmdvel_listener.py    # ROS2 node: subscribe to /cmd_vel & send serial
    │   └── __init__.py
    │
    ├── arduino/
    │   └── cmdvel_listener.ino   # Arduino code for motor control (your working sketch)
    │
    ├── launch/
    │   └── mapping.launch.py     # Example launch for lidar+odom+slam
    │
    ├── config/
    │   └── mapper_params_online_async.yaml  # Tuning for slam_toolbox
    │
    └── images/
    |   ├── rover-top-view.jpg
    |   ├── rover-side-view.jpg
    |   ├── rover-gazebo-mapping.jpg
    ├
    │
    └── screenshots/
        ├── bcr-bot-cmd-terminal.png
        ├── cmdvel-listener.png
        ├── remmina-remote-desktop.png
        └── rf2o-laser-odom.png
        ├── ssh-raspberry-pi.png
        └── teleop-twist-keyboard.png

   ---

## 🦾 MG995 Robotic Arm — Pick & Place (ESP32)

**Short:** ESP32 + MG995 robotic arm performing a simple pick & place demo.  
Contains Arduino code (`app.ino`), wiring diagram, video demo, and calibration notes.

- Controlled using ESP32 PWM outputs
- 4-DOF arm with MG995 servos
- Calibration script included
- Pick & place demo verified with small objects

📂 See [`robotic_arm/`](Robotic-arm-mg995-pick-place)

Example images:
  
![Demo Video](Robotic-arm-mg995-pick-place/demo.gif)