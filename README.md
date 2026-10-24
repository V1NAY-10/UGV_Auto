# UGV_Auto — Autonomous Navigation and Mapping for UGVs Using LiDAR

A 4WD unmanned ground vehicle that **maps an unknown area with 2D LiDAR SLAM** and then **drives autonomously from point A to point B** with Nav2. You can develop and test everything in Gazebo first, then run it on the real robot.

> **Full technical documentation:** [`docs/PROJECT_DOCUMENTATION.md`](docs/PROJECT_DOCUMENTATION.md). It covers the hardware, power and wiring, firmware, ROS architecture, TF tree, URDF, simulation, odometry, SLAM, AMCL, Nav2 tuning, motor-control math, installation, calibration, the test plan, verification status and troubleshooting. This README is the quick start.

| | |
|---|---|
| Main computer | Raspberry Pi 4B (4 GB), Ubuntu 22.04 + **ROS 2 Humble** |
| LiDAR | RPLIDAR A1 (USB) |
| Motor controller | ESP32, connected to the Pi by **wired GPIO UART** → 2× L298N → 4× 12 V 300 RPM DC gear motors (**no encoders**), skid steer |
| SLAM | `slam_toolbox` (online async) |
| Navigation | Nav2: AMCL, NavFn (A*) planner, Regulated Pure Pursuit controller |
| Simulation | Gazebo Classic 11 + RViz2 |

## How it fits together

```
           ┌──────────────────── Raspberry Pi 4B (ROS 2 Humble) ───────────────────────┐
RPLIDAR ─► │ /scan ─► rf2o laser odometry ─► odom→base_link TF                           │
 A1 USB    │   │                                                                       │
           │   ├────► slam_toolbox ─► /map + map→odom TF    (mapping)                  │
           │   └────► AMCL on saved map ─► map→odom TF       (navigation)              │
           │                                                                           │
           │ Nav2 (planner + controller) ─► /cmd_vel ─► esp32_bridge.py ─► /dev/ttyAMA0 ┼─GPIO UART─► ESP32 ─► 2× L298N ─► 4 motors
           └───────────────────────────────────────────────────────────────────────────┘                ▲
                                                                                            Bluetooth teleop (phone)
```

**No wheel encoders:** the motors are plain DC motors, so wheel odometry isn't available. The robot gets odometry from the LiDAR instead: `rf2o_laser_odometry` estimates motion by matching consecutive scans. SLAM and AMCL then correct the remaining drift. As a fallback, `cmd_vel_odom.py` dead-reckons from the commanded velocity. In Gazebo, the diff-drive plugin provides odometry.

**Why a wired UART link to the ESP32, not Wi-Fi:** every motor command travels over this link 20 times a second. A wire has fixed, millisecond-level latency and nothing to drop out, so the ESP32's 500 ms safety stop only fires when something is really wrong. Over Wi-Fi, latency spikes and dropouts would make the robot stutter or stop in the middle of a path, and the robot would depend on a network being there. The Pi's Wi-Fi stays free for RViz on your laptop.

## Package layout

```
install.sh    one-shot installer: bash install.sh pi | pc
docs/         PROJECT_DOCUMENTATION.md (full technical write-up)
firmware/     esp32_motor_bridge/esp32_motor_bridge.ino   (the only ESP32 file)
config/       slam_params.yaml, nav2_params.yaml, esp32_bridge.yaml
description/  robot.urdf.xacro (+ robot_core, lidar, gazebo_control, inertial_macros)
launch/       sim.launch.py        Gazebo world + robot
              robot.launch.py      real robot bringup (Pi): LiDAR, odometry, motors
              slam.launch.py       slam_toolbox mapping
              navigation.launch.py Nav2 (saved map + AMCL, or slam:=true)
              rviz.launch.py       RViz with the Nav2 view
              rsp.launch.py        robot_state_publisher
maps/         put saved maps here (or pass map:=/abs/path.yaml)
scripts/      esp32_bridge.py, cmd_vel_odom.py, go_to_goal.py
udev/         99-ugv.rules  (/dev/ugv_lidar for the RPLIDAR)
worlds/       obstacles.world (8×6 m arena), empty.world
```

---

## 1. Install

Both machines need **Ubuntu 22.04**. On the Pi, flash **Ubuntu Server 22.04 LTS (64-bit)** with Raspberry Pi Imager. Raspberry Pi OS has no official ROS 2 Humble packages.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/V1NAY-10/UGV_Auto.git
cd UGV_Auto

bash install.sh pc     # laptop/PC: ROS 2 desktop, Gazebo, SLAM, Nav2
bash install.sh pi     # Raspberry Pi: see the list below, then reboot
```

`install.sh pi` does the following:
- installs ROS 2 Humble (ros-base), slam_toolbox, Nav2 and pyserial
- builds the **RPLIDAR A1 driver** (`Slamtec/rplidar_ros`, ros2 branch) and **rf2o laser odometry** from source
- adds the `/dev/ugv_lidar` udev rule and adds you to the `dialout` group
- enables the GPIO UART for the ESP32: sets `enable_uart=1` and `dtoverlay=disable-bt` in `/boot/firmware/config.txt` and removes the serial console. After a reboot, the ESP32 link is `/dev/ttyAMA0`. This turns off the Pi's own Bluetooth; the ESP32 keeps its own Bluetooth.

You can re-run the script safely.

**ESP32 (Arduino IDE):** you need the **esp32 by Espressif** board package, version 2.0.x or 3.x (Boards Manager). No extra libraries are needed, because `BluetoothSerial` comes with the board package. The sketch is confirmed to compile with core 2.0.4 for *ESP32 Dev Module*.

---

## 2. Simulation: SLAM in Gazebo

```bash
# T1: Gazebo world + robot
ros2 launch ugv_auto sim.launch.py
# T2: SLAM
ros2 launch ugv_auto slam.launch.py use_sim_time:=true
# T3: RViz
ros2 launch ugv_auto rviz.launch.py use_sim_time:=true
# T4: drive around until the whole arena is mapped
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive slowly (press `z` a few times to lower the speed) and close loops around obstacles. Save the map:

```bash
mkdir -p ~/ugv_maps
ros2 run nav2_map_server map_saver_cli -f ~/ugv_maps/sim_map
```

Stop SLAM (T2) and teleop (T4).

## 3. Simulation: navigate from A to B

```bash
ros2 launch ugv_auto navigation.launch.py use_sim_time:=true map:=$HOME/ugv_maps/sim_map.yaml
ros2 launch ugv_auto rviz.launch.py use_sim_time:=true
```

In RViz:
1. **Point A:** AMCL assumes the robot starts at the map origin, which is where mapping began. If the robot is somewhere else, click **2D Pose Estimate** and drag on its actual position.
2. **Point B:** click **Nav2 Goal** and drag on the target.

You can also set the points from the command line ("go to x = 4.0 m, y = −0.5 m"):

```bash
ros2 run ugv_auto go_to_goal.py --start 0 0 0 --goal 4.0 -0.5 0
ros2 run ugv_auto go_to_goal.py --start 0 0 0 --goal 4.0 -0.5 0 --goal 0 0 3.14   # A -> B -> C
```

**Navigate while mapping:** you can skip saving a map. SLAM runs and Nav2 plans on the growing map:

```bash
ros2 launch ugv_auto navigation.launch.py use_sim_time:=true slam:=true
ros2 run ugv_auto go_to_goal.py --goal 4.0 -0.5 0    # no --start in SLAM mode
```

---

## 4. Real robot

### 4.1 Wiring

**Raspberry Pi ↔ ESP32 (UART, both sides are 3.3 V, connect directly):**

| Raspberry Pi | ESP32 |
|---|---|
| GPIO14 / TXD (physical pin 8) | GPIO21 (RX) |
| GPIO15 / RXD (physical pin 10) | GPIO22 (TX) |
| GND (physical pin 6) | GND |

Power the ESP32 from the 5 V buck converter into **VIN**, not from the Pi's GPIO 5 V pin. The ESP32's USB port stays free for flashing and Serial Monitor tests.

**ESP32 → L298N (unchanged from the report):**

| Motor | IN1 | IN2 | EN (PWM) | Side |
|---|---|---|---|---|
| M1 | 14 | 27 | 26 | left |
| M4 | 17 | 16 | 4 | left |
| M2 | 25 | 33 | 32 | right |
| M3 | 19 | 18 | 5 | right |

- Keep the L298N jumper caps **off** ENA/ENB, because the ESP32 drives those pins with PWM.
- The ESP32 GND, the L298N GND and the Pi GND must all be connected.
- The E-stop cuts the 12 V motor supply only, so the Pi keeps running.

**RPLIDAR A1** plugs into a Pi USB port. It shows up as `/dev/ugv_lidar`.

### 4.2 Flash and test the ESP32

Open `firmware/esp32_motor_bridge/esp32_motor_bridge.ino` in the Arduino IDE (board: *ESP32 Dev Module*) and upload.

- **Bench test:** open the Serial Monitor over USB (115200 baud, *Newline* line ending) and send `150,150`. All four wheels should spin forward for 0.5 s. If a motor spins the wrong way, set its entry in `INVERT[]` to `true` and re-upload.
- **Bluetooth teleop:** the old keys still work through the `ESP32_Robot` device: `w a s d` to move, `x` to stop, `l`/`h` for low/high speed. Bluetooth overrides the Pi until you send `x`.
- **Test the Pi link** (after `install.sh pi` and a reboot):
  ```bash
  python3 -c "import serial,time; s=serial.Serial('/dev/ttyAMA0',115200); [ (s.write(b'150,150\n'), time.sleep(0.05)) for _ in range(20) ]"
  ```
  The robot should drive forward for about 1 s.

### 4.3 Run: mapping, then A→B

The Pi runs the robot. A laptop on the same Wi-Fi runs RViz. Use the same `ROS_DOMAIN_ID` on both machines, e.g. add `export ROS_DOMAIN_ID=7` to `~/.bashrc` on both.

```bash
# Pi: bringup (RPLIDAR + rf2o odometry + ESP32 bridge)
ros2 launch ugv_auto robot.launch.py
#   options: odom:=cmd_vel (fallback odometry)   lidar_port:=/dev/ttyUSB0

# Pi: mapping
ros2 launch ugv_auto slam.launch.py
# Laptop: RViz + teleop
ros2 launch ugv_auto rviz.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Save the map (on the Pi)
ros2 run nav2_map_server map_saver_cli -f ~/ugv_maps/lab

# Pi: stop slam.launch.py, then navigate on the saved map
ros2 launch ugv_auto navigation.launch.py map:=$HOME/ugv_maps/lab.yaml
# Laptop: Nav2 Goal in RViz, or
ros2 run ugv_auto go_to_goal.py --start 0 0 0 --goal 2.0 1.0 0
```

**Bring it up one layer at a time:** check each step before moving to the next.
1. Motors respond to the Serial Monitor.
2. The UART test above moves the robot.
3. `ros2 topic hz /scan` shows about 5–7 Hz.
4. `ros2 topic hz /odom` shows data.
5. `ros2 topic pub /cmd_vel ...` moves the robot.
6. SLAM.
7. Navigation, starting with a short goal of about 1 m.

### 4.4 Calibration (important without encoders)

Speed control is open loop: `esp32_bridge.py` turns `/cmd_vel` into PWM. The values in `config/esp32_bridge.yaml` therefore decide how closely the robot does what Nav2 asks.

1. **`min_pwm`:** raise PWM from about 60 (Serial Monitor: `60,60`, `70,70`, …) until the robot just starts to move on the floor, then add about 10.
2. **`max_wheel_speed`:** send `255,255` repeatedly for 2 s, measure the distance driven, and divide by 2. Use that number in m/s.
3. **`turn_gain`:** run `ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.8}}" -r 10` for 10 s. Compare the actual rotation with the expected 8 rad (about 1.3 turns). If the robot under-rotates, increase `turn_gain`.
4. If the robot overshoots goals, lower `desired_linear_vel` in `config/nav2_params.yaml`.
5. If the scan looks rotated in RViz (for example, walls in front of the robot appear behind it), set `lidar_yaw` in `description/lidar.xacro`. On the A1, the cable side usually faces the robot's rear.

Measure your chassis and update the dimensions at the top of `description/robot_core.xacro`. Also update `footprint` in `nav2_params.yaml` and `wheel_separation` in `esp32_bridge.yaml` to match.

---

## Project status

| | |
|---|---|
| ESP32 firmware | Compiles (Arduino ESP32 core 2.0.4, 77 % flash) |
| URDF, configs, Python, launch files | Validated (xacro expands, YAML parses, Python compiles) |
| colcon build, Gazebo + SLAM + Nav2 run | Not yet run on Ubuntu |
| `install.sh pi`, real-robot bring-up | Not yet run on the Pi |

Follow the bring-up order in section 4.3 and report any errors. Details are in [docs §20](docs/PROJECT_DOCUMENTATION.md#20-verification-status).

## Troubleshooting

| Symptom | Fix |
|---|---|
| RViz error `Frame [map] does not exist` | SLAM or AMCL isn't running yet. In AMCL mode, set a **2D Pose Estimate**. |
| Robot doesn't move in Gazebo | Check that `ros2 topic echo /cmd_vel` shows messages, and that Gazebo isn't paused. |
| `Cannot open /dev/ttyAMA0` | Reboot after `install.sh pi`. Check `ls -l /dev/ttyAMA0` and that you're in the `dialout` group (`groups`). |
| UART connected but motors don't react | TX and RX are swapped (Pi TX must go to ESP32 RX), there's no common GND, or Bluetooth teleop is active (send `x`). |
| `/dev/ugv_lidar` missing | Check `ls /dev/ttyUSB*`, or run with `lidar_port:=/dev/ttyUSB0`. |
| `Could not transform ... odom` on the real robot | Odometry isn't publishing. Check `ros2 topic hz /odom`, or try `odom:=cmd_vel`. |
| Real robot hums but doesn't move | `min_pwm` is too low, or the battery is low. |
| Map smears or doubles on the real robot | Drive slower and turn more slowly. rf2o and the ~5.5 Hz A1 need gentle motion. |

More cases are in [docs §21](docs/PROJECT_DOCUMENTATION.md#21-troubleshooting).
