# WI25-DSC190-Team-1

## ART Team Progress on Jetson UCSD Go Kart

---

## How to Run the Sensors on the Go Kart

1. **Connect to Wi-Fi**

   ```
   Wi-Fi: UCSDRoboCar  
   Password: UCSDrobocars2018
   ```

2. **Use NoMachine to Connect to the Jetson**

   ```
   Connection: ucsd-gk  
   (Ask Aryan for IP address if not connecting)
   ```

3. **Run the Docker Container**

   ```bash
   cd projects/race_common/
   sudo bash run.sh  # Password: jetsonucsd
   cd opt/race_common/
   ```

4. **Load Stack**

   ```bash
   tmuxp load tool/tmux_configs/ucsd_gokart.yaml
   ```

   - Windows for each program specified in `ucsd_gokart.yaml` will open.
   - Use `CMD + B + [Window #]` to switch.

   ![Image](/img/stack_running.png)

5. **Launch RViz2 to Visualize Sensors**

   - Visualize any sensor (camera, IMU, GNSS) in RViz2. Ensure UCSD config is selected.
   - Steps:
     1. Open new viz and select topic.
     2. Set fixed frame to relevant frame.
     3. If needed, check topics in new tab:
        ```bash
        docker exec -it ucsd-gk bash
        ros2 topic echo [topic_name]
        ros2 topic list
        ```

   ![Image](/img/topics_running.png)

---

## How to Check Individual Sensors without Docker

1. **Use NoMachine to Connect to the Jetson**

   ```
   Wi-Fi: UCSDRoboCar  
   Password: UCSDrobocars2018
   ```

2. **Login**

   ```
   Host: ucsd-gk  
   Username: jetson  
   Password: jetsonucsd
   ```

3. **Jetson (Terminal 1)**

   Ensure you're at `jetson@ucsd-gk:~$`

   ```bash
   cd projects/race_common
   source opt/ros/humble/setup.bash
   source tools/scripts/source_all.sh
   rviz2
   ```

   ![Image](/img/source_all.png)

   ### For Old Jetson:

   ```bash
   cd projects/race_common
   source install/setup.bash
   source tools/scripts/source_all.sh
   rviz2
   ```

   ### For Docker:

   ```bash
   docker exec -it nostalgic_engelbart /bin/bash
   docker run -it --privileged --runtime=nvidia --network host \
     ghcr.io/aryan-palaskar/art-built-gk:devel bash

   # Inside Docker
   cd /opt/race_common
   source install/setup.bash
   source tools/scripts/source_all.sh
   ```

---

4. **Camera (Terminal 2)**

   ```bash
   source tools/scripts/source_all.sh

   # Choose one of the following:
   ros2 launch ucsd_gokart_launch oakd_lr.launch.py 
   ros2 launch ucsd_gokart_launch oakd_lr_yolo.launch.py 
   ros2 launch ucsd_gokart_launch oakd_lr_camera_base.launch.py
   ros2 launch ucsd_gokart_launch multi_cam.launch.py 
   ```

   - **RViz2 Setup:**
     - Set Global Options: `oak`
     - Add Camera
     - Set topic: `/img_raw`

---

5. **LiDAR (Terminal 3)**

   ```bash
   cd projects/race_common
   source tools/scripts/source_all.sh
   ros2 launch ucsd_gokart_launch livox.launch.py 
   ```

   - **RViz2 Setup:**
     - Set Global Options: `livox`
     - Add `PointCloud2`
     - Add `Camera`
     - Set topic: `livox`

   ![Image](/img/lidar_0.png)

---

6. **GNSS (Terminal 4)**

   ```bash
   cd projects/race_common
   source tools/scripts/source_all.sh
   ros2 launch ucsd_gokart_launch septentrio_gnss.launch.py 
   ```

   - **RViz2 Setup:**
     - Set Global Options: `gnss`
     - Add: `gnss`
     - Set topic: `gnss`

---

7. **RTC (Terminal 5)**

   ```bash
   cd projects/race_common
   source tools/scripts/source_all.sh
   ros2 launch ucsd_gokart_launch rtc.launch.py 
   ```

   - **RViz2 Setup:**
     - Set Global Options: `rtc`
     - Add: `rtc`
     - Set topic: `rtc`

![Image](/img/rtc_handshake.png)
