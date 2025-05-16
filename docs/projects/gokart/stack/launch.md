---
title: Launching the GoKart
hide_title: True
description: GoKart Launches
sidebar_position: 1
---
import ImageGallery from "@site/src/components/ImageGallery"
import ImageCard from "@site/src/components/ImageCard"

import Stack from "@site/static/img/projects/gokart/launch/stack_running.png"
import Topics from "@site/static/img/projects/gokart/launch/topics_running.png"
import LIDAR from "@site/static/img/projects/gokart/launch/LiDAR_inside.png"
import RTC from "@site/static/img/projects/gokart/launch/rtc_handshake.png"
import FrontCam from "@site/static/img/projects/gokart/launch/Front_Cam.png"

:::danger
Whenever you are working on the GoKart, please make sure to be inside a docker container by using the provided ```run.sh``` script. We use a docker container for security privileges and permissions reasons.
:::

## Full GoKart Launch
To run the entire autonomy stack, please navigate to race_common and rrun these commands:
```bash
source install/setup.bash
tmuxp load tools/tmux_configs/car.yaml
```

To navigate between terminals, here are the keybinds:
1. (CTRL + B) + (0-9) to change windows
2. (CTRL + B) + (:) to get command-prompt
3. (CTRL + B) + (Left, Up, Right, Down) to navigate between panes

For more tmux bindings, please check [this](https://gist.github.com/mzmonsour/8791835).
<ImageCard
    src={Stack}
    alt="GoKart Stack"
    description="Tmuox of GoKart Autonomy Stack"
/>

### Autonomous Flags
In order to make the GoKart start, you must pass in certain flags. 

## Controller Launch

## Localization Launch

## Perception Launch

## Simulator Launch

## Specific Launches 
This will go into into how to launch specific sensors.

### Camera (OAKD)
```bash
source tools/scripts/source_all.sh

# Choose one of the following:
ros2 launch ucsd_gokart_launch oakd_lr.launch.py # Only LR (Front Camera)
ros2 launch ucsd_gokart_launch oakd_lr_yolo.launch.py # LR + YOLO on it
ros2 launch ucsd_gokart_launch oakd_lr_camera_base.launch.py 
ros2 launch ucsd_gokart_launch multi_cam.launch.py # Launches all three cameras
```

#### RViz2 Configurations
1. Global Frame **oak**
2. Add Display Topic - Camera
3. Set Topic to **/img_raw**

<ImageCard
    src={FrontCam}
    alt="Front Camera OAKD LR"
    description="Front Camera of GoKart, OAKD LR" 
/>


### LIDAR (Livox)
```bash
source tools/scripts/source_all.sh
ros2 launch ucsd_gokart_launch livox.launch.py
```

#### RViz2 Configurations
1. Global Frame **livox**
2. Add Display Topic - **PointCloud2**
3. Add Display Topic - **Camera**
4. Set Topic to **/livox**

<ImageCard
    src={LIDAR}
    alt="Indoor LIDAR RViz2"
    description="Indoor LIDAR LIVOX RViz2 Visualization" 
/>

 
### GNSS (Sepentrio)
```bash
source tools/scripts/source_all.sh
ros2 launch ucsd_gokart_launch septentrio_gnss.launch.py
```

#### RViz2 Configurations
1. Global Frame **gnss**
2. Add Topic - **gnss**
3. Set Topic to **gnss**

### RTC
```bash
source tools/scripts/source_all.sh
ros2 launch ucsd_goKart_launch rtc.launch.py
```

#### RViz2 Configurations
1. Global Frame **rtc**
2. Add Topic - **rtc**
3. Set Topic to **rtc**

<ImageCard
    src={RTC}
    alt="RTC"
    description="Real Time Controller for GoKart" 
/>

