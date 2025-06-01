---
title: Camera and LIDAR Fusion
hide_title: True
description: Camera and LIDAR Sensor Fusion
sidebar_position: 1
---
import ImageGallery from "@site/src/components/ImageGallery"
import ImageCard from "@site/src/components/ImageCard"

import Stack from "@site/static/img/projects/gokart/launch/stack_running.png"
import Topics from "@site/static/img/projects/gokart/launch/topics_running.png"
import LIDAR from "@site/static/img/projects/gokart/launch/LiDAR_inside.png"
import RTC from "@site/static/img/projects/gokart/launch/rtc_handshake.png"
import FrontCam from "@site/static/img/projects/gokart/launch/Front_Cam.png"

:::info
The documentation is made to explain the [sick_lidar_fusion_project](https://github.com/Triton-AI/sick_lidar_fusion_project). For source code, please check the github.
:::

## 

## Launching the Fusion Node
In order to launch the fusion node and all the necessary dependencies, run the following command:

```bash
tmuxp load tools/tmux_configs/ucsd_perception_gokart.yaml
```

