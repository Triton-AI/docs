---
title: Installation and Environment
description: Documentation for installating necessary dependencies and environment
sidebar_position: 1
---

import Tower from "@site/static/img/projects/gokart/tower_egg.jpg"
import Dome from "@site/static/img/projects/gokart/dome.jpg"
import WiredNVIDIA from "@site/static/img/projects/gokart/WiredNVIDIA.jpg"
import ImageGallery from "@site/src/components/ImageGallery"
import ImageCard from "@site/src/components/ImageCard"

# Installation & Environment Setup

This documentation outlines the necessary installation and environmental setup, both software and hardware to work on the GoKart.

---
## Network Configuration
The GoKart has a dedicated local network that is used to both remotely SSH into and get internet access from. The configuration is as follows:
- Egg Dome broadcast a network named `UCSDRobofield`.
- Blue WiFi tower receives signal and delivers connectivity to ```UCSDRobofield```
- Devices must connect to `UCSDRobofield` in order to remote SSH into the GoKart.

<ImageGallery
  items={[
    {
      src: Tower,
      alt: 'Tower',
      description: 'The GoKart Tower is Blue and the F1Tenth Tower is Yellow',
    },
    {
      src: Dome,
      alt: 'Dome',
      description: 'Egg Dome for broadcasting UCSDRobofield',
    }
  ]}
/>

## Secure Shell Hosting into the GoKart
You can SSH into the GoKart either wirelessly or wired.

### Wirelessly
To wirelessly SSH into the GoKart, you must be connected to the `UCSDRobofield` network. To access the GoKart, run the bash command:

```bash
ssh jetson@<ip>
```
When prompted, type yes to confirm and save the SSH key fingerprint.

If you'd prefer to SSH using a name instead of an IP address, you can modify the ```/etc/hosts``` file. Add a line in the following format:

```txt
<ip> <hostname>
```

This allows you to connect using:

```txt
ssh jetson@<hostname>
```

### Wired
Connect via wire and run the exact same commands, but the ip will be different. Please consult the TritonAI Drive for all credentials.

<ImageCard
  src={WiredNVIDIA}
  alt="WiredNVIDIA"
  description="Make sure your wire is __"
/>

## Base Environment
The base environment is provided by the Jetson AGX:

| Component           | Version                 |
|---------------------|------------------------ |
| JetPack SDK         | 5.1.3                   |
| L4T Release         | R35.4.1 (Ubuntu 20.04)  |
| CUDA Toolkit        | 11.4                    |
| GPU Drivers         | Installed via JetPack   |

---

