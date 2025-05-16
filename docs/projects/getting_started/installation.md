---
title: Software Environment & Access
hide_title: True
description: Documentation for optional dependencies and environment setup
sidebar_position: 1
---
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import ImageGallery from "@site/src/components/ImageGallery"
import ImageCard from "@site/src/components/ImageCard"

import GKTower from "@site/static/img/projects/getting_started/GKTower.jpg"
import F12Tower from "@site/static/img/projects/getting_started/F1Tower.jpg"
import GKDome from "@site/static/img/projects/getting_started/GKDome.jpg"
import WiredNVIDIA from "@site/static/img/projects/getting_started/WiredNVIDIA.jpg"

### Network Configuration
TritonAI has a dedicated local network that is used to both remotely SSH into and get internet access from.

<Tabs>

<TabItem value="gokart" label="GoKart">
You must have these devices to have the dedicated network **UCSDRobofield** show up on the network.
- Egg Dome broadcast a network named **UCSDRobofield**.
- Blue WiFi tower receives signal and delivers connectivity to **UCSDRobofield**
- Devices must connect to **UCSDRobofield** in order to remote SSH into the GoKart.

<ImageGallery
  items={[
    {
      src: GKTower,
      alt: 'Tower',
      description: 'The GoKart Tower responsible for signal',
    },
    {
      src: GKDome,
      alt: 'Dome',
      description: 'Egg Dome for broadcasting network UCSDRobofield',
    }
  ]}
/>

</TabItem>

<TabItem value="f1tenth" label="F1Tenth">
- Blue Yellow Tower broadcasts and receives a signal for **UCSDRobocars**
- Devices must connect to **UCSDRobocars** in order to remote SSH into any F1Tenth.

<ImageCard
    src={F12Tower}
    alt="F1Tenth Tower"
    description="The F1Tenth Tower responsible for both signal and network of UCSDRobocars"
/>

</TabItem> 
</Tabs>

### Secure Shell Hosting into the GoKart
You can SSH into the system either wirelessly or through a wired connection. Note that the IP address used for wired SSH is different from the wireless one. The wired IP is static and does not change, making it the preferred option when the network is unstable. Keep in mind that connecting to a different network (other than ours) may assign a different wireless IP. To avoid inconsistencies, always use our designated network when accessing the system wirelessly.

#### Wirelessly
To wirelessly SSH into the GoKart, you must be connected to the correct network. For GoKart, it is **UCSDRobofield** and for F1Tenth, it is **UCSDRobocars**. To SSH into any of the projects, run:
```bash
ssh jetson@<ip>
```
When prompted, type yes to confirm and save the SSH key fingerprint.

If you'd prefer to SSH using a name instead of an IP address, you can modify the ```/etc/hosts``` file on **your** computer. Add a line in the following format:

```txt
<ip> <hostname>
```

This maps the IP to the hostname and allows you to connect using:

```txt
ssh jetson@<hostname>
```

#### Wired
Connect via wire and run the exact same commands, but the IP will be different.  

<Tabs>
<TabItem value="gokart" label="GoKart">
You will require a USB-C to USB-A. The USB-C should be connected to the Jetson AGX Xavier while the USB-A goes to your laptop. The image below shows where the USB-C port is on the AGX Xavier.

<ImageCard
  src={WiredNVIDIA}
  alt="WiredNVIDIA"
  description="Make sure your wire is USB-C to USB-A"
/>
</TabItem>

<TabItem value="f1tenth" label="F1Tenth">
Needs to be added here for wired ssh, an image preferably.
</TabItem>
</Tabs>

### X11 Forwarding
Jetson AGX Xavier is a full development computer capable of running graphical applications such as `rviz2`, `gazebo`, and more. When connecting via SSH, you may still want to view these GUI applications on your local machine. There are two main ways we use to enable this:

- **X11 Forwarding**: For launching individual GUI applications remotely over SSH.
- **NoMachine**: For full desktop environment access with minimal setup.

<Tabs>
<TabItem value="x11" label="X11 Forwarding">
X11 is the graphical system used by Linux to display GUI applications. With X11 forwarding, you can run GUI programs on the Jetson and display them on your host computer over SSH.

<Tabs>
<TabItem value="windows" label="Windows">
1. Download and install the [MobaXTerm Home Edition](https://mobaxterm.mobatek.net/download.html). It includes both an SSH client and an X11 server.
</TabItem>

<TabItem value="mac" label="MacOS">
1. Install [XQuartz](https://www.xquartz.org/), an X11 server for macOS.
2. After installation, log out and back in (or reboot).
3. Open XQuartz manually if it doesn't auto-start.
</TabItem>
</Tabs>

Simply replace the terminal you are using with these terminals and the X11 should be setup for you.
</TabItem>
<TabItem value="nomachine" label="NoMachine">
If you prefer full desktop access to your Jetson device (rather than individual terminals), use NoMachine

1. Install [NoMachine](https://downloads.nomachine.com/) on your computer. The NoMachine should already be setup on the Jetsons.
2. Ensure both devices are on the same network.
3. Open NoMachine on your local device and look for the Jetson in the available hosts list.
4. Connect using the Jetson’s username and password to access the full desktop environment.

</TabItem>
</Tabs>

To test whether it works, simply run any GUI command. The simplest one is **xeyes** which belongs to the **x11-apps** package in Ubuntu. If it doesn't exist, install it via ```sudo apt update && sudo apt install x11-apps```.
