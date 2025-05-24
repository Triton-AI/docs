---
title: Getting Started
description: Introduction to the GoKart Project
sidebar_position: 1
---

## Overview
Goal of the project is to compete in the [Autonomous Karting Series](https://www.autonomouskartingseries.com). We use a GoKart manufactured by [TopKart](https://topkartusa.net/electric/) with modifications to drive autonomously.

## Table of Contents

- [Overview](#overview)
- [Sensor Stack](#sensor-stack)
- [Basic Electrical](#basic-electrical)
- [Basic Hardware](#basic-hardware)


## Sensor Stack
As of 05-22-2025

- Lidar: Livox HAP
- 3 Cameras: 1 [OAK-D Long Range](https://shop.luxonis.com/products/oak-d-lr) and 2 [OAK-D Pro W](https://shop.luxonis.com/products/oak-d-pro-w)
- GPS: [PointOneNav Atlas](https://pointonenav.com/atlas/) with [Beitian Antenna](https://store.beitian.com/collections/gnss-antenna/products/beitian-rtk-omnidirectional-differential-high-precision-four-star-full-frequency-gnss-four-arm-spiral-antenna-formation-flight-control-bt-t076) that gives 10cm accuracy with RTK corrections
- 2 Vescs: One for the electric motor, One for steering unit
- [Nvidia Jetson AGX Xavier Development Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/)
- [Rev robotics Absolute Through Bore Encoder](https://www.revrobotics.com/rev-11-1271/)
- Brake [KarTech Linear Actuator](https://kar-tech.com/12-24v-linear-throttle-actuator-1.html)

## Basic Electrical

### Main Power

The GoKart runs a 48V system that is stepped down to 24V, 12V, and 5V for different electronic requirements.
It uses 2 24V LiFePO4 batteries wired in series to achieve 48V.
These go into a switch which allows the kart to be turned on and off. We use 48V for the motors and step down the voltage for everything else which is distributed through PDBs (Power Distribution Boards).

### Motor Control

The commands to the motors are sent through VESCs which obtain their commands from the RTC (real time controller) in the rear box. The RTC receives high level throttle and steering commands from either manually (RadioMaster Pocket) or computed from the Jetson through a serial connection.

### Power Distribution

- 24V Back: KarTech Brake, PoE for Bullet AC, Switch, Lighttower light.
- 12V Back: Jetson (Computer), RTC Green Breakout Board, PON Atlas (GPS), Jetson PCIe USB board power
- 5V Back: RadioMaster Pocket Receiver, 2 5V Relays, Brake Pressure Sensor, Wheel Speed Encoder
- 12V Front: Livox HAP (Lidar), White Lights for night driving, FPV, FPV Fans
- 5V Front: OAK-Ds with separated power (Camera)

### Communication

The GoKart has a switch that connects the different sensors together. The main network connection comes from a [unifi bullet](https://store.ui.com/us/en/category/wireless-airmax-5ghz/products/b-db-ac) that is on a local network you are able to connect to.
There used to be a LTE cellular router (Pepwave) to give internet, this was phased out as internet could be given through the bullet.

- Bullet
- Jetson
- Lidar
- PoE Camera
- Atlas GPS
- Extra

## Basic Hardware

### Motors

We use the electric GoKart from [TopKart](https://topkartusa.net/electric/) as the base chassis. 
The main driving electric motor with a hall sensor is controlled by a VESC.
The steering unit is pulled from an (~2005) junkyard Toyota Prius and is sent power signal through the front steering VESC. 

### Mounted Units

The front slanted tower contains the lidar and the cameras powered through PDBs and the steering VESC.
The box mounted on the back contains the computer and all other electronics that go into controlling the GoKart.
The brake uses a KarTech linear actuator to push the brake lever and clamp the brake pads on the rear axle.
The back box is made with plastic from ePlastics that cannot be lasercut and only water jet.
The antenna is mounted on a pole above the GoKart to avoid GPS obstructions and interference.
