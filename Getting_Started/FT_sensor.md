---
layout: default
title: Force Torque Sensor Setup
parent: Getting Started
nav_order: 2
---

# Introduction

Each appendage uses a HEX12 sensor manufactured by [Resense](https://www.resense.io/en-en/)

Please note that these sensors are quite small, and must be handled with care when mounting to the chopstick backend, and while inserting the chopstick appendage.

There is a quickstart guide that comes with the sensor components, that has similar instructions to these. Feel free to refer to both, as this set of instructions has a few additional notes for HASHI specifically.

## Sub-system Assembly Guide

The two main components of the HEX12 sensing package are the sensor itself, and the electronics control box. Instructions for assembly and configuration are detailed as follows:

1. Look at the 8-switch configuration on the front of the control box, and make sure that switches XX are flipped to ON, and all others are flipped to OFF

2. Insert the flat-flex HEX12 sensor cable into the board-mount connector with the flat-flex metallic pads facing _away_ from the PCB

3. Plug the other end of that cable into the electronics control box, and engage the screw terminals

4. Insert a small zip-tie into the divot on the spherical ball joint mount and use to secure this cable

    * Side note: when routing this cable away from your HASHI platform to the control box, make sure you leave enough slack near HASHI to allow for full linear range of motion without any binding

6. Connect the micro USB cable from the electronics box to your computer, and turn on the electronics box

Good job! Now you are ready to calibrate your HEX12 sensor!

## Electronics and Initial Software Configuration

After everything is assembled, we need to do a couple things to make sure that the sensor is calibrated correctly

1. Locate the USB flashdrive shipped with the sensor and electronics and plug into your computer. This flashdrive contains a few things

* A GUI that allows for customization and viewing of raw or processed signals on each of the sensor channels

* A calibration matrix _specific to each individual sensor_ that must be loaded 

## Software Configuration with ROS

One of our contributions with this work was a ROS1 driver for these sensors. This is used to visualize the filtered output in RQT, as well as save time-dependent data as a bag file