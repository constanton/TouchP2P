# TouchP2P

C/C++ application for bilateral teleoperation (teleoperation with force feedback) with two 3D Systems Touch devices in Position-Position configuration.

This software has been tested under Ubuntu 18.04 and Fedora 30 using 3D Systems Touch (USB interface) and Geomagic Touch (Ethernet interface) devices. 

## Dependencies
**For Ubuntu**:
```
sudo apt-get install make g++ qt5-default libqt5widgets5 libqt5opengl5 freeglut3-dev libncurses5-dev zlib1g-dev
```
**For Fedora**:
```
dnf install make g++ qt5-devel freeglut ncurses-devel zlib-devel)
```
Ofcourse to be able to calibrate and control the haptic devices you need to install the device drivers and OpenHaptics libraries which are currently available (along with installation instructions) on 3dssupport.microsoftcrmportals.com.

You also need to install the Boost C++ library (for the multi-threading and networking functionalities of the code). I recommend downloading the latest version from boost.org.

## Set up
Each haptic device needs to be connected to a PC. Both PCs should belong to the same network and IPs must be defined in the touchp2p.cpp file.

## Future work

There is always room for improvement. Apart from improving the code itself, the PID controller needs fine-tuning.
Another interesting feature would be to include some stability control algorithm to compensate for latency introduced by the network.

**Special thanks** to Professor Marcelo A. C. Fernandez as this code is based on previous effort for a similar project.
