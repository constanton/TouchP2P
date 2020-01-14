# TouchP2P

C/C++ application for bilateral teleoperation (teleoperation with force feedback) with two 3D Systems Touch devices in Position-Position configuration.

This software has been tested under Ubuntu 18.04 and Fedora 30 using 3D Systems Touch (USB interface) and Geomagic Touch (Ethernet interface) devices. The main libraries used for this application are OpenHaptics and Boost C++.

## Dependencies
**For Ubuntu**:
```
sudo apt-get install make g++ qt5-default libqt5widgets5 libqt5opengl5 freeglut3-dev libncurses5-dev zlib1g-dev
```
**For Fedora**:
```
dnf install make g++ qt5-devel freeglut ncurses-devel zlib-devel
```
Of course to be able to calibrate and control the haptic devices you need to install the device drivers and OpenHaptics libraries which are currently available (along with installation instructions) on 3dssupport.microsoftcrmportals.com.

You also need to install the Boost C++ library (for the multi-threading and networking functionalities of the code). I recommend downloading the latest version from boost.org.

## Set up
Each haptic device needs to be connected to a PC. Both PCs should belong to the same network and IPs must be defined in the touchp2p.cpp file. Each PC must run the application using the touchp2p.cpp file by assigning the correct IPs at the top of the file.

For your convenience I have also added a Makefile similar to the ones that the OpenHaptics examples use.

## Future work

There is always room for improvement. Apart from improving the code itself, the PID controller needs fine-tuning.
Another interesting feature would be to include some stability control algorithm to compensate for latency introduced by the network.

**Special thanks** to Professor Marcelo A. C. Fernandes as this code is based on previous effort for a similar project.
