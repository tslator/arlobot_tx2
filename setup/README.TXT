This document describes how to install all components needed for Arlobot on TX2

1. Bringup TX2 (after every JetPack install we start here)
2. Mount external drive (I'm using a 120G sata drive)
3. Add call to arlobot.bash to ~/.bashrc
4. Bring Ubuntu up-to-date
5. Use the Jetson Hacks ROS install scripts
6. Use the Arlobot ROS install script


ARLOBOT PATH SETUP

The pathsetup.conf contains environment variables which point to various components of the
Arlobot package.  A call to pathsetup.conf should be added to .bashrc.  Additionally, the 
ARLOBOT_PYTHON_PATH should be appended to PYTHONPATH.


----------------------

JETSONHACKS ROS


This folder contains scripts obtained from the JetsonHack github.  Included is a script
to update repositories, install ROS, and create a catkin workspace.

----------------------

PIP/PACKAGE DEPENDENCIES

Required Installations

* Python Packages
    - pip
        - sudo apt-get install python-pip
        - pip install -U pip setuptools
    - can
        - https://python-can.readthedocs.io/en/stable/installation.html
        - pip install python-can
    - smbus
        - sudo apt-get install libi2c-dev i2c-tools
        - sudo apt-get install python-smbus
        - Add users to i2c group
	    sudo usermod -a -G i2c nvidia
            Note: logout and login to take effect
        Note: Test i2c connection via smbus
            - i2cdetect -y -r 1

	nvidia@tegra-ubuntu:~$ i2cdetect -y -r 1
	     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
	00:          -- -- -- -- -- 08 -- -- -- -- -- -- -- 
	10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
	20: -- -- -- -- -- -- -- -- 28 -- -- -- -- -- -- -- 
	30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
	40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
	50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
	60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
	70: -- -- -- -- -- -- -- --

* Setup rules to access i2c
    sudo bash -c 'echo \''KERNEL==\"i2c-[0-7]\",MODE=\"0666\"\'' > /etc/udev/rules.d/90-i2c.rules'
	
Note: At this point (with ARLOBOT_PYTHON_PATH (see pathsetup.conf) appended to PYTHONPATH) it
should be possible to run the HAL node, e.g., rosrun arlobot_bringup halnode.py

--------------------------------------------------------------------------------------

ROS PACKAGE DEPENDENCIES

- joy
    sudo apt-get install ros-kinetic-joy


- xbox360
    Ref: https://andrewdai.co/xbox-controller-ros.html
    sudo add-apt-repository ppa:grumbel/ppa
    sudo apt-get update

    Test:
        sudo xboxdrv
    sudo apt-get install xboxdrv

--------------------------------------------------------------------------------------

HOSTNAME

1. Set hostname to arlobot
    copy the hostname file in this directory to /etc/.

    sudo cp hostname /etc/.

    REBOOT for it to take place


        

