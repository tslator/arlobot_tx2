This document describes how to install all components needed for Arlobot on TX2

0. Move home to external drive
1. Bringup TX2 (after every JetPack install we start here)
2. Mount external drive (I'm using a 120G sata drive)
3. Add call to arlobot.bash to ~/.bashrc
4. Bring Ubuntu up-to-date
5. Use the Jetson Hacks ROS install scripts
6. Use the Arlobot ROS install script


MOVE HOME TO EXTERNAL DRIVE

After Jetpack update the home directory will be located on the eMMC.  Follow the directions in the
reference to mount the SSD drive and point the /home directory the SSD mount point

Ref: https://www.maketecheasier.com/move-home-folder-ubuntu/


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
    sudo apt-get install xboxdrv

    Test:
        sudo xboxdrv

- Addition features (service
    Ref: https://github.com/raelgc/ubuntu_xboxdrv

    sudo apt-add-repository -y ppa:rael-gc/ubuntu-xboxdrv
    sudo apt-get update
    sudo apt-get install ubuntu-xboxdrv

    sudo service xboxdrv restart

    Test:
        rosrun joy joy_node

--------------------------------------------------------------------------------------

HOSTNAME

1. Set hostname to arlobot
    copy the hostname file in this directory to /etc/.

    sudo cp hostname /etc/.

    REBOOT for it to take place


        
GATE ONE SETUP

Ref: http://liftoffsoftware.com/
GitHub: https://github.com/liftoff/GateOne/

Gate One is a full-featured shell that can run in a web browser.
It is written in Python and is Tornado-based.

Installation
Ref: https://www.digitalocean.com/community/tutorials/how-to-ssh-into-your-vps-from-the-browser-with-gateone

I followed the instructions here but couldn't get the debian package stuff to work.  In the INSTALL.TXT
for the Gate One source there are number of options including using setup.py.

* curl -L http://github.com/liftoff/GateOne/archive/master.tar.gz > ~/arlobot_development/gateone/
* tar xvzf gateone.tar.gz
* sudo pip install tornado
* sudo python setup.py 


PYTEST install

sudo pip install pytest
sudo pip install pytest-pythonpath


NODE RED



Start on Bootup

Ref: https://nodered.org/docs/getting-started/running


Using PM2

PM2 is a process manager for Node.js. It makes it easy to run applications on boot and ensure they are restarted if necessary.

Note: PM2 is released under GNU-AGPL-3.0 license - please check the terms of the license before deploying.
1. Install PM2

sudo npm install -g pm2
Note: sudo is required if running as a non-root user on Linux or OS X. If running on Windows, you will need to run in a command shell as Administrator, without the sudo command.
If running on Windows, you should also ensure tail.exe is on your path, as described here.
2. Determine the exact location of the node-red command.

If you have done a global install of node-red, then on Linux/OS X the node-red command will probably be either: /usr/bin/node-red or /usr/local/bin/node-red. The command which node-red can be used to confirm the location.

If you have done a local install, it will be node_modules/node-red/bin/node-red, relative to where you ran npm install from.

3. Tell PM2 to run Node-RED

The following command tells PM2 to run Node-RED, assuming /usr/bin/node-red as the location of the node-red command.

The -- argument must appear before any arguments you want to pass to node-red.

pm2 start /usr/bin/node-red -- -v
Note: if you are running on a device like the Raspberry Pi or BeagleBone Black that have a constrained amount of memory, you must pass an additional argument:
pm2 start /usr/bin/node-red --node-args="--max-old-space-size=128" -- -v
Note: if you want to run as the root user, you must use the `--userDir` option to specify where Node-RED should store your data.
This will start Node-RED in the background. You can view information about the process and access the log output using the commands:

pm2 info node-red
pm2 logs node-red
More information about managing processes under PM2 is available here.

4. Tell PM2 to run on boot

PM2 is able to generate and configure a startup script suitable for the platform it is being run on.

Run these commands and follow the instructions it provides:

pm2 save
pm2 startup
for newer Linux systems that use systemd use

pm2 startup systemd
Temporary Note: There's an open issue on PM2 on GitHub which highlights an issue that has been introduced recently. Linux users need to manually edit the generated `/etc/init.d/pm2-init.sh` file and replace
export PM2_HOME="/root/.pm2"
to point at the correct directory, which would be like:
export PM2_HOME="/home/{youruser}/.pm2"


Output:
sudo env PATH=$PATH:/usr/local/bin /usr/local/lib/node_modules/pm2/bin/pm2 startup systemd -u nvidia --hp /home/nvidia


5. Reboot

Finally, reboot and check everything starts as expected.


ROS ROBOT UPSTART

sudo apt install ros-kinetic-robot-upstart













