# Setup Process for the gritsbot\_2

# 1 - Upload the Code to Teensy
Connect a Teensy to the main computer using a micro-USB cable. Open Arduino IDE (by either entering "arduino" on a terminal or clicking "Show Applications" icon on the bottom left corner of the screen) and open "defaultOperation.ino. Then, click the upload icon (right arrow icon).

When adding/replacing robots,
1. Add/Replace the MAC address of the robots in "~/git/gritsbot_2/config/mac_list.json"
2. 


# 1 - Making the Base Image

This section details how to make the base image.  Relatively few changes are made to keep the image small.  Once the changes in this section have been made, copy the new image to an SD card and use that as the base image.

## 1 - Load the RPi image onto an SD card
Install (https://www.raspberrypi.com/software/) and run the Raspberry Pi Imager. For 'Operating System,' select 'Raspberry Pi OS (other)' and select 'Raspberry Pi OS Lite (32-bit).' For 'Storage,' choose the SD card to be used. Before clicking the 'WRITE' button, click on the gear icon below the 'WRTIE' button to open the 'Advanced options.' Check 'Enable SSH' and 'Use password authentication.' Then, check 'Set username and password' and type 'pi' for the 'Username' and 'raspberry' for 'Password.' Next, check 'Configure wireless LAN' and type 'RobotEcologyLab' for 'SSID' and 'NoMoGrits4Me' for 'Password.' Change 'Wireless LAN country' to 'US.' Click 'SAVE' and click 'WRITE' button to start loading the image to the SD card.

## 2 - Disable Unused Services

Boot the Pi and ssh to it. You can lookup the IP address of the Pi through the lab router. Navigate to the router settings page by navigating to '192.168.1.1' using a web browser (admin credential for the router is currently saved in Firefox). The new Pi will appear as 'RASPBERRYPI.' Click on it to look up its IP address. After looking up the IP address of the new Pi, ssh to it by
```
ssh pi@<IP-address-of-Pi>
```
When promted to enter password, type 'raspberry'

Open /boot/config.txt file by

```
sudo nano /boot/config.txt
```

Add to /boot/config.txt the text

```
# Disable bluetooth
dtoverlay=pi3-disable-bt
```

# 2 - Automated Setup

This section assumes that you have built a base image as previously detailed.

## 1 - Automatic Installation

To install the firmware automatically, run the setup script by

```
./setup
```

## 2 - Manual Installation

This section details the installation process for the firmware.  This process can be made automatic via the setup scripts.

### 1 Remove Unused Services

Remove plymouth with 

```
sudo apt-get purge --remove plymouth
```

Disable unused services with 

```
sudo systemctl disable triggerhappy.service
sudo systemctl disable hciuart.service
sudo systemctl disable keyboard-setup.service
sudo systemctl disable dphys-swapfile.service
```

### 2 - Install Docker

This section follows from the official (Docker)[https://docs.docker.com/install/linux/docker-ce/ubuntu/].  First, remove old versions of Docker.

```
sudo apt-get remove docker docker-engine docker.io
```

Next, install Docker using the convenience script.

```
curl -fsSL get.docker.com -o get-docker.sh && export VERSION=23.0 && sh get-docker.sh
```

Now tie Docker to the pi user so that we don't need sudo to use Docker.

```
sudo usermod -aG docker pi
```

### 3 - Clone Git Repos and install Deps

Install pip for python3

```
sudo apt-get install python3-pip
```

To clone the firmware, run
```
sudo apt-get install git
git clone https://github.com/robotarium/gritsbot_2
```

Also, install the MAC discovery repository
```
git clone https://github.com/robotarium/mac_discovery
```

as well as the python serial library used to communicate to the robot.

```
python3 -m pip install pyserial
```

### 4 - WiFi Power Management

Turn off power management by adding the line
```
/sbin/iw dev wlan0 set power_save off
```

in the file /etc/rc.local.  This line disables WiFi power management on boot.

### 5 - Start necessary containers

From wherever the git repository is cloned, run 
```
cd <path_to_gritsbot_2_repo>/docker
./docker_run.sh
./docker_watch.sh
```
which will permanently start a Docker container running the firmware and the watchtower container.  Watchtower watches containers and automatically updates them from Dockerhub.  This watchtower instance
checks and updates **all running containers**, so this instance will also update the MAC container as well.

Start MAC discovery as well with
```
cd <path_to_mac_discovery_repo>/docker
./docker_run.sh
```
