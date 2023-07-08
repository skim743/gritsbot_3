# Setup Process for the gritsbot\_2

# 1 - Program Teensy
1. Connect a Teensy to the main computer using a micro-USB cable. 
2. Open Arduino IDE (by either entering 'arduino' on a terminal or clicking 'Show Applications' icon on the bottom left corner of the screen)
3. Open 'defaultOperation.ino' located in '~/git/gritsbotx/firmware/teensyCode' directory.
4. Click the upload icon (right arrow icon).

# 2 - Make the Base Image for Raspberry Pi

This section details how to make the base image.  Relatively few changes are made to keep the image small.  Once the changes in this section have been made, copy the new image to an SD card and use that as the base image.

## 1 - Load the RPi image onto an SD card
1. Install and run the Raspberry Pi Imager (https://www.raspberrypi.com/software/).
2. For 'Operating System,' select 'Raspberry Pi OS (other)' and select 'Raspberry Pi OS Lite (32-bit).'
3. For 'Storage,' choose the SD card to be used.
4. Before clicking the 'WRITE' button, click on the gear icon below the 'WRTIE' button to open the 'Advanced options.'
5. Check 'Enable SSH' and 'Use password authentication.'
6. Then, check 'Set username and password' and type 'pi' for the 'Username' and 'raspberry' for 'Password.'
7. Next, check 'Configure wireless LAN' and enter 'SSID' and 'Password' for the router.
8. Change 'Wireless LAN country' to 'US.' Click 'SAVE' and click 'WRITE' button to start loading the image to the SD card.

## 2 - Register the RPi as a Robot
Assign an unallocated robot index for the MAC address of the Raspberry Pi. Then, on the main computer,
1. Add/Replace the MAC address of the Raspberry Pi in '~/git/gritsbot_3/config/mac_list.json'
2. Add/Replace the robot ID in '~/git/vicon_tracker_python/config/node_desc_tracker.json'
3. Add/Replace the robot ID in '~/git/robotarium_matlab_backend/config/node_desc_api.json'
4. Build the firmware Docker image by running
```
cd ~/git/gritsbot_3/docker/
./docker_build.sh <192.168.1.8> <1884>
```
- When making making multiple robots, register the MAC address of all new robots in the files listed above before building the firmware image. Otherwise, the firmware needs to be built as many as the number of new robots.
- When assigning an index to a new robot,  assign the number engraved on the Vicon hat plate. Make sure not to use any numbers that are assigned to existing robots.

## 3 - Setup the RPi
1. Eject the card from the computer and insert it onto a Raspberry Pi and power it up.
It should automatically connect to the wifi. The Pi needs some time to boot for the first time. The boot up process can be visually inspected by plugging the Raspberry Pi to a monitor through a mini HDMI cable. When the Pi completes the booting process it will prompt a login. If the Pi shows a blue screen prompting to enter a new username, something is wrong with uploading the image to Pi, and the Raspbian OS needs to be reinstalled. Before loading another image to the SD card, make sure to re-type the passwords for the Pi and the Wifi. The Raspberry Pi Imager seems to be ruining the passwords saved in the advanced setting when the program is restarted.
2. Navigate to the router settings page by navigating to '192.168.1.1' using a web browser (admin credential for the router is currently saved in Firefox). The new Pi will appear as 'RASPBERRYPI.' 
3. Click on it to look up its IP address and MAC address. 
4. After looking up the IP address of the new Pi, ssh to it by
```
ssh pi@<IP-address-of-Pi>
```
When promted to enter password, type 'raspberry'

5. Open /boot/config.txt file by

```
sudo nano /boot/config.txt
```

6. Add to /boot/config.txt the text

```
# Disable bluetooth
dtoverlay=pi3-disable-bt
```

# 3 - Automated Setup

This section assumes that you have built a base image as previously detailed.

## 1 - Automatic Installation

1. Copy '.git-credentials' in '\~/git/gritsbot_3/docker' directory and 'setup' in '\~/git/gritsbot_3/setup' directory to 'rootfs/home/pi' directory of the SD card.

2. Connect the Pi and Teensy with a USB cable. If the Pi is not connected with a programmed Teensy as instructed in Step 1, the firmware will not successfully be started by the setup script in the next step.

3. On the Pi, run the setup script with

```
./setup
```

This can be done either by directly on the Pi by connecting a mini HDMI cable and a keyboard to the Pi or SSHing to the Pi as instructed in Step 2.3.

## 2 - Network Host Name Change
This process changes the name of the Raspberry Pi on the network. This helps to identify each robot easily on the router page (192.168.1.1). See Step 2.3, if you forgot how to access the router page.

After the setup script is completed,
Run
```
sudo raspi-config
```
on the Raspberry Pi, and change the Host Name to 'robot#' where # is the new robot index assigned to the Pi in Step 2.3.

Reboot the Pi to apply the new network host name by selecting 'yes' when the raspi-config asks for a restart, or by using
```
sudo reboot
```

# 4 - Manual Installation

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
