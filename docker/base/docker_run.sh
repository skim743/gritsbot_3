#!/bin/bash

# Use the detect_serial module to get the correct serial port
docker run -d --restart=always \
	--name firmware \
	--net host \
	--device $(python3 -m gritsbot.utils.detect_serial):/dev/ttyACM0 \
	robotarium:firmware
