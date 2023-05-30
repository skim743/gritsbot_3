#!/bin/bash

docker run -d ---restart=always \
	   -p 192.168.1.5:5000:5000 \
	   --name=registry \
	   registry:2
