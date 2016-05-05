#!/bin/sh

echo "Starting VNC server with 1024x576 resolution, connect with 192.168.42.1:5901"
vncserver :1 -geometry 1024x576 -depth 24 -dpi 96
