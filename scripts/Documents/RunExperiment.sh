#!/bin/bash

# Create folder with current date and copy scripts from template
now=$(date +%Y-%m-%d_%H-%M-%S)
mkdir -p "/home/pi/Documents/experiments/${now}"
cp -r /home/pi/Documents/template/. "/home/pi/Documents/experiments/${now}/"

# Blank screen switched off (default 600s)
xset s off
xset -dpms 

# Start python environment
cd /home/pi/Documents
source venv_2/bin/activate

# Run EduCrys
cd "/home/pi/Documents/experiments/${now}/"
python3 democz.py

