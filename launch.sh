#!/bin/bash

echo "$(date): Starting the Fixedwing Mission server..."

while true; do
  python /home/pi/fixedwing-mission/main.py --port /dev/serial/by-id/usb-ArduPilot_fmuv2_2A004C001951303330343831-if00
  if [ $? -ne 0 ]; then
    echo "$(date): Fixedwing mission script exited with status code $?, restarting..."
  else
    break
  fi
done
