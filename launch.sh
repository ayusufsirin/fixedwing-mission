#!/bin/bash

while true; do
  python main.py --port /dev/serial/by-id/usb-ArduPilot_fmuv2_2A004C001951303330343831-if00
  if [ $? -ne 0 ]; then
    echo "Fixedwing mission script exited with status code $?, restarting..."
  else
    break
  fi
done